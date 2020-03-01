#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>

#include "libminiavr.h"
#include "spi_master.h"
#include "lora.h"


#define ARRAY_SIZE(x) (sizeof((x)) / sizeof(*(x)))

#define FXOSC 32000000
#define FREQ_TO_REG(in_freq) ((uint32_t)(( ((uint64_t)in_freq) << 19) / FXOSC))
#define REG_TO_FREQ(in_reg) ((uint32_t)((FXOSC*in_reg) >> 19))

const static uint8_t LONGR_MODEM_CONFIG[][2] = {
    // Configure PA_BOOST with max power
    {LORA_REG_PA_CONFIG, 0x8f},

    // Enable overload current protection with max trim
    {LORA_REG_OCP, 0x3f},

    // Set RegPaDac to 0x87
    {REG_PA_DAC, 0x87},

    // Set the FIFO RX/TX pointers to 0
    {LORA_REG_FIFO_TX_BASE_ADDR, 0x00},
    {LORA_REG_FIFO_RX_BASE_ADDR, 0x00},

    // Set RegModemConfig1 as follows:
    // Bw = 0b1001 (500kHz)
    // CodingRate = 0b100 (4/8)
    // ImplicitHeader = 1
    {LORA_REG_MODEM_CONFIG_1, 0x99},

    // Set RegModemConfig2 as follows:
    // SpreadingFactor = 12
    // TxContinuousMode = 0
    // RxPayloadCrcOn = 1
    {LORA_REG_MODEM_CONFIG_2, 0xc4},

    // Set preamble length to 16
    {LORA_REG_PREAMBLE_MSB, 0x00},
    {LORA_REG_PREAMBLE_LSB, 0x08},

    // Set payload length and max length to 15
    {LORA_REG_PAYLOAD_LENGTH, 0x0f},
    {LORA_REG_MAX_PAYLOAD_LENGTH, 0x0f},

    // Disable frequency hopping
    {LORA_REG_HOP_PERIOD, 0x00},

    // Set DetectionThreshold to 0xA (for SF > 6)
    {LORA_REG_DETECTION_THRESHOLD, 0x0A},

    // Set DetectionOptimize to 0x3 (for SF > 6)
    {LORA_REG_DETECT_OPTIMIZE, 0x03},

    // Set the frequency to 915MHz
    // We can use FREQ_TO_REG() here because it is declared as `constexpr`
    // and can therefore be evaluated at compile-time
    {LORA_REG_FR_MSB, (FREQ_TO_REG(915000000) >> 16) & 0b11111111},
    {LORA_REG_FR_MID, (FREQ_TO_REG(915000000) >> 8) & 0b11111111},
    {LORA_REG_FR_LSB, FREQ_TO_REG(915000000) & 0b11111111},
};

bool lora_setup(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin) {
    // Configure pin modes
    lora->rst_pin = rst_pin;
    lora->cs_pin = cs_pin;
    lora->irq_pin = irq_pin;
    pin_mode(rst_pin, OUTPUT);
    pin_mode(cs_pin, OUTPUT);
    pin_mode(irq_pin, INPUT);
    digital_write(rst_pin, 1);

    // Enable SPI
    spi_auto_enable(SPI_MODE_0, SCK_DIV_BY_32);

    // Change mode LORA + SLEEP
    lora_write_reg_and_check(lora, LORA_REG_OP_MODE, MODE_LORA | MODE_SLEEP, true);
    // Set registers
    for (size_t i=0; i<ARRAY_SIZE(LONGR_MODEM_CONFIG); i++) {
        if (!lora_write_reg_and_check(lora, LONGR_MODEM_CONFIG[i][0], LONGR_MODEM_CONFIG[i][1], 0)) {
#ifdef DEBUG
            fprintf(&serial0->iostream, "Failed to write reg: 0x%x\r\n", LONGR_MODEM_CONFIG[i][0]);
#endif
            return false;
        }
    }
    
#ifdef DEBUG
    fprintf(&serial0->iostream,"Starting RNG generation\n\r");
#endif
    seed_random(lora);
    return true;
}

void seed_random(struct lora_modem *lora) {
    uint32_t new_seed = 0;
    lora_write_reg(lora,LORA_REG_OP_MODE,MODE_LORA | MODE_RXCON);
    for(uint8_t i = 0;i<32;i++){
        uint32_t val = lora_read_reg(lora, LORA_REG_RSSI_WIDEBAND);
        _delay_ms(1);
        new_seed = new_seed | ( (val&0x00000001) << i);   
    }
#ifdef DEBUG
    fprintf(&serial0->iostream, "generated seed %lx\r\n",new_seed);
#endif
    srandom((long) new_seed); 
    lora_write_reg(lora,LORA_REG_OP_MODE,MODE_LORA | MODE_STDBY);
}

uint32_t rand_32() {
    return random();
} 

void lora_write_fifo(struct lora_modem *lora, uint8_t* buf, uint8_t len, uint8_t offset) {
    // Some assertions to check input data in DEBUG mode


    // Update modem's FIFO address pointer
    digital_write(lora->cs_pin, 0);
    spi_cycle(LORA_REG_FIFO_ADDR_PTR | WRITE_MASK);
    spi_cycle(offset);
    digital_write(lora->cs_pin, 1);
    //assume compiler is not good enough to emit an sbi instruction for the digital_writes
    // Write data to FIFO.
    digital_write(-lora->cs_pin, 0);
    spi_cycle(LORA_REG_FIFO | WRITE_MASK);
    for (uint8_t i=0; i<len; i++) {
        spi_cycle(buf[i]);
    }
    digital_write(lora->cs_pin,1);

}

void lora_read_fifo(struct lora_modem *lora, uint8_t *buf, uint8_t len, uint8_t offset){

    // Update modem's FIFO address pointer
    digital_write(lora->cs_pin, 0);
    spi_cycle(LORA_REG_FIFO_ADDR_PTR | WRITE_MASK);
    spi_cycle(offset);
    digital_write(lora->cs_pin,1);

    // Read data from FIFO
    digital_write(lora->cs_pin, 0);
    spi_cycle(LORA_REG_FIFO);
    for (uint8_t i=0; i<len; i++) {
        buf[i] = spi_cycle(0x00);
    }
    digital_write(lora->cs_pin,1);
}

uint8_t lora_read_reg(struct lora_modem *lora, uint8_t reg) {
    digital_write(lora->cs_pin, 0);

    spi_cycle(reg & 0x7F);
    uint8_t ret = spi_cycle(0);

    digital_write(lora->cs_pin, 1);

    return ret;
}

void lora_write_reg(struct lora_modem *lora, uint8_t reg, uint8_t val) {
    digital_write(lora->cs_pin, 0);

    spi_cycle(reg | WRITE_MASK);
    spi_cycle(val);

    digital_write(lora->cs_pin, 1);
}

bool lora_write_reg_and_check(struct lora_modem *lora, uint8_t reg, uint8_t val, bool delay) {
    // Write register
    lora_write_reg(lora, reg, val);

    // Delay
    if (delay)
        _delay_ms(10);
    
    //read reg
    uint8_t new_val = lora_read_reg(lora, reg);

    // Return whether write succeeded
    return val == new_val;
}
