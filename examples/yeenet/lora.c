#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>
#include <avr/interrupt.h>

#include "libminiavr.h"
#include "spi_master.h"
#include "lora.h"


#define ARRAY_SIZE(x) (sizeof((x)) / sizeof(*(x)))

#define FXOSC 32000000
#define FREQ_TO_REG(in_freq) ((uint32_t)(( ((uint64_t)in_freq) << 19) / FXOSC))
#define REG_TO_FREQ(in_reg) ((uint32_t)((FXOSC*in_reg) >> 19))

// LoRA interrupt routine
static struct lora_modem *int0_modem;
ISR(INT0_vect) {
    // Read irq_data register
    int0_modem->irq_data = lora_read_reg(int0_modem, LORA_REG_IRQFLAGS);

    // Clear IRQ flag. Needs to be done twice for some reason (hw errata?)
    lora_write_reg(int0_modem, LORA_REG_IRQFLAGS, 0xFF);
    lora_write_reg(int0_modem, LORA_REG_IRQFLAGS, 0xFF);

    int0_modem->irq_seen = false;
}

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
    //{LORA_REG_PAYLOAD_LENGTH, 0x0f},
    //{LORA_REG_MAX_PAYLOAD_LENGTH, 0x0f},

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

const static uint8_t DEFAULT_MODEM_CONFIG[][2] = {
    // Configure PA_BOOST with max power
    {LORA_REG_PA_CONFIG, 0x8f},

    // Enable overload current protection with max trim
    {LORA_REG_OCP, 0x3f},

    // Set RegPaDac to 0x87 (requried for SF=6)
    {REG_PA_DAC, 0x87},

    // Set the FIFO RX/TX pointers to 0
    {LORA_REG_FIFO_TX_BASE_ADDR, 0x00},
    {LORA_REG_FIFO_RX_BASE_ADDR, 0x00},

    // Set RegModemConfig1 as follows:
    // Bw = 0b1001 (500kHz)
    // CodingRate = 0b001 (4/5)
    // ImplicitHeader = 1
    {LORA_REG_MODEM_CONFIG_1, 0x93},

    // Set RegModemConfig2 as follows:
    // SpreadingFactor = 6
    // TxContinuousMode = 0
    // RxPayloadCrcOn = 1
    {LORA_REG_MODEM_CONFIG_2, 0x64},

    // Set preamble length to 8
    {LORA_REG_PREAMBLE_MSB, 0x00},
    {LORA_REG_PREAMBLE_LSB, 0x08},

    // Set payload length and max length to 255
    //{LORA_REG_PAYLOAD_LENGTH, 0x0f},
    //{LORA_REG_MAX_PAYLOAD_LENGTH, 0x0f},

    // Disable frequency hopping
    {LORA_REG_HOP_PERIOD, 0x00},

    // Set DetectionThreshold to 0xC (for SF = 6)
    {LORA_REG_DETECTION_THRESHOLD, 0x0c},

    // Set DetectionOptimize to 0x5 (for SF = 6)
    {LORA_REG_DETECT_OPTIMIZE, 0x05},

    // Set the frequency to 915MHz
    // We can use FREQ_TO_REG() here because it is declared as `constexpr`
    // and can therefore be evaluated at compile-time
    {LORA_REG_FR_MSB, (FREQ_TO_REG(915000000) >> 16) & 0b11111111},
    {LORA_REG_FR_MID, (FREQ_TO_REG(915000000) >> 8) & 0b11111111},
    {LORA_REG_FR_LSB, FREQ_TO_REG(915000000) & 0b11111111},
};

//
// High-level LoRA API
//

#define CUR_MODEM_CONFIG DEFAULT_MODEM_CONFIG

bool lora_setup(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin) {
    lora->rst_pin = rst_pin;
    lora->cs_pin = cs_pin;
    lora->irq_pin = irq_pin;
    lora->irq_data = 0;
    lora->irq_seen = true;

    // Configure pin modes
    pin_mode(rst_pin, OUTPUT);
    pin_mode(cs_pin, OUTPUT);
    pin_mode(irq_pin, INPUT);
    digital_write(rst_pin, 1);

    // TODO: Allow user-configurable interrupt pins
    // For now, assume INT0 is used
    int0_modem = lora;
    EICRA = 0b00000011; // Rising edge of INT0 generates an interrupt
    EIMSK = 0b00000001; // Enable INT0

    // Enable SPI
    spi_auto_enable(SPI_MODE_0, SCK_DIV_BY_32);

    // Change mode LORA + SLEEP
    if (!lora_write_reg_and_check(lora, LORA_REG_OP_MODE, MODE_LORA | MODE_SLEEP, true)) {
#ifdef DEBUG
        fputs("Failed to enter LORA+SLEEP mode!\r\n", &serial0->iostream);
#endif
        return false;
    }

    // Set packet size
    if (!lora_write_reg_and_check(lora, LORA_REG_PAYLOAD_LENGTH, LORA_PACKET_SIZE, false) ||
        !lora_write_reg_and_check(lora, LORA_REG_MAX_PAYLOAD_LENGTH, LORA_PACKET_SIZE, false)) {
#ifdef DEBUG
        fprintf(&serial0->iostream, "Failed to set payload size!\n");
#endif
        return false;
    }

    // Set registers
    for (size_t i=0; i<ARRAY_SIZE(CUR_MODEM_CONFIG); i++) {
        if (!lora_write_reg_and_check(lora, CUR_MODEM_CONFIG[i][0], CUR_MODEM_CONFIG[i][1], false)) {
#ifdef DEBUG
            fprintf(&serial0->iostream, "Failed to write reg: 0x%x\r\n", CUR_MODEM_CONFIG[i][0]);
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

void lora_load_message(struct lora_modem *lora, uint8_t msg[LORA_PACKET_SIZE]) {
    lora_write_fifo(lora, msg, LORA_PACKET_SIZE, 0);
}

bool lora_transmit(struct lora_modem *lora) {
    // Configure DIO0 to interrupt on TXDONE, switch to TX mode
    lora_write_reg(lora, REG_DIO_MAPPING_1, 0x40 /* TXDONE */);
    lora_write_reg(lora, LORA_REG_OP_MODE, MODE_LORA | MODE_TX);
    uint8_t reg = lora_read_reg(lora,LORA_REG_OP_MODE);
    uint8_t reg2 = lora_read_reg(lora,0x12);

    // Wait for IRQ
    while (lora->irq_seen) fprintf(&serial0->iostream,"waiting...mode:%x irq:%x\r\n",reg,reg2);
    bool ret = lora->irq_data == LORA_MASK_IRQFLAGS_TXDONE;
    lora->irq_seen = true;

    return ret;
}

void lora_listen(struct lora_modem *lora) {
    lora_write_reg(lora, REG_DIO_MAPPING_1, 0x00 /* RXDONE */);
    lora_write_reg(lora,LORA_REG_OP_MODE,MODE_LORA | MODE_RXCON);
    lora->irq_seen = true; //reset irq flag
}

enum lora_fifo_status lora_get_packet(struct lora_modem *lora, uint8_t buf_out[LORA_PACKET_SIZE]) {
    uint8_t data = lora->irq_data;
    if (lora->irq_seen)
        return FIFO_EMPTY;
    lora->irq_seen = true;

    if (!(data & LORA_MASK_IRQFLAGS_RXDONE))
        return FIFO_BAD;

    if (data & LORA_MASK_IRQFLAGS_PAYLOADCRCERROR)
        return FIFO_BAD;

    // FIFO contains valid packet, return it
    uint8_t ptr = lora_read_reg(lora, LORA_REG_FIFO_RX_CUR_ADDR);
    lora_read_fifo(lora, buf_out, LORA_PACKET_SIZE, ptr);
    return FIFO_GOOD;
}

//
// Low-level LoRA API
//

void lora_write_fifo(struct lora_modem *lora, uint8_t* buf, uint8_t len, uint8_t offset) {
    // Some assertions to check input data in DEBUG mode

    // Update modem's FIFO address pointer
    digital_write(lora->cs_pin, 0);
    spi_cycle(LORA_REG_FIFO_ADDR_PTR | WRITE_MASK);
    spi_cycle(offset);
    digital_write(lora->cs_pin, 1);
    //assume compiler is not good enough to emit an sbi instruction for the digital_writes
    // Write data to FIFO.
    digital_write(lora->cs_pin, 0);
    spi_cycle(LORA_REG_FIFO | WRITE_MASK);
    for (uint8_t i=0; i<len; i++) {
        spi_cycle(buf[i]);
    }
    digital_write(lora->cs_pin,1);

}

void lora_read_fifo(struct lora_modem *lora, uint8_t *buf, uint8_t len, uint8_t offset) {

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

void lora_dbg_print_irq(uint8_t data) {
    if(data & LORA_MASK_IRQFLAGS_RXTIMEOUT)
        fputs("RX timed out\r\n", &serial0->iostream);

    if(data & LORA_MASK_IRQFLAGS_RXDONE)
        fputs("RX finished \r\n", &serial0->iostream);

    if(data & LORA_MASK_IRQFLAGS_PAYLOADCRCERROR)
        fputs("CRC error\r\n", &serial0->iostream);

    if(!(data & LORA_MASK_IRQFLAGS_VALIDHEADER))
        fputs("Last header invalid\r\n", &serial0->iostream);

    if(data & LORA_MASK_IRQFLAGS_TXDONE)
        fputs("TX complete\r\n", &serial0->iostream);

    if(data & LORA_MASK_IRQFLAGS_CADDONE)
        fputs("CAD finished\r\n", &serial0->iostream);

    if(data & LORA_MASK_IRQFLAGS_FHSSCHANGECHANNEL)
        fputs("FHSS change channel\r\n", &serial0->iostream);

    if(data & LORA_MASK_IRQFLAGS_CADDETECTED)
        fputs("Channel activity detected\r\n", &serial0->iostream);
}
