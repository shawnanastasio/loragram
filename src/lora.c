#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>
#include <avr/interrupt.h>

#include "libminiavr.h"
#include "spi_master.h"
#include "lora.h"
#include "sx127x.h"
#include "lora_config.h"


#define ARRAY_SIZE(x) (sizeof((x)) / sizeof(*(x)))

#define FXOSC 32000000
#define FREQ_TO_REG(in_freq) ((uint32_t)(( ((uint64_t)in_freq) << 19) / FXOSC))
#define REG_TO_FREQ(in_reg) ((uint32_t)((FXOSC*in_reg) >> 19))

// LoRA interrupt routine
static struct lora_modem *int0_modem;
ISR(INT0_vect,ISR_NOBLOCK) {
    // Read irq_data register
    int0_modem->irq_data = lora_read_reg(int0_modem, LORA_REG_IRQFLAGS);

    // Clear IRQ flag. Needs to be done twice for some reason (hw errata?)
    lora_write_reg(int0_modem, LORA_REG_IRQFLAGS, 0xFF);
    lora_write_reg(int0_modem, LORA_REG_IRQFLAGS, 0xFF);
	int0_modem->irq_seen = false; //report new irq data
   
    
    switch (int0_modem->irq_mode){
		case RX_DONE:
			(*(int0_modem->post_rx))(int0_modem);
			break;
		case TX_DONE:
			(*(int0_modem->post_tx))(int0_modem);
			break;
	}
}

//
// High-level LoRA API
//

//helper function for setting modulation registers--may be called by user for on-the-fly modulation changes
void lora_config_modulation(struct lora_modem *lora, struct modulation_config *modulation){
	uint8_t temp = 0;
	temp |= ((uint8_t) modulation->bandwidth) << 4; //set top nibble for bandwidth
	temp |= ((uint8_t) modulation->coding_rate) << 1; //set top 3 bits of bottom nibble for coding rate
	if (!(modulation->header_enabled)) temp |= 1; //set bottom bit to indicate header mode
	lora_write_reg(lora,LORA_REG_MODEM_CONFIG_1,temp);

	temp = 0;
	temp |= (((uint8_t) modulation->spreading_factor) + 6) << 4; //set spreading factor
	if (modulation->crc_enabled) temp |= (1<<2); //set crc enable bit
	lora_write_reg(lora,LORA_REG_MODEM_CONFIG_2,temp);

	//set preamble length
	uint8_t top = (uint8_t) modulation->preamble_length;
	uint8_t bot = (uint8_t) (modulation->preamble_length >> 8);
	lora_write_reg(lora,LORA_REG_PREAMBLE_MSB,top);
	lora_write_reg(lora,LORA_REG_PREAMBLE_LSB,bot);
    
    //handle settings required if on SF6
    if(modulation->spreading_factor==SF6) {
		lora_write_reg(lora,LORA_REG_DETECTION_THRESHOLD, 0x0C);
		lora_write_reg(lora,LORA_REG_DETECT_OPTIMIZE, 0x05);
	} else {
		lora_write_reg(lora,LORA_REG_DETECTION_THRESHOLD, 0x0A);
		lora_write_reg(lora,LORA_REG_DETECT_OPTIMIZE, 0x03);
	}
    
    //set payload length
    if(modulation->payload_length==0) modulation->payload_length = 1; //prevent 0 from being written to the register
    lora_write_reg(lora,LORA_REG_PAYLOAD_LENGTH,modulation->payload_length);
    
    //inform lora struct of current modulation
    lora->modulation = modulation;
}

bool lora_setup(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin,struct modulation_config *modulation, void (*post_rx)(struct lora_modem*),void (*post_tx)(struct lora_modem*)) {
    lora->rst_pin = rst_pin;
    lora->cs_pin = cs_pin;
    lora->irq_pin = irq_pin;
    lora->irq_data = 0;
    lora->irq_seen = true;
    lora->post_rx = post_rx;
    lora->post_tx = post_tx;

    // Configure pin modes
    pin_mode(rst_pin, OUTPUT);
    pin_mode(cs_pin, OUTPUT);
    pin_mode(irq_pin, INPUT);
    
    //reset the chip
    digital_write(rst_pin, 0);
    _delay_ms(10);
    digital_write(rst_pin, 1);

    _delay_ms(500);
    
	// TODO: Allow user-configurable interrupt pins
    // For now, assume INT0 is used
    int0_modem = lora;
    EICRA = 0b00000011; // Rising edge of INT0 generates an interrupt
    EIMSK = 0b00000001; // Enable INT0
    
	// Enable SPI
	spi_auto_enable(SPI_MODE_0, SCK_DIV_BY_32);
    
	uint8_t lora_mode;
	lora_mode = lora_read_reg(lora,LORA_REG_OP_MODE);
	//fprintf(&serial0->iostream, "test point\r\n");
	fprintf(&serial0->iostream,"lora mode before setup: %x\r\n",lora_mode);
	//test point will execute here
	
	
    // Change mode LORA + SLEEP 
    //NOTE:Module ABSOLUTELY MUST be put into sleep mode when activated first. Write to mode register will not work otherwise.
    if (!lora_write_reg_and_check(lora, LORA_REG_OP_MODE, MODE_LORA | MODE_SLEEP, true)) {
#ifdef DEBUG
        fprintf(&serial0->iostream,"Failed to enter LORA+SLEEP mode!\r\n");
        
#endif
        
        return false;
    }
    //test point will not execute here
    _delay_ms(5000);
	fprintf(&serial0->iostream, "test point\r\n");
	
    // Set packet size. If headers are enabled, this will be overwritten as needed
    if (!lora_write_reg_and_check(lora, LORA_REG_PAYLOAD_LENGTH, lora->modulation->payload_length, false) ||
        !lora_write_reg_and_check(lora, LORA_REG_MAX_PAYLOAD_LENGTH, LORA_MAX_PAYLOAD_SIZE, false)) {
#ifdef DEBUG
        fprintf(&serial0->iostream, "Failed to set payload size!\r\n");
#endif
        return false;
    }
    
    //fprintf(&serial0->iostream, "test point\r\n");
#ifdef DEBUG
	fprintf(&serial0->iostream, "meme configuring radio settings\r\n");
#endif
	//assume max power settings
	//Configure PA_BOOST with max power
    lora_write_reg_and_check(lora,LORA_REG_PA_CONFIG, 0x8f,true);
    // Enable overload current protection with max trim
    lora_write_reg_and_check(lora,LORA_REG_OCP, 0x3f,true);
    // Set RegPaDac to 0x87, increase power?
    lora_write_reg_and_check(lora,REG_PA_DAC, 0x87,true);
    
    // Set the FIFO RX/TX pointers to 0
    lora_write_reg(lora,LORA_REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(lora,LORA_REG_FIFO_RX_BASE_ADDR, 0x00);
    
    // Disable frequency hopping
    lora_write_reg(lora,LORA_REG_HOP_PERIOD, 0x00);
    
    //Set frequency
    lora_write_reg(lora,LORA_REG_FR_MSB, (FREQ_TO_REG(915000000) >> 16) & 0b11111111);
    lora_write_reg(lora,LORA_REG_FR_MID, (FREQ_TO_REG(915000000) >> 8) & 0b11111111);
    lora_write_reg(lora,LORA_REG_FR_LSB, FREQ_TO_REG(915000000) & 0b11111111);
    
    fprintf(&serial0->iostream,"starting modulation config\r\n");
	lora_config_modulation(lora,modulation);
	

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

void lora_set_payload(struct lora_modem *lora, uint8_t msg[LORA_MAX_PAYLOAD_SIZE], uint8_t length) {
	lora_write_reg(lora,LORA_REG_FIFO_TX_BASE_ADDR,0); //set tx base address, not sure if this is neccessary
	if(lora->modulation->header_enabled){ //explicit header mode
		lora_write_reg(lora,LORA_REG_PAYLOAD_LENGTH, length); //inform lora of payload length
	} 
	else{ //fixed length, implicit mode
#ifdef DEBUG
		if(length>lora->modulation->payload_length){
			fprintf(&serial0->iostream,"input to load_payload too long--excess will not be transmitted");
		}
#endif	
	}
	uint8_t old_reg = lora_read_reg(lora,LORA_REG_OP_MODE); //save the old register
	lora_write_reg(lora, LORA_REG_OP_MODE, MODE_LORA | MODE_STDBY); //put the lora into standby
    lora_write_fifo(lora, msg, length, 0); //write the data to the fifo
    lora_write_reg(lora, LORA_REG_OP_MODE, old_reg); //return the old mode
}

void lora_transmit(struct lora_modem *lora) {
    // Configure DIO0 to interrupt on TXDONE, switch to TX mode
    lora_write_reg(lora, REG_DIO_MAPPING_1, 0x40 /* TXDONE */);
    lora->irq_mode = TX_DONE; //inform the lora_modem object of its interrupt status
    lora->irq_seen = true; //reset irq flag
    lora_write_reg_and_check(lora, LORA_REG_OP_MODE, MODE_LORA | MODE_STDBY,true); //enter standby mode before entering TX mode
    lora_write_reg_and_check(lora, LORA_REG_OP_MODE, MODE_LORA | MODE_TX,true);
    /*
    // Wait for IRQ
    while (lora->irq_seen); 
    bool ret = lora->irq_data == LORA_MASK_IRQFLAGS_TXDONE;
    lora->irq_seen = true;
    */

}

void lora_listen(struct lora_modem *lora) {
    lora_write_reg(lora, REG_DIO_MAPPING_1, 0x00/* RXDONE */);
    lora->irq_mode = RX_DONE;
    lora_write_reg_and_check(lora,LORA_REG_OP_MODE,MODE_LORA | MODE_RXCON,true);
    lora->irq_seen = true; //reset irq flag
}

enum lora_fifo_status lora_get_payload(struct lora_modem *lora, uint8_t buf_out[LORA_MAX_PAYLOAD_SIZE], uint8_t *length) {
	
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
    if(lora->modulation->header_enabled) {
		*length = lora_read_reg(lora, LORA_REG_RX_NB_BYTES); //if headers are enabled, pull the payload length
	} else {
		*length = lora->modulation->payload_length; //if header are disabled, assume payload length from struct
	}
    lora_read_fifo(lora, buf_out, *length, ptr);
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
#ifdef DEBUG
	//fprintf(&serial0->iostream,"Writing: %x to register: %x\r\n",val,reg);
#endif
    lora_write_reg(lora, reg, val);

    // Delay
    if (delay)
        _delay_ms(1000);

    //read reg
    uint8_t new_val = lora_read_reg(lora, reg);
#ifdef DEBUG
	//fprintf(&serial0->iostream,"Read: %x from register: %x\r\n",new_val,reg);
#endif

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

bool modem_is_clear(struct lora_modem *lora){
	uint8_t modem_status = lora_read_reg(lora, LORA_REG_MODEMSTAT); //pull the modem status register
	return ( (LORA_MASK_MODEMSTAT_CLEAR & modem_status) == LORA_MASK_MODEMSTAT_CLEAR); //return true if modem clear bit is set
}

