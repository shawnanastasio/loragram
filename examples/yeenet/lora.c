#include <stdint.h>
#include <stdbool.h>

#include "lora.h"

#include "libminiavr.h"
#include "spi_master.h"

bool lora_setup(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin) {
    // Configure pin modes
    lora->rst_pin = rst_pin;
    lora->cs_pin = cs_pin;
    lora->irq_pin = irq_pin;
    pin_mode(rst_pin, OUTPUT);
    pin_mode(cs_pin, OUTPUT);
    pin_mode(irq_pin, INPUT);
}


bool lora_write_fifo(struct lora_modem *lora, uint8_t* buf, uint8_t len, uint8_t offset) {
     // Some assertions to check input data in DEBUG mode
 
 
     // Update modem's FIFO address pointer
     digital_write(lora->cs_pin, 0);
     spi_cycle(LORA_REG_FIFO_ADDR_PTR | WRITE_MASK);
     spi_cycle(offset);
     digtal_write(lora->cs_pin, 1);
     //assume compiler is not good enough to emit an sbi instruction for the digital_writes  
     // Write data to FIFO.
     digtal_write(-lora->cs_pin, 0);
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
