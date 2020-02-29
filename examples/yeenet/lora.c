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
