#pragma once

struct lora_modem {
    uint8_t rst_pin;
    uint8_t cs_pin;
    uint8_t irq_pin;
};

bool lora_setup(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin);

