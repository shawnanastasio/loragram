#pragma once

#include <stdbool.h>

struct message {

};

void send_msg(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin, struct message *msg);

struct message* recv_msg(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin);

bool msg_rcvd(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin);

