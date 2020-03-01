#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "lora.h"

#define BRDCAST 0
#define PKT_LEN 255

struct packet {
    uint8_t dest;
    uint8_t type;
    uint8_t payload[PKT_LEN - 2];
};

uint8_t *msg_to_b(struct packet *pkt);

struct packet *b_to_msg(uint8_t *msg);

struct packet *dply_msg(struct lora_modem *lora);

void load_msg(struct lora_modem *lora, struct packet *pkt);

