#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "lora.h"

void send_msg(struct lora_modem *lora, uint8_t *msg);

void signal_listen(struct lora_modem *lora);

uint8_t *recv_msg(struct lora_modem *lora);

bool msg_rcvd(struct lora_modem *lora);
