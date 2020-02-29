#pragma once

#include <stdbool.h>

struct message {

};

void send_msg(struct lora_modem *lora, struct message *msg);

void listen(struct lora_modem *lora);

struct message* recv_msg(struct lora_modem *lora);

bool msg_rcvd(struct lora_modem *lora);

