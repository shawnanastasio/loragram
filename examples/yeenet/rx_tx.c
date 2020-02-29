#include <stdint.h>
#include <stdbool.h>

#include "lora.h"
#include "rx_tx.h"

void send_msg(struct lora_modem *lora, struct message *msg) {

}

void listen(struct lora_modem *lora) {

}

struct message* recv_msg(struct lora_modem *lora) {
    struct message *msg;
    return msg;
}

bool msg_rcvd(struct lora_modem *lora) {
    return true;
}

