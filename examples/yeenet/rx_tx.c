#include <stdint.h>
#include <stdbool.h>

#include "lora.h"
#include "rx_tx.h"

void send_msg(struct lora_modem *lora, uint8_t *msg) {

}

void listen(struct lora_modem *lora) {

}

uint8_t* recv_msg(struct lora_modem *lora) {
    static uint8_t msg[255];
    return msg;
}

bool msg_rcvd(struct lora_modem *lora) {
    return true;
}

