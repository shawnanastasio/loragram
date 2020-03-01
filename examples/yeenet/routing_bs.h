#include <stdbool.h>

#include "lora.h"

typedef enum {
    NULL_PKT,
    DATA,
    ACK,
    NAK
} packet_t;

typedef enum {
    IDLE,
    BUSY
} channel_state;

bool rcvd_pkt_state(struct lora_modem *lora, struct packet *rcvd_pkt);

void retransmit(struct lora_modem *lora, struct packet *pkt);

void send_ack(struct lora_modem *lora, struct packet *rcvd_packet);

bool timeout();

