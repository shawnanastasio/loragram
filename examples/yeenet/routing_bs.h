#include <stdbool.h>

#include "lora.h"

#define PKT_LEN 255

struct packet {
    uint8_t src;
    uint8_t dest;
    uint8_t type;
    uint8_t payload[PKT_LEN - 3];
};

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

bool rcvd_pkt_dest(struct lora_modem *lora, struct packet *rcvd_pkt);

bool rcvd_pkt_state(struct lora_modem *lora, struct packet *rcvd_pkt, uint8_t pkt_id);

void retransmit(struct lora_modem *lora, struct packet *pkt);

void send_ack(struct lora_modem *lora, struct packet *rcvd_packet);

bool timeout();

