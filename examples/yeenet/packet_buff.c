#include <stddef.h>
#include <string.h>

#include "rx_tx.h"
#include "packet_buff.h"

uint8_t *msg_to_b(struct packet *pkt) {
    static uint8_t msg[PKT_LEN];
    msg[0] = pkt->src;
    msg[1] = pkt->dest;
    msg[2] = pkt->type;
    memcpy(&msg[3], pkt->payload, PKT_LEN - 3);
    return msg;
}

struct packet *b_to_msg(uint8_t *msg) {
    static struct packet *pkt;
    pkt->src = msg[0];
    pkt->dest = msg[1];
    pkt->type = msg[2];
    memcpy(pkt->payload, msg + 3, PKT_LEN - 3);
    return pkt;
}

struct packet *dply_pkt(struct lora_modem *lora) {
    if (msg_rcvd(lora)) {
        struct packet *pkt = b_to_msg(recv_msg(lora));
        return pkt;
    }
    return NULL;
}

void load_pkt(struct lora_modem *lora, struct packet *pkt) {
    send_msg(lora, msg_to_b(pkt));
    signal_listen(lora);
}

