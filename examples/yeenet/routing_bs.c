#include <string.h>

#include "rx_tx.h"
#include "routing_bs.h"

bool rcvd_pkt_dest(struct lora_modem *lora, struct packet *rcvd_pkt) {
    return true;
}

bool rcvd_pkt_state(struct lora_modem *lora, struct packet *rcvd_pkt, uint8_t pkt_id) {
    switch(rcvd_pkt->type) {
        case DATA:
            send_ack(lora, rcvd_pkt);
            break;
        case ACK:
            pkt_id++;
            break;
        case NAK:

            break;
        default:
            return false;
    }
    return true;
}

void retransmit(struct lora_modem *lora, struct packet *pkt) {

}

void send_ack(struct lora_modem *lora, struct packet *rcvd_pkt) {
    static struct packet *ack_pkt;
    ack_pkt->src = rcvd_pkt->dest;
    ack_pkt->dest = rcvd_pkt->src;
    ack_pkt->type = ACK;
    memset(&(ack_pkt->payload), 0, PKT_LEN - 3);
    send_msg(lora, (uint8_t*) ack_pkt);
}

bool timeout() {
    return true;
}

