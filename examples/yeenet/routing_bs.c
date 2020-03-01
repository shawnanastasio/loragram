#include <string.h>

#include "packet_buff.h"
#include "routing_bs.h"

bool rcvd_pkt_state(struct lora_modem *lora, struct packet *rcvd_pkt) {
    switch(rcvd_pkt->type) {
        case DATA:
            send_ack(lora, rcvd_pkt);
            break;
        case ACK:
            break;
        case NAK:
            break;
        case NULL_PKT:
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
}

bool timeout() {
    return true;
}

