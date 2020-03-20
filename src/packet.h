/*
#pragma once

#define MAX_PAYLOAD_SIZE 124
#define PACKET_OVERHEAD 4 //make sure this matches the overhead of the struct
#define MAX_PACKET_SIZE (MAX_PAYLOAD_SIZE + PACKET_OVERHEAD)

struct packet {
    uint8_t src;
    uint8_t dest;
    uint8_t type;
    uint8_t len; //expresses number of bytes in payload
    uint8_t payload[MAX_PAYLOAD_SIZE];
};

enum packet_type {
	NULL_PKT = 0,
    DATA,
    ACK
};

static inline void packet_ackify(struct packet *pkt){
	uint8_t buf;
	buf = pkt->src;
	pkt->src = pkt->dest;
	pkt->dest = buf;
	pkt->type = ACK;
	pkt->len = 0;
}

static inline uint8_t packet_get_length(struct packet *pkt){
	return PACKET_OVERHEAD + pkt->len;
}
*/
