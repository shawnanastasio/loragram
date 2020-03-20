/*
#include "packet.h"
#include <avr/io.h>

void packet_set_type(struct packet *pkt, enum packet_type type){
	pkt->type = type;
}

bool packet_chars_to_payload(struct packet *pkt, char *payload, uint8_t payload_len){
	if(payload_len>MAX_PAYLOAD_SIZE) return false;
	
	for(uint8_t i = 0;i<payload_len;i++){
		pkt->payload[i] = payload[i];
	}
	
	return true;
}

void packet_set_dest(struct packet *pkt, uint8_t dest_addr){
	pkt->dest = dest_addr;
}

void packet_set_src(struct packet *pkt, uint8_t src_addr){
	pkt->src = src_addr;
}
*/
