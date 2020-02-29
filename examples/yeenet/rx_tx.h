#pragma once

struct packet {
    uint8_t dest_addr;
    uint8_t pkt_type;
};

bool rx_pkt(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin, struct packet *pkt);

bool tx_pkt(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin, struct packet *pkt);

