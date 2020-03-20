#pragma once

#define LORA_MAX_PAYLOAD_SIZE 255

enum interrupt_status {
	RX_DONE,
	TX_DONE
};

struct lora_modem {
    uint8_t rst_pin;
    uint8_t cs_pin;
    uint8_t irq_pin;
    bool header_enabled;

	void (*post_rx)(struct lora_modem*);
	void (*post_tx)(struct lora_modem*);
	
	struct modulation_config *modulation;

    volatile uint8_t irq_data;
    volatile bool irq_seen;
    volatile enum interrupt_status irq_mode;
};

// High level API

enum lora_fifo_status {
    FIFO_GOOD,
    FIFO_BAD,
    FIFO_EMPTY
};



bool lora_setup(struct lora_modem *lora, uint8_t rst_pin, uint8_t cs_pin, uint8_t irq_pin,struct modulation_config *modulation, void (*post_rx)(struct lora_modem*),void (*post_tx)(struct lora_modem*));
void lora_set_payload(struct lora_modem *lora, uint8_t msg[LORA_MAX_PAYLOAD_SIZE], uint8_t length);
void lora_transmit(struct lora_modem *lora);
void lora_listen(struct lora_modem *lora);
enum lora_fifo_status lora_get_payload(struct lora_modem *lora, uint8_t buf_out[LORA_MAX_PAYLOAD_SIZE], uint8_t *length);
void configure_modulation(struct lora_modem *lora,struct modulation_config *modulation);

// Low level API
void lora_write_fifo(struct lora_modem *lora, uint8_t *buf, uint8_t len, uint8_t offset);
void lora_read_fifo(struct lora_modem *lora, uint8_t *buf, uint8_t len, uint8_t offset);

uint8_t lora_read_reg(struct lora_modem *lora, uint8_t reg);
void lora_write_reg(struct lora_modem *lora, uint8_t reg, uint8_t val);
bool lora_write_reg_and_check(struct lora_modem *lora, uint8_t reg, uint8_t val, bool delay);

void seed_random(struct lora_modem *lora);
uint32_t rand_32();

void lora_dbg_print_irq(uint8_t data);
bool modem_is_clear(struct lora_modem *lora);
