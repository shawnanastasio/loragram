#pragma once

#include <avr/io.h>

typedef enum {
	SCK_DIV_BY_2,
	SCK_DIV_BY_4,
	SCK_DIV_BY_8,
	SCK_DIV_BY_16,
	SCK_DIV_BY_32,
	SCK_DIV_BY_64,
	SCK_DIV_BY_128
} sck_divider_t;


typedef enum {
    LOW_WHEN_IDLE = 0,
    HIGH_WHEN_IDLE = 1
} clock_polarity_t;

typedef enum {
    LSB_FIRST = 1,
    MSB_FIRST = 0
} data_order_t;

typedef enum {
    SAMPLE_ON_LEADING = 0,
    SAMPLE_ON_TRAILING = 1
} clock_phase_t;

typedef enum {
	SPI_MODE_0,
	SPI_MODE_1,
	SPI_MODE_2,
	SPI_MODE_3
} spi_mode_t;

void spi_manual_enable(sck_divider_t divider, clock_polarity_t cpol_bit,
                        data_order_t dord_bit, clock_phase_t cpha_bit);
void spi_auto_enable(spi_mode_t mode, sck_divider_t divider);
void spi_disable();

static inline uint8_t spi_read() {
	uint8_t read = SPDR;
	return read;
}

static inline void spi_write(uint8_t write_me) {
	SPDR = write_me;
}

static inline uint8_t spi_cycle(uint8_t write_me) {
		spi_write(write_me); //start transaction and clock out write_me
		while(!(SPSR & (1<<SPIF))); //busywait for transaction completion
		return spi_read(); //report contents of fifo buffer
}
