#include  <avr/io.h>
#include "libminiavr.h"
#include "spi_master.h"
void spi_manual_enable(sck_divider_t divider, clock_polarity_t cpol_bit,
                        data_order_t dord_bit, clock_phase_t cpha_bit) {
	//configure clock divider for SCK
	switch(divider) {
		case SCK_DIV_BY_2:
			SPSR = SPSR | (1 << SPI2X);
			SPCR = SPCR & ~(1 << SPR0);
			SPCR = SPCR & ~(1 << SPR1);
            break;
		case SCK_DIV_BY_4:
			SPSR = SPSR & ~(1 << SPI2X);
			SPCR = SPCR & ~(1 << SPR0);
			SPCR = SPCR & ~(1 << SPR1);
            break;
		case SCK_DIV_BY_8:
			SPSR = SPSR | (1 << SPI2X);
			SPCR = SPCR & ~(1 << SPR0);
			SPCR = SPCR & ~(1 << SPR1);
            break;
		case SCK_DIV_BY_16:
			SPSR = SPSR & ~(1 << SPI2X);
			SPCR = SPCR | (1 << SPR0);
			SPCR = SPCR & ~(1 << SPR1);
            break;
		case SCK_DIV_BY_32:
			SPSR = SPSR | (1 << SPI2X);
			SPCR = SPCR & ~(1 << SPR0);
			SPCR = SPCR | (1 << SPR1);
            break;
		case SCK_DIV_BY_64:
			#ifdef FORCE_SPI2X
			SPSR = SPSR | (1 << SPI2X);
			SPCR = SPCR | (1 << SPR0);
			SPCR = SPCR | (1 << SPR1);
			#else
			SPSR = SPSR & ~(1 << SPI2X);
			SPCR = SPCR & ~(1 << SPR0);
			SPCR = SPCR | (1 << SPR1);
			#endif
            break;
		case SCK_DIV_BY_128:
			SPSR = SPSR & ~(1 << SPI2X);
			SPCR = SPCR | (1 << SPR0);
			SPCR = SPCR | (1 << SPR1);
            break;
	}

	uint8_t temp_spcr = SPCR & 0x03; //preserve last two bits of spcr
	temp_spcr |= (0 << SPIE); //disable interrupts TODO: implement this functionality
	temp_spcr |= (1 << SPE); //enable SPI
	temp_spcr |= (dord_bit << DORD);//set data direction
	temp_spcr |= (1 << MSTR); //set spi to master mode
	temp_spcr |= (cpol_bit << CPOL); //set clock polarity
	temp_spcr |= (cpha_bit << CPHA); //set clock phase
	SPCR = temp_spcr;
}


void spi_auto_enable(spi_mode_t mode, sck_divider_t divider) {
	switch(mode){
		case SPI_MODE_0:
			spi_manual_enable(divider, LOW_WHEN_IDLE, MSB_FIRST, SAMPLE_ON_LEADING);
            break;
		case SPI_MODE_1:
			spi_manual_enable(divider, LOW_WHEN_IDLE, MSB_FIRST, SAMPLE_ON_TRAILING);
            break;
		case SPI_MODE_2:
			spi_manual_enable(divider, HIGH_WHEN_IDLE, MSB_FIRST, SAMPLE_ON_LEADING);
            break;
		case SPI_MODE_3:
			spi_manual_enable(divider, HIGH_WHEN_IDLE, MSB_FIRST, SAMPLE_ON_TRAILING);
            break;
	}
}

void spi_disable() {
	SPCR = SPCR & ~(1 << SPE);
}
