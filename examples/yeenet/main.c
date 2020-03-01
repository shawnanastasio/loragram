/**
 * Copyright 2019 Shawn Anastasio
 *
 * This file is part of libminiavr.
 *
 * libminiavr is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * libminiavr is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libminiavr.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * This file contains a short example of using libminiavr's UART
 * facilities with both the low-level I/O API and the C stdio API.
 *
 * After uploading the program, interact with it using GNU screen:
 * $ screen /dev/ttyX 115200 8n1
 */

#include <stdio.h>
#include <string.h>

#include <util/delay.h>

#include "libminiavr.h"
#include "spi_master.h"
#include "lora.h"

#define RST_PIN 9
#define SS_PIN 10
#define IRQ_PIN 2

static struct lora_modem lora0;

int main(void) {
    // Initalize USART0 at 115200 baud
    serial_begin(serial0, 115200);
    lora_setup(&lora0, RST_PIN, SS_PIN, IRQ_PIN);

#if 0
    for (;;) {
        // Transmit a packet
        uint8_t buf[15 + 1] = "no sana no life";
        lora_write_fifo(&lora0, buf, 15, 0);
        lora_write_reg(&lora0, REG_DIO_MAPPING_1, 0x40 /* TXDONE */);
        lora_write_reg(&lora0, LORA_REG_OP_MODE, MODE_LORA | MODE_TX);

        while (lora0.irq_seen);
        fputs("GOT IRQ!\r\n", &serial0->iostream);
        lora_dbg_print_irq(lora0.irq_data);
        lora0.irq_seen = true;

        _delay_ms(1000);
    }
#endif

#if 1
    for (;;) {
        // Transmit packet using HLAPI
        uint8_t buf[15 + 1] = "no sana no life";
        lora_load_message(&lora0, buf);
        lora_transmit(&lora0);

        _delay_ms(1000);
    }
#endif

#if 0
    for (;;) {
        uint8_t buf[16];

        // Receive a packet
        lora_write_reg(&lora0, LORA_REG_OP_MODE, MODE_LORA | MODE_RXCON);
        lora_write_reg(&lora0, REG_DIO_MAPPING_1, 0x00 /* RXDONE */);

        while (lora0.irq_seen);
        fputs("GOT IRQ!\r\n", &serial0->iostream);
        lora_dbg_print_irq(lora0.irq_data);
        lora0.irq_seen = true;

        // Get FIFO ptr and read
        uint8_t ptr = lora_read_reg(&lora0, LORA_REG_FIFO_RX_CUR_ADDR);
        lora_read_fifo(&lora0, buf, 15, ptr);
        buf[15] = 0;

        fprintf(&serial0->iostream, "Got data: %s\r\n", buf);

        _delay_ms(1000);
    }
#endif

#if 0
    for (;;) {
        uint8_t val = lora_read_reg(&lora0, 0x01);
        fprintf(&serial0->iostream, "val of 0x01: %x\r\n", val);
        _delay_ms(1000);
    }
#endif
    for(;;);
}
