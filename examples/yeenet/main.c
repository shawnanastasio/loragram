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

    lora_write_reg(&lora0, LORA_REG_OP_MODE, MODE_LORA | MODE_SLEEP);
    _delay_ms(10);
    //uint8_t new_mode = lora_read_reg(&lora0, LORA_REG_OP_MODE);

    for (;;) {
        uint8_t val = lora_read_reg(&lora0, 0x01);
        fprintf(&serial0->iostream, "val of 0x01: %x\r\n", val);
        _delay_ms(1000);
    }
}
