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

int main(void) {
    // Initalize USART0 at 115200 baud
    serial_begin(serial0, 115200);

    // Prompt the user for their name
    serial_write(serial0, "Hello, What's your name? ", 25);

    // Read until a carriage return
    uint8_t in_buf[20];
    uint8_t n = serial_read_until(serial0, in_buf, sizeof(in_buf), '\r');
    in_buf[n - 1] = '\0';

    // Repeatedly greet the user
    for(;;) {
        fprintf(&serial0->iostream, "Hello, %s!\r\n", in_buf);
        _delay_ms(1000);
    }
}
