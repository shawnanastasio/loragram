/**
 * Copyright 2019 Shawn Anastasio
 *
 * This file is part of libminiavr.
 *
 * libminiavr is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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
 * This file contains an implementation of the famous blinky program.
 * It flashes the on-board LED of an Arduino UNO (Pin 13) at 1Hz.
 */

#include <util/delay.h>

#include "libminiavr.h"

#define LED_PIN 13

int main(void) {
    pin_mode(LED_PIN, OUTPUT);
    for(;;) {
        digital_write(LED_PIN, 1);
        _delay_ms(1000);
        digital_write(LED_PIN, 0);
        _delay_ms(1000);
    }
}
