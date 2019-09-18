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

#ifndef LIBMINIAVR_PRIV
#error "This file must not be included directly. Use libminiavr.h"
#endif

#ifndef LIBMINIAVR_BOARD_ARDUINO_UNO_H
#define LIBMINIAVR_BOARD_ARDUINO_UNO_H

#ifdef ARDUINO_AVR_UNO

const MAYBE_STATIC struct pin_mapping libminiavr_board_pins[] = {
    /* UNO pins 0-7 are PORTD */
    {(uint16_t)&PORTD, 1 << 0},
    {(uint16_t)&PORTD, 1 << 1},
    {(uint16_t)&PORTD, 1 << 2},
    {(uint16_t)&PORTD, 1 << 3},
    {(uint16_t)&PORTD, 1 << 4},
    {(uint16_t)&PORTD, 1 << 5},
    {(uint16_t)&PORTD, 1 << 6},
    {(uint16_t)&PORTD, 1 << 7},

    /* UNO pins 8-13 are PORTB */
    {(uint16_t)&PORTB, 1 << 0},
    {(uint16_t)&PORTB, 1 << 1},
    {(uint16_t)&PORTB, 1 << 2},
    {(uint16_t)&PORTB, 1 << 3},
    {(uint16_t)&PORTB, 1 << 4},
    {(uint16_t)&PORTB, 1 << 5},
};

#endif /* ARDUINO_AVR_UNO */

#endif /* LIBMINIAVR_BOARD_ARDUINO_UNO_H */
