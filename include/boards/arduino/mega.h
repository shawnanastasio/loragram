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

#ifndef LIBMINIAVR_BOARD_ARDUINO_MEGA_H
#define LIBMINIAVR_BOARD_ARDUINO_MEGA_H

#ifdef ARDUINO_AVR_MEGA

const MAYBE_STATIC struct pin_mapping libminiavr_board_pins[] = {
    /* 1-7 */
    {(uint16_t)&PORTE, 1 << 0},
    {(uint16_t)&PORTE, 1 << 1},
    {(uint16_t)&PORTE, 1 << 4},
    {(uint16_t)&PORTE, 1 << 5},
    {(uint16_t)&PORTG, 1 << 5},
    {(uint16_t)&PORTE, 1 << 3},
    {(uint16_t)&PORTH, 1 << 3},
    {(uint16_t)&PORTH, 1 << 4},

    /* 8-13 */
    {(uint16_t)&PORTH, 1 << 5},
    {(uint16_t)&PORTH, 1 << 6},
    {(uint16_t)&PORTB, 1 << 4},
    {(uint16_t)&PORTB, 1 << 5},
    {(uint16_t)&PORTB, 1 << 6},
    {(uint16_t)&PORTB, 1 << 7},

    /* 14-21 */
    {(uint16_t)&PORTJ, 1 << 1},
    {(uint16_t)&PORTJ, 1 << 0},
    {(uint16_t)&PORTH, 1 << 1},
    {(uint16_t)&PORTH, 1 << 0},
    {(uint16_t)&PORTD, 1 << 3},
    {(uint16_t)&PORTD, 1 << 2},
    {(uint16_t)&PORTD, 1 << 1},
    {(uint16_t)&PORTD, 1 << 0},

    /* 22-37 */
    {(uint16_t)&PORTA, 1 << 0},
    {(uint16_t)&PORTA, 1 << 1},
    {(uint16_t)&PORTA, 1 << 2},
    {(uint16_t)&PORTA, 1 << 3},
    {(uint16_t)&PORTA, 1 << 4},
    {(uint16_t)&PORTA, 1 << 5},
    {(uint16_t)&PORTA, 1 << 6},
    {(uint16_t)&PORTA, 1 << 7},
    {(uint16_t)&PORTC, 1 << 7},
    {(uint16_t)&PORTC, 1 << 6},
    {(uint16_t)&PORTC, 1 << 5},
    {(uint16_t)&PORTC, 1 << 4},
    {(uint16_t)&PORTC, 1 << 3},
    {(uint16_t)&PORTC, 1 << 2},
    {(uint16_t)&PORTC, 1 << 1},
    {(uint16_t)&PORTC, 1 << 0},

    /* 38-53 */
    {(uint16_t)&PORTD, 1 << 7},
    {(uint16_t)&PORTG, 1 << 2},
    {(uint16_t)&PORTG, 1 << 1},
    {(uint16_t)&PORTG, 1 << 0},
    {(uint16_t)&PORTL, 1 << 7},
    {(uint16_t)&PORTL, 1 << 6},
    {(uint16_t)&PORTL, 1 << 5},
    {(uint16_t)&PORTL, 1 << 4},
    {(uint16_t)&PORTL, 1 << 3},
    {(uint16_t)&PORTL, 1 << 2},
    {(uint16_t)&PORTL, 1 << 1},
    {(uint16_t)&PORTL, 1 << 0},
    {(uint16_t)&PORTB, 1 << 3},
    {(uint16_t)&PORTB, 1 << 2},
    {(uint16_t)&PORTB, 1 << 1},
    {(uint16_t)&PORTB, 1 << 0},
};

#endif /* ARDUINO_AVR_MEGA */

#endif /* LIBMINIAVR_BOARD_ARDUINO_MEGA_H */
