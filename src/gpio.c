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

#define LIBMINIAVR_DONT_HIDE_ARRAYS
#include "libminiavr.h"

/**
 * A highly optimized pin_mode implementation in AVR assembly.
 * Used as a fallback for when parameters are not known at compile-time
 * and the operation can't be reduced to two instructions.
 */
asm(
".global pin_mode_asm\n"
"pin_mode_asm:\n"
    /* Load array addr into X */
    "ldi r26, lo8(pins)\n"
    "ldi r27, hi8(pins)\n"

    /* Double pin no. to get array offset */
    "lsl r24\n"
    "add r26, r24\n"

    /* Load port and bit into Z, r20 respectively */
    "ld r30, X+\n"
    "clr r31\n"
    "ld r20, X\n"

    /* load DDR value */
    "ld r18, -Z\n"

    /* skip OR if mode is !OUTPUT */
    "tst r22\n"
    "brne 2f\n"

    /* Set output mode, flush, ret */
    "or r18, r20\n"
    "st Z, r18\n"
    "ret\n"

    /* Set input mode */
    "2: com r20\n"
    "and r18, r20\n"

    /* flush DDR */
    "st Z, r18\n" // 15

    /* set pullup if needed */
    "cpi r22, 2\n"
    "brne 3f\n"

    "ldd r18, Z+1\n"
    "com r24\n"
    "or r18, r24\n"
    "std Z+1, r18\n"

    "3: ret\n"
);

/**
 * A highly optimized digital_write implementation in AVR assembly.
 * Used as a fallback for when parameters are not known at compile-time
 * and the operation can't be reduced to one instruction.
 */
asm(
".global digital_write_asm\n"
"digital_write_asm:\n"
    /* Load array addr into X */
    "ldi r26, lo8(pins)\n"
    "ldi r27, hi8(pins)\n"

    /* Double pin no. to get array offset */
    "lsl r24\n"
    "add r26, r24\n"

    /* Load port and bit into Z, r20 respectively */
    "ld r30, X+\n"
    "clr r31\n"
    "ld r20, X\n"

    /* load PORT value */
    "ld r18, Z\n"

    "tst r22\n"
    "breq 2f\n"

    /* HIGH, flush, ret */
    "or r18, r20\n"
    "st Z, r18\n"
    "ret\n"

    /* LOW, flush, ret */
    "2: com r20\n"
    "and r18, r20\n"
    "st Z, r18\n" // 15
    "ret"
);
