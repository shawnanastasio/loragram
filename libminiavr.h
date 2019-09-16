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

#ifndef LIBMINIAVR_H
#define LIBMINIAVR_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>

#ifdef LIBMINIAVR_DONT_HIDE_ARRAYS
#define MAYBE_STATIC
#else
#define MAYBE_STATIC static
#endif

#ifndef __GNUC__
#error "libminiavr is highly dependent on GCC-specific features and won't compile without them"
#endif

#ifndef __OPTIMIZE__
#error "libminiavr relies heavily on compiler optimizations for maximum performance, please enable them."
#endif

#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)

#if GCC_VERSION <= 50100
#warning "Old GCC versions may not optimize some routines properly. Please upgrade."
#endif

struct pin_mapping {
    uint8_t port; /* Data Space Address of PORT */
    uint8_t bit; /* Bitmask that controls pin */
};

/* MCU-specific definitions */
#if defined(__AVR_ATmega328P__)

/**
 * Get the offset of the corresponding DDR/PIN register
 * from a port
 */
#define DDR_FROM_PORT(port_addr) ((uint8_t *)((uint16_t)(port_addr) - 0x1))
#define PIN_FROM_PORT(port_addr) ((uint8_t *)((uint16_t)(port_addr) - 0x2))

/* ATMega328P only has 1 USART */
#define USART_COUNT 1

#else
#error "No support for your MCU, please submit an issue or PR."
#endif

/* Board-specific definitions */
#ifdef ARDUINO_AVR_UNO
const MAYBE_STATIC struct pin_mapping pins[] = {
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

#else
#error "No support for your board pinout, please submit an issue or PR"
#endif

/* Private function prototypes - don't call these directly */

extern void digital_write_asm(uint8_t pin, uint8_t state);
extern void pin_mode_asm(uint8_t pin, uint8_t mode);

/* Public function prototypes - GPIO */

enum pin_mode {
    OUTPUT,
    INPUT,
    INPUT_PULLUP,
};

/**
 * pin_mode - Change a GPIO pin's mode
 *
 * @param pin  board pin number to change
 * @param mode OUTPUT, INPUT, or INPUT_PULLUP
 */
__attribute__((always_inline))
static inline void pin_mode(uint8_t pin, uint8_t mode) {
    /**
     * If both constants are known at compile time, use the
     * C implementation which GCC can optimize away into two
     * instructions, otherwise use the assembly implementation
     * which is faster than whatever GCC would spit out
     */
    if (__builtin_constant_p(pin) && __builtin_constant_p(mode)) {
        const struct pin_mapping *mapping = &pins[pin];
        uint8_t bit = mapping->bit;
        volatile uint8_t *port = (volatile uint8_t *)
                                    ((uint16_t)mapping->port);

        if (mode == OUTPUT) {
            *DDR_FROM_PORT(port) |= bit;
        } else if (mode == INPUT_PULLUP) {
            *DDR_FROM_PORT(port) &= ~bit;
            *port |= bit;
        } else {
            *DDR_FROM_PORT(port) &= ~bit;
        }
    } else {
        pin_mode_asm(pin, mode);
    }
}

/**
 * digital_write - Change a GPIO pin's state
 *
 * @param pin   board pin number to change
 * @param state non-zero for HIGH, zero for LOW
 */
__attribute__((always_inline))
static inline void digital_write(uint8_t pin, uint8_t state) {
    /**
     * If both constants are known at compile time, use the
     * C implementation which GCC can optimize away into a single
     * instruction, otherwise use the assembly implementation
     * which is faster than whatever GCC would spit out
     */
    if (__builtin_constant_p(pin) && __builtin_constant_p(state)) {
        volatile uint8_t *port = (volatile uint8_t *)
                                    ((uint16_t)pins[pin].port);
        const uint8_t bit = pins[pin].bit;
        if (state)
            *port |= bit;
        else
            *port &= ~bit;
    } else {
        digital_write_asm(pin, state);
    }
}

/* Public function prototypes - USART */
struct serial_ringbuf {
    uint8_t buf[256];
    uint8_t pos_start;
    uint8_t pos_end;
};

struct serial_port {
    uint8_t udr;
    uint8_t ucsra;
    uint8_t ucsrb;
    uint8_t ucsrc;
    uint8_t ubrrh;
    uint8_t ubrrl;
    volatile struct serial_ringbuf tx_buf;
    volatile struct serial_ringbuf rx_buf;
    FILE iostream;
};

#if USART_COUNT >= 1
extern struct serial_port *serial0;
#endif
#if USART_COUNT >= 2
#error "Fill in pointer declarations here."
#endif

/**
 * Initialize a UART
 *
 * @param port serial port to initialize
 * @param baud baud rate to sue
 */
void serial_begin(struct serial_port *port, uint32_t baud);

/**
 * Write bytes to a serial port
 *
 * @param port  serial port to write to
 * @param buf   buffer to read from
 * @param count number of bytes to write
 * @return      number of bytes written
 */
uint8_t serial_write(struct serial_port *port, uint8_t *buf, uint8_t count);

/**
 * Write bytes to a serial port (blocking)
 * Same as serial_write but will block until space is free in the buffer
 */
void serial_write_blocking(struct serial_port *port, uint8_t *buf, uint8_t count);

/**
 * Read bytes from a serial port
 *
 * @param port    port to read from
 * @param buf_out buffer to write to
 * @param count   maximum number of bytes to write
 * @return        number of bytes read
 */
uint8_t serial_read(struct serial_port *port, uint8_t *buf_out, uint8_t count);

/**
 * Read bytes from a serial port
 * Same as serial_read but will block until the out buffer can be filled
 */
void serial_read_blocking(struct serial_port *port, uint8_t *buf_out, uint8_t count);

/**
 * Get the number of bytes that can currently be read from the serial port
 *
 * @param port  port to check
 * @return      number of bytes ready to read
 */
uint8_t serial_available(struct serial_port *port);

/**
 * Read bytes from a serial port until a delimiter byte is encountered, or
 * the buffer is full
 *
 * @param port      port to read from
 * @param buf_out   buffer to write to
 * @param count     maximum number of bytes to write
 * @param delimiter stop reading when this byte is encountered
 * @return          number of bytes read
 */
uint8_t serial_read_until(struct serial_port *port, uint8_t *buf_out, uint8_t count,
                          uint8_t delimiter);

#endif /* LIBMINIAVR_H */
