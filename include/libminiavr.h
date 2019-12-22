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

/**
 * Get the offset of the corresponding DDR/PIN register
 * from a port
 */
#define DDR_FROM_PORT(port_addr) ((volatile uint8_t *)((uint16_t)(port_addr) - 0x1))
#define PIN_FROM_PORT(port_addr) ((volatile uint8_t *)((uint16_t)(port_addr) - 0x2))

/* MCU-specific definitions */
#if defined(__AVR_ATmega328P__)

#define USART_COUNT 1
#define USE_ASM_PIN_ROUTINES 1

#elif defined(__AVR_ATmega2560__)

#define USART_COUNT 4
/* Disable ASM routines on 2560 for now */
#define USE_ASM_PIN_ROUTINES 0

/* ATmega2560 needs a u16 to store all possible I/O registers */
#define NEED_16BIT_IO_PTR 1

#else
#error "No support for your MCU, please submit an issue or PR."
#endif

struct pin_mapping {
#ifndef NEED_16BIT_IO_PTR
    uint8_t port; /* Data Space Address of PORT */
    uint8_t bit; /* Bitmask that controls pin */
#else
    uint16_t port;
    uint8_t bit;
    uint8_t _pad;
#endif
};

/* Board-specific definitions */
#define LIBMINIAVR_PRIV
#include <boards/arduino/uno.h>
#include <boards/arduino/mega.h>
#undef LIBMINIAVR_PRIV


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
    if (!USE_ASM_PIN_ROUTINES || (__builtin_constant_p(pin) && __builtin_constant_p(mode))) {
        const struct pin_mapping *mapping = &libminiavr_board_pins[pin];
        uint8_t bit = mapping->bit;
        volatile uint8_t *port = (volatile uint8_t *)((uint16_t)mapping->port);

        if (mode == OUTPUT) {
            *DDR_FROM_PORT(port) |= bit;
        } else if (mode == INPUT_PULLUP) {
            *DDR_FROM_PORT(port) &= ~bit;
            *port |= bit;
        } else {
            *DDR_FROM_PORT(port) &= ~bit;
            *port &= ~bit;
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
    if (!USE_ASM_PIN_ROUTINES || (__builtin_constant_p(pin) && __builtin_constant_p(state))) {
        volatile uint8_t *port = (volatile uint8_t *)((uint16_t)libminiavr_board_pins[pin].port);
        const uint8_t bit = libminiavr_board_pins[pin].bit;
        if (state)
            *port |= bit;
        else
            *port &= ~bit;
    } else {
        digital_write_asm(pin, state);
    }
}

/**
 * digital_read - Read a GPIO pin's state
 *
 * @param pin   board pin number to read
 * @return      non-zero for HIGH, zero for LOW
 */
__attribute__((always_inline))
static inline uint8_t digital_read(uint8_t pin) {
    volatile uint8_t *port = (volatile uint8_t *)((uint16_t)libminiavr_board_pins[pin].port);
    const uint8_t bit = libminiavr_board_pins[pin].bit;
    return *PIN_FROM_PORT(port) & bit;
}

/* Public function prototypes - USART */
struct serial_ringbuf {
    uint8_t buf[256];
    uint8_t pos_start;
    uint8_t pos_end;
};

struct serial_port {
#ifndef NEED_16BIT_IO_PTR
    uint8_t udr;
    uint8_t ucsra;
    uint8_t ucsrb;
    uint8_t ucsrc;
    uint8_t ubrrh;
    uint8_t ubrrl;
#else
    uint16_t udr;
    uint16_t ucsra;
    uint16_t ucsrb;
    uint16_t ucsrc;
    uint16_t ubrrh;
    uint16_t ubrrl;
#endif
    volatile struct serial_ringbuf tx_buf;
    volatile struct serial_ringbuf rx_buf;
    FILE iostream;
};

#if USART_COUNT >= 1
extern struct serial_port *serial0;
#endif
#if USART_COUNT >= 2
extern struct serial_port *serial1;
#endif
#if USART_COUNT >= 3
extern struct serial_port *serial2;
#endif
#if USART_COUNT >= 4
extern struct serial_port *serial3;
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
