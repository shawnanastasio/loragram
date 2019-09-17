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
 * This file contains helper routines for utilizing on-chip
 * USARTs.
 *
 * For now, Asynchronous 8n1 mode is only supported for a simplified API.
 */

#include <stdio.h>
#include <stdbool.h>

#include <avr/interrupt.h>

#include "libminiavr.h"

#define REG(x) (*((uint8_t *)(uint16_t)(x)))
#define DIV_ROUND_CLOSEST(dividend, divisor) (((dividend) + ((divisor) / 2)) / (divisor))
#define ARRAY_SIZE(x) (sizeof((x)) / sizeof(*(x)))
#define MIN(x, y) ((x) > (y) ? (y) : (x))

// Flags in UCSRA
#define UCSRA_RXC  (1 << 7)
#define UCSRA_TXC  (1 << 6)
#define UCSRA_UDRE (1 << 5)
#define UCSRA_FE   (1 << 4)
#define UCSRA_DOR  (1 << 3)
#define UCSRA_UPE  (1 << 2)
#define UCSRA_U2X  (1 << 1)
#define UCSRA_MPCM (1 << 0)

// Flags in UCSRB
#define UCSRB_RXCIE (1 << 7)
#define UCSRB_TXCIE (1 << 6)
#define UCSRB_UDRIE (1 << 5)
#define UCSRB_RXEN  (1 << 4)
#define UCSRB_TXEN  (1 << 3)
#define UCSRB_UCSZ2 (1 << 2)
#define UCSRB_RXB8  (1 << 1)
#define UCSRB_TXB8  (1 << 0)

// Flags in UCSRC
#define UCSRC_UMSEL1 (1 << 7)
#define UCSRC_UMSEL0 (1 << 6)
#define UCSRC_UPM1   (1 << 5)
#define UCSRC_UPM0   (1 << 4)
#define UCSRC_USBS   (1 << 3)
#define UCSRC_UCSZ1  (1 << 2)
#define UCSRC_UCSZ0  (1 << 1)
#define UCSRC_UCPOL  (1 << 1)

// UCSRC.UMSEL modes
#define UMSEL_ASYNC 0

// UCSRC.UPM modes
#define UPM_DISABLED 0

// UCSRC.USBS modes
#define USBS_1BIT 0

// UCSRC.UCSZ modes
#define UCSRC_UCSZ_8BIT (UCSRC_UCSZ1 | UCSRC_UCSZ0)

__attribute__((always_inline))
static inline uint8_t ringbuf_free_space(const volatile struct serial_ringbuf *rb) {
    if (rb->pos_start == rb->pos_end)
        return sizeof(rb->buf) - 1;
    else if (rb->pos_end == rb->pos_start - 1)
        return 0;
    else if (rb->pos_start < rb->pos_end)
        return (sizeof(rb->buf) - 1) - (rb->pos_end - rb->pos_start);
    else // rb->pos_start > rb->pos_end
        return rb->pos_start - rb->pos_end;
}

__attribute__((always_inline))
static inline uint8_t ringbuf_available(const volatile struct serial_ringbuf *rb) {
    return (sizeof(rb->buf) - 1) - ringbuf_free_space(rb);
}

static struct serial_port serial_ports[USART_COUNT + 1] = {
#if USART_COUNT >= 1
    {
        .udr = (uint16_t)&UDR0,
        .ucsra = (uint16_t)&UCSR0A,
        .ucsrb = (uint16_t)&UCSR0B,
        .ucsrc = (uint16_t)&UCSR0C,
        .ubrrh = (uint16_t)&UBRR0H,
        .ubrrl = (uint16_t)&UBRR0L
    },
#endif
#if USART_COUNT >= 2
#error "Fill in registers here."
#endif
    {}, // NULL delimiter
};

#if USART_COUNT >= 1
struct serial_port *serial0 = &serial_ports[0];
#endif
#if USART_COUNT >= 2
#error "Fill in pointers here."
#endif

__attribute__((always_inline))
static inline uint16_t baud_to_ubrr(uint32_t baud) {
    return DIV_ROUND_CLOSEST(F_CPU, baud << 3) - 1;
}

/* ISRs */

// Called whenever a byte is ready to be flushed from the RX buf
ISR(USART_RX_vect) {
#if USART_COUNT > 1
    // Find UART that triggered the interrupt (RXC set)
    struct serial_port *port;
    for (port=&serial_ports[0]; port; port++) {
        if (REG(port->ucsra) & UCSRA_RXC)
            break;
    }
    if (!port)
        return; // None of the USARTs are ready??
#else
    struct serial_port *port = serial0;
#endif

    // Flush byte
    uint8_t byte = REG(port->udr);

    // Insert into ringbuf if there's space
    if (ringbuf_free_space(&port->rx_buf) > 0) {
        port->rx_buf.buf[port->rx_buf.pos_end++] = byte;
    }
}

// Called whenever the TX buffer is ready to be written to
ISR(USART_UDRE_vect) {
#if USART_COUNT > 1
    // Find UART that triggered the interrupt (UDRE set)
    struct serial_port *port;
    for (port=&serial_ports[0]; port; port++) {
        if (REG(port->ucsra) & UCSRA_UDRE)
            break;
    }
    if (!port)
        return; // None of the USARTs are ready??
#else
    struct serial_port *port = serial0;
#endif

    if (ringbuf_available(&port->tx_buf) == 0) {
        // No bytes in the tx ringbuffer, disable UDR interrupts.
        REG(port->ucsrb) &= ~UCSRB_UDRIE;
        return;
    }

    uint8_t byte = port->tx_buf.buf[port->tx_buf.pos_start++];
    REG(port->udr) = byte;
}

/* Low-level Serial I/O API */

__attribute__((always_inline))
static inline void serial_write_impl(struct serial_port *port, uint8_t *buf, uint8_t count) {
    // Flush buffer
    while (count-- > 0)
        port->tx_buf.buf[port->tx_buf.pos_end++] = *(buf++);

    // Enable UDR interrupt
    REG(port->ucsrb) |= UCSRB_UDRIE;
}

void serial_write_blocking(struct serial_port *port, uint8_t *buf, uint8_t count) {
    // Block until there's enough space
    while (ringbuf_free_space(&port->tx_buf) < count) ;

    serial_write_impl(port, buf, count);
}

uint8_t serial_write(struct serial_port *port, uint8_t *buf, uint8_t count) {
    uint8_t free = ringbuf_free_space(&port->tx_buf);
    if (free == 0)
        return 0;

    count = MIN(count, free);
    serial_write_impl(port, buf, count);
    return count;
}

__attribute__((always_inline))
static inline uint8_t serial_read_impl(struct serial_port *port, uint8_t *buf_out, uint8_t count) {
    uint8_t c = count;
    while (c-- > 0)
        *(buf_out++) = port->rx_buf.buf[port->rx_buf.pos_start++];

    return count;
}

void serial_read_blocking(struct serial_port *port, uint8_t *buf_out, uint8_t count) {
    while (ringbuf_available(&port->rx_buf) < count) ;

    serial_read_impl(port, buf_out, count);
}

uint8_t serial_read(struct serial_port *port, uint8_t *buf_out, uint8_t count) {
    uint8_t avail = ringbuf_available(&port->rx_buf);
    if (avail == 0)
        return 0;

    count = MIN(count, avail);
    return serial_read_impl(port, buf_out, count);
}

uint8_t serial_read_until(struct serial_port *port, uint8_t *buf_out, uint8_t count, uint8_t delimiter) {
    uint8_t buf_i = 0;
    for (;;) {
        // Wait for byte to become available
        uint8_t avail;
        while ((avail = ringbuf_available(&port->rx_buf)) == 0) ;

        // Flush bytes
        while (avail-- > 0) {
            if (buf_i == count)
                goto out;

            uint8_t cur = port->rx_buf.buf[port->rx_buf.pos_start++];
            buf_out[buf_i++] = cur;

            if (cur == delimiter)
                goto out;
        }
    }

out:
    return buf_i;
}

inline uint8_t serial_available(struct serial_port *port) {
    return ringbuf_available(&port->rx_buf);
}

/* FILE iostream wrapper functions */

static int file_put(char c, FILE *f) {
    struct serial_port *port = fdev_get_udata(f);
    if (!port)
        return 1;

    // Block for free space
    while (ringbuf_free_space(&port->tx_buf) == 0) ;

    port->tx_buf.buf[port->tx_buf.pos_end++] = c;

    // Enable UDR interrupt
    REG(port->ucsrb) |= UCSRB_UDRIE;
}

static int file_get(FILE *f) {
    struct serial_port *port = fdev_get_udata(f);
    if (!port)
        return _FDEV_ERR;

    // Block for bytes to read
    while (ringbuf_available(&port->rx_buf) == 0) ;

    return port->rx_buf.buf[port->rx_buf.pos_start++];
}

/* Serial init */

void serial_begin(struct serial_port *port, uint32_t baud) {
    // Disable interrupts during init
    cli();

    // Set baud in UBRR
    uint16_t ubrr = baud_to_ubrr(baud);
    REG(port->ubrrh) = ubrr >> 8;
    REG(port->ubrrl) = ubrr & 0xFF;

    // Set U2X (double divisor)
    REG(port->ucsra) = UCSRA_U2X;

    // Set RXEN, TXEN, RXCIE
    REG(port->ucsrb) = UCSRB_RXEN | UCSRB_TXEN | UCSRB_RXCIE;

    // Set Async 8n1 in UCSRC
    REG(port->ucsrc) = UCSRC_UCSZ_8BIT;

    // Enable interrupts
    sei();

    // Setup FILE iostream
    fdev_setup_stream(&port->iostream, file_put, file_get, _FDEV_SETUP_RW);
    fdev_set_udata(&port->iostream, port);
}
