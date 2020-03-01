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

static void serial_flush(struct serial_port *serial) {
    while (serial_available(serial)) {
        uint8_t buf;
        serial_read(serial, &buf, 1);
    }
}

enum serial_command {
    CMD_HELLO = 1,
    CMD_SEND = 2,
    CMD_GET
};

int main(void) {
    // Initalize USART0 at 115200 baud
    serial_begin(serial0, 115200);
    lora_setup(&lora0, RST_PIN, SS_PIN, IRQ_PIN);

#if 1
    uint8_t buf[255];
    for(;;){
        for(int i = 0; i<255;i++) buf[i] = 0;
        lora_listen(&lora0);
        if(serial_available(serial0)!=0){
            uint8_t n = serial_read_until(serial0, buf, sizeof(buf), '\r');
            lora_load_message(&lora0,buf);
            
            uint8_t reg = lora_read_reg(&lora0,LORA_REG_OP_MODE);
            uint8_t reg2 = lora_read_reg(&lora0,0x12);

            // Wait for IRQ
            fprintf(&serial0->iostream,"waiting...mode:%x irq:%x\r\n",reg,reg2);


            fprintf(&serial0->iostream,"tranmsititng\r\n"); 
            lora_transmit(&lora0);
            fprintf(&serial0->iostream,"done transmitting\r\n");
        }
        enum lora_fifo_status msg_stat = lora_get_packet(&lora0,buf);
        fprintf(&serial0->iostream,"packet stat: %x\r\n",msg_stat);
        if(msg_stat == FIFO_GOOD){
            fprintf(&serial0->iostream,"got message:\r\n");
            fputs(buf,&serial0->iostream);
        }else if(msg_stat == FIFO_BAD){
            fprintf(&serial0->iostream, "bad packet\r\n");
        }
        _delay_ms(500);
    }
        
        
        
        
#endif


#if 0
    lora_listen(&lora0);
    for (;;) {
        // Receive command
        while (serial_available(serial0) == 0);

        uint8_t cmd;
        serial_read_blocking(serial0, &cmd, 1);
        switch(cmd) {

        }
    }
#endif

#if 0
    fputs("Press any key to send a message.\r\n", &serial0->iostream);
    lora_listen(&lora0);
    for (;;) {
        // Check if user input is available
        if (serial_available(serial0) > 0) {
            serial_flush(serial0);

            // Prompt user for information
            uint8_t recip[10];
            uint8_t location[10];
            uint8_t message[235];
            uint8_t confirmation[2];

            fputs("Recipient: ", &serial0->iostream);
            uint8_t n = serial_read_until(serial0, recip, sizeof(recip), '\r');
            recip[n - 1] = '\0';
            serial_flush(serial0);
            fputs("\r\n", &serial0->iostream);

            fputs("Location: ", &serial0->iostream);
            n = serial_read_until(serial0, location, sizeof(location), '\r');
            location[n - 1] = '\0';
            serial_flush(serial0);
            fputs("\r\n", &serial0->iostream);

            fputs("Message: ", &serial0->iostream);
            n = serial_read_until(serial0, message, sizeof(message), '\r');
            message[n - 1] = '\0';
            serial_flush(serial0);
            fputs("\r\n", &serial0->iostream);

            // Format message and prompt for confirmation
            fprintf(&serial0->iostream, "Recipient: %s\r\nLocation: %s\r\nMessage: %s\r\n", recip, location, message);
            fputs("Send? (y/n) ", &serial0->iostream);
            n = serial_read_until(serial0, confirmation, sizeof(confirmation), '\r');
            confirmation[n - 1] = '\0';
            serial_flush(serial0);
            fputs("\r\n", &serial0->iostream);

            if (confirmation[0] == 'y') {
                fputs("Sending message.\r\n", &serial0->iostream);
                uint8_t msgbuf[255];
                memcpy(msgbuf, recip, sizeof(recip));
                memcpy(msgbuf, location, sizeof(location));
                memcpy(msgbuf, message, sizeof(message));

                // Send message
                lora_load_message(&lora0, msgbuf);
                lora_transmit(&lora0);

                // Switch back to listen mode
                lora_listen(&lora0);
            } else {
                fputs("Cancelled.\r\n", &serial0->iostream);
            }
        }

        // See if any packets were received
        struct {
            uint8_t recip[10];
            uint8_t location[10];
            uint8_t message[235];
        } buf;

        enum lora_fifo_status ret = lora_get_packet(&lora0, (uint8_t *)&buf);

        if (ret == FIFO_BAD) {
            fputs("Corrupted message received.\r\n", &serial0->iostream);
        } else if (ret == FIFO_GOOD) {
            fputs("Received Message!", &serial0->iostream);

            fprintf(&serial0->iostream, "Recipient: %s\r\nLocation:%s\r\nMessage: %s\r\n", buf.recip, buf.location, buf.message);
        }
    }
#endif

#if 0
    for (;;) {
        // Receive packets using HLAPI
        lora_listen(&lora0);

        enum lora_fifo_status ret;
        uint8_t buf[LORA_PACKET_SIZE + 1];
        while ((ret = lora_get_packet(&lora0, buf)) == FIFO_EMPTY);

        if (ret == FIFO_BAD) {
            fputs("Bad packet received\r\n", &serial0->iostream);
        } else {
            buf[LORA_PACKET_SIZE] = '\0';
            fprintf(&serial0->iostream, "Got data: %s\r\n", buf);
        }

        _delay_ms(1000);
    }
#endif

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

#if 0
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
        // Receive packets using HLAPI
        lora_listen(&lora0);

        enum lora_fifo_status ret;
        uint8_t buf[LORA_PACKET_SIZE + 1];
        while ((ret = lora_get_packet(&lora0, buf)) == FIFO_EMPTY);

        if (ret == FIFO_BAD) {
            fputs("Bad packet received\r\n", &serial0->iostream);
        } else {
            buf[LORA_PACKET_SIZE] = '\0';
            fprintf(&serial0->iostream, "Got data: %s\r\n", buf);
        }

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
