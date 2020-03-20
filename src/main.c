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
#include "lora_config.h"
#include "sx127x.h"

#define RST_PIN 9
#define SS_PIN 10
#define IRQ_PIN 2

static struct lora_modem lora0;

static struct modulation_config short_range_config = {
	SF6,
	chiprate_500000,
	CR4_5,
	false, //disable header
	true, //enable crc
	8,
	255
};

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
volatile bool flag = false;
void done_rx(struct lora_modem *lora){
	//uint8_t data = lora->irq_data;
	//lora->irq_seen = true;
	//lora_dbg_print_irq(data);
	//fprintf(&serial0->iostream,"reception complete");
	flag = true;
}

void done_tx(struct lora_modem *lora){
	//uint8_t data = lora->irq_data;
	lora->irq_seen = true;
	//lora_dbg_print_irq(data);
	//fprintf(&serial0->iostream,"transmission complete");
	flag = true;
}

int main(void) {
    // Initalize USART0 at 115200 baud
    serial_begin(serial0, 115200);//115200);
    lora_setup(&lora0, RST_PIN, SS_PIN, IRQ_PIN, &short_range_config, &done_rx, &done_tx);
	
#if 1
    uint8_t buf[255];
    lora_listen(&lora0);
    for(;;){
        for(int i = 0; i<255;i++) buf[i] = 0;
        if(serial_available(serial0)!=0){
            serial_read_until(serial0, buf, sizeof(buf), '\r');
            
            uint8_t plen = 0;
            bool move = true;
            while(plen<255 && move){
				if(buf[plen]=='\n') move = false;
				plen++;	
			}
			uint8_t lora_mode;
			lora_mode = lora_read_reg(&lora0,LORA_REG_OP_MODE);
			fprintf(&serial0->iostream,"lora mode before fifo write: %x\r\n",lora_mode);
			
			fprintf(&serial0->iostream,"in buffer: %s\r\n",buf);
            lora_set_payload(&lora0,buf,plen);
            uint8_t test_buf[255];
            lora_read_fifo(&lora0,test_buf,255,0);
            
            fprintf(&serial0->iostream,"in fifo: %s\r\n",test_buf);


            // Wait for IRQ


            fprintf(&serial0->iostream,"transmiting\r\n");
            flag = false;
            lora_transmit(&lora0);
            while(!flag){
				/*
				uint8_t state = lora_read_reg(&lora0,REG_DIO_MAPPING_1);
				int8_t object_state = (&lora0)->irq_mode;
				uint8_t mode = lora_read_reg(&lora0,LORA_REG_OP_MODE);
				fprintf(&serial0->iostream,"register: %x\r\n",state);
				fprintf(&serial0->iostream,"object info: %x\r\n",object_state);
				fprintf(&serial0->iostream,"waiting\r\n");
				fprintf(&serial0->iostream,"mode:%x\r\n",mode);*/
            }
            fprintf(&serial0->iostream,"sent: ");
            fwrite(buf,plen,1,&serial0->iostream);
            lora_listen(&lora0);
        }
		uint8_t expected_length = 0;
        enum lora_fifo_status msg_stat = lora_get_payload(&lora0,buf,&expected_length);
        //fprintf(&serial0->iostream,"packet stat: %x\r\n",msg_stat);

        if (msg_stat == FIFO_GOOD) {
            fprintf(&serial0->iostream,"got message:");
			fwrite(buf,expected_length,1,&serial0->iostream);
        } else if(msg_stat == FIFO_BAD) {
            fprintf(&serial0->iostream, "bad packet\r\n");
        }
        
		//fprintf(&serial0->iostream,"hi\r\n");
        _delay_ms(50);
    }
#endif

}
