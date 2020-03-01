# Loragram: A low power, cost-effective digital radio communicator
---
## Purpose
Many locations lack robust communications infrastructure. Even in areas with widepsread communcations infrastructure blackouts can still occur. Services like the American Radio Relay League (ARRL) prepare for exactly these kinds of scenarios. Members of the ARRL recieve emergency communications training to relay messages using analog voice modulation. However, the necessary hardware and training are not accessible to the average person. LoRa-gram aims to resolve this by leveraging cheap off-the-shelf open source hardware. LoRa-gram paired with a suite of open source software can provide a simple way to communicate with others at a distance when other infrastructure is not available. 

## Bill of Materials
* Arduino Nano (or any other board with an atmega328p)
* Jumper wires
* RFM95 breakoutboard from adafruit
* 915MHz ISM band antenna
* USB cable

## Technical Information

| Function | Arduino Nano Pin |
|---|---|
| Chip Select | 10 |
| Reset | 9 |
| IRQ | 2 |
| MOSI | 11 |
| MISO | 12 |
| SCK | 13 |

**libminiavr** provides serial and gpio libraries. **spi_master.h** provides access to the spi interface of the atmega328p. **lora.h** uses **spi_master.h** to provide a way to read/write registers on the chip and exposes high level API functions. **main.c** runs an event loop to permit the user to transmit and recieve messages.