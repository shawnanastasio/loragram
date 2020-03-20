# Loragram: A low power, cost-effective digital radio communicator
---
## Purpose
Many locations lack robust communications infrastructure. Even in areas with widespread communications infrastructure blackouts can still occur. Services like the American Radio Relay League (ARRL) prepare for exactly these kinds of scenarios. Members of the ARRL receive emergency communications training to relay messages using analog voice modulation. However, the necessary hardware and training are not accessible to the average person. Loragram aims to resolve this by leveraging cheap off-the-shelf open source hardware. Loragram paired with a suite of open source software can provide a simple way to communicate with others at a distance when other infrastructure is not available. 

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

`libminiavr` provides serial and gpio libraries. **spi_master.h** provides access to the spi interface of the atmega328p. **lora.h** uses **spi_master.h** to provide a way to read/write registers on the chip and exposes high level API functions. **main.c** runs an event loop to permit the user to transmit and recieve messages.

Usage
------
Loragram can be built on any system with GNU make and the avr-gcc toolchain.

To get started, clone the repository:
```
$ git clone --recursive https://github.com/shawnanastasio/loragram
```

Next, build the included `libminiavr` library and the loragram project:
```
$ cd loragram/libminiavr
$ make
$ cd ..
$ make
```

With the project built, you can install it to a connected Arduino Nano using the `upload` target:
```
$ make upload PORT=/dev/ttyUSBx
```
Replace `/dev/ttyUSBx` with the serial port that your Arduino is connected to. (e.g. `COMx` on Windows or `/dev/cu.usbserialX` on macOS).

Once the program is uploaded, you can interact with it using any standard serial terminal program configured for 115200 baud. To use it with GNU screen, just run:
```
$ screen /dev/ttyUSBx 115200
```

## Notes
* The reset pin on the adafruit breakout board must be held LOW in order for the device to function, contrary to the instructions on the adafruit website.
* The reset pin must be pulled low momentarily in order to reset the device when the mcu reboots. This guarantees registers are reset to the original state. Failing to do this can cause weird behavior.
* In order to clear the IRQ flags register, zeros must be written twice over SPI. This is a hardware bug.
* The atmega328p SPI interface will not function if the data direction (pin mode) of SCK, MISO, and MOSI are not set correctly.
* If the SS pin on the atmega328p (PB2 or arduino pin 10) is an input in SPI master mode and is driven low, the SPI interface will switch to the slave mode. This is documented in the atmega328p datasheet on page 139.
* The SX127x module defaults to FSK/OOK mode, not LoRa mode. In order to enter LoRa mode the LongRangeMode bit in RegOpMode must be set.
* Additionally, the bottom three mode bits, labelled "Mode" in the datasheet must reflect a "SLEEP" state when writing to RegOpMode when enabling LoRa mode. If the LoRa's first mode request is not sleep, the write to the register will not be successful. Not even STANDBY can be the first mode. Always SLEEP.
* LoRa FIFO can only be accessed in STANDBY mode
* 
