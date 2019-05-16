libminiavr
==========
libminiavr is a minimalist Hardware Abstraction Layer (HAL) targeting AVR 8-bit microcontrollers.
It is meant to implement a similar API to the Arduino project, but written in C and with a focus on optimization.

libminiavr can be used standalone with nothing more than an avr-gcc toolchain and avr-libc, though
adapting it for use with the Arduino IDE should be trivial.

For best results, you should use a modern version of GCC to enable advanced optimizations.
Many routines in the library are implemented in both C and AVR assembly and the one with the highest
optimization potential is selected by the compiler. For example, when all parameters
to a function are known at compile time, C versions are used to allow GCC to perform many calculations
at compile-time and inline the result. In other cases, a hand-written assembly routine is used.

Build
-----
To build libminiavr, simply clone the repository and run `make`:
```
$ make MCU=atmega328p BOARD=ARDUINO_AVR_UNO
```

To build for other targets (currently not supported), simply change `MCU` and `BOARD`.
The resulting `libminiavr.a` file is a static library that can be linked against in
your projects. For an example of this, see `examples/blinky`.


License
-------
libminiavr is licensed under the GNU LGPL v3. For full license text, see `LICENSE.md`
