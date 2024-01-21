
LCD-NHD0440
===========

An Arduino driver library for ST7066U-compatible LCD or OLED character display devices,
with extra dual-driver support to enable the Newhaven Devices `0440` series of 4x40 char LCDs.
Includes a "TTY scrolling" feature that scrolls lines upward as you emit more lines with
`print()` / `println()`.

This can work with the NHD0440 or any other single- or dual-ST7066U display connected to the Arduino
through a PCF8574 or PCF8574A remote 8-bit I/O expander connected on the I2C bus, or through direct
wiring of the 4-bit-mode NHD0440 inputs to Arduino GPIO lines.

Dependencies
------------

This requires the following of my other Arduino libraries:

* [i2cparallel](https://github.com/kimballa/i2cparallel)
* [PyArduinoDebug](https://github.com/kimballa/PyArduinoDebug)

This latter dependency is optional; comment out the `#include<dbg.h>` line if you don't
want to compile with debugger support.

Compiling
---------

I build this with my [Arduino makefile](https://github.com/kimballa/arduino-makefile).

* Clone the makefile project such that `arduino-makefile/` is a sibling of this project directory.
* Create `~/arduino_mk.conf` from the template in that directory and customize it to your board
  and local environment. See other one-time setup instructions in that project's README.md and/or
  the comment header of `arduino.mk`.
* Clone the library projects as siblings as well.
* In each library directory, build with `make install`.
* Build this library with `make install`

Usage
-----

* Include `LCD-NHD0440.h` in your sketch source file.
* Create an instance of the `ST7066UController<n_rows, n_cols, n_screens>` class and
  provide a reference to a `constexpr ST7066UTiming` struct with the timing data needed to
  communicate with the device.
* To use the Newhaven 0440 specifically, include `Newhaven0440Impl.h` and create an
  instance of the `NewhavenLcd0440` class, instead. The timing data is already configured.
* Add `libs := Wire LCD-NHD0440 i2cparallel` to your arduino.mk-driven Makefile.
* This project includes multiple communication bus implementations depending on whether
  the LCD is driven through direct GPIO or through an I2C bus expander. See the comments
  in LCD-NHD0440.h for instructions on how to configure for each mode including how to
  wire the NHD0440's inputs to the bus expander.

License
-------

This project is licensed under the BSD 3-Clause license. See LICENSE.txt for complete details.
