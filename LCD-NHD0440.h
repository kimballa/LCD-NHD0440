/*
  (c) Copyright 2021 Aaron Kimball
  See full licensing terms at the end of this comment block, or in LICENSE.txt.

  Arduino driver for ST7066U-compatible LCD controllers.
  This library can handle one or two ST7066U controllers per screen; the latter case
  is used in the Newhaven Devices 0440-series 4x40 char LCDs, which this library was built to support.
  These LCDs are integrated on top of a pair of ST7066U LCD ICs.
  (A Hitachi HD44780-compatible chip.)

  Datasheet references:
  * Display:   https://www.mouser.com/datasheet/2/291/NHD-0440WH-ATFH-JT-47935.pdf
  * Driver IC: https://www.newhavendisplay.com/resources_dataFiles/datasheets/LCDs/ST7066U.pdf

  This driver uses the 4-bit bus mode (not 8-bit).

  This can either be directly attached to 8 I/O lines of the AVR, or the E1,
  E2, RW, RS, and the 4 data bits can be collectively attached over an I2C
  8-bit bus connected with the I2CParallel lib.  In this mode, the LCD can be
  controlled with only 2 shared I2C pins from the AVR.

  You must instantiate the correct NhdByteSender based on the connection mode:

     #include<LCD-NHD0440.h>  // Include this header file.
     #include<Newhaven0440Impl.h> // Include the Newhaven 0440-specific implementation header.

     // -- Use one of the following two code snippets --
     // Connect via I2C to PCF8574[A] parallel port.
     Wire.begin();
     I2C4BitNhdByteSender byteSender();
     byteSender.init(I2C_PCF8574A_MIN_ADDR); // or your addr of parallel port on I2C bus.

     // --or--
     // Use Arduino D11..D4 gpio pins as direct-wired bus.
     Direct4BitNhdByteSender byteSender(11, 10, 9, 8, 7, 6, 5, 4);
     byteSender.init();

     // -- Then follow either of the above with --
     NewhavenLcd0440 lcd;

     // ... or define an LCD of arbitrary dimensions or I/O timing:
     constexpr ST7066UTiming timing(bootMs, clearUs, homeUs, defaultUs, busyFlagUs, busNanos);
     ST7066UController<n_rows, n_cols, n_screens> lcd(timing);

     // -- Then connect LCD to bus & send LCD init commands and use LCD according to API. e.g.: --
     lcd.init(&byteSender);

     lcd.print("Hello, world!");
     delay(1000);
     lcd.clear(); // clear screen
     lcd.setCursor(2, 0); // put cursor on 3rd row
     lcd.println("More text...");


  BSD 3-Clause license text:
  Copyright 2022 Aaron Kimball

  Redistribution and use in source and binary forms, with or without modification, are
  permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list of
       conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, this list
       of conditions and the following disclaimer in the documentation and/or other materials
       provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors may be
       used to endorse or promote products derived from this software without specific prior
       written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef LCD_NHD_0440_H
#define LCD_NHD_0440_H

#include<Arduino.h>
#include<I2CParallel.h>

class NhdByteSender {
public:
  NhdByteSender() {};
  ~NhdByteSender() {};

  // Must be overridden by implementation class
  virtual void sendByte(uint8_t v, uint8_t ctrlFlags, uint8_t enFlags) = 0;
  /* Send the high nibble of v with RW/RS and latch. */
  virtual void sendHighNibble(uint8_t v, uint8_t ctrlFlags, uint8_t enFlags) = 0;
  /* Read a byte from the NHD memory. */
  virtual uint8_t readByte(uint8_t ctrlFlags, uint8_t enFlag) = 0;
  /** Switch between NHD_MODE_WRITE and NHD_MODE_READ. */
  virtual void setBusMode(uint8_t busMode) = 0;
};

#define NHD_MODE_WRITE (OUTPUT)
#define NHD_MODE_READ  (INPUT)

/**
 * Send data to the NHD 0440 in 4-bit bus mode. The Arduino sends data over I2C
 * to a PCF8574 8-bit bus expander (via I2CParallel lib). The 8-bit bus i/o is wired
 * to the NHD 0440 as follows:
 *
 *   P7 P6 P5 P4  P3  P2  P1  P0 <-- PCF8574
 *    |  |  |  |  |   |   |   |
 *   E1 E2 RW RS DB7 DB6 DB5 DB4 <-- LCD0440
 *
 * For a single controller (1- or 2-line) LCD, P6 is not connected:
 *
 *   P7 P6 P5 P4  P3  P2  P1  P0 <-- PCF8574
 *    |  |  |  |  |   |   |   |
 *   E1  x RW RS DB7 DB6 DB5 DB4 <-- ST7066U controller
 */
class I2C4BitNhdByteSender : public NhdByteSender {
public:
  I2C4BitNhdByteSender();
  ~I2C4BitNhdByteSender();

  // Initialize the I2C interface. Due to the fact that the PCF8574 initially
  // starts with all bits high (actually tri-stated, but pullups draw them high),
  // you should call this method as early as possible in the boot process.
  void init(const uint8_t i2cAddr);

  /* Send the byte value 'v'; initialize RW/RS via ctrlFlags; latch with E1 and/or E2
   * based on enFlags.
   */
  virtual void sendByte(uint8_t v, uint8_t ctrlFlags, uint8_t enFlags);
  /* Send the high nibble of v with RW/RS and latch. */
  virtual void sendHighNibble(uint8_t v, uint8_t ctrlFlags, uint8_t enFlags);
  /* Read a byte from the NHD memory. */
  virtual uint8_t readByte(uint8_t ctrlFlags, uint8_t enFlag);
  /** Switch between NHD_MODE_WRITE and NHD_MODE_READ. */
  virtual void setBusMode(uint8_t busMode);
private:
  I2CParallel _i2cp;
};

/**
 * Send data to the LCD in 4-bit bus mode. The Arduino sends data using
 * 8 GPIO pins wired to the NHD 0440.
 *
 * The LCD0440 has 8 I/O pins as follows: En1 En2 RW RS DB7 DB6 DB5 DB4.
 * You must initialize the ByteSender with Arduino pin numbers that map to each
 * of these. For a single-controller (1- or 2-line) LCD, En2 should be the same
 * pin number as En1.
 *
 * In 4-bit mode, DB3..0 are not used and should be tied to GND. (Testing of my
 * own device suggests they can also be allowed to float.)
 */
class Direct4BitNhdByteSender : public NhdByteSender {
public:
  Direct4BitNhdByteSender(uint8_t EN1, uint8_t EN2, uint8_t RW, uint8_t RS,
      uint8_t DB7, uint8_t DB6, uint8_t DB5, uint8_t DB4);
  ~Direct4BitNhdByteSender();

  void init(); // Set up the GPIO pins as outputs and pull them low.

  /* Send the byte value 'v'; initialize RW/RS via ctrlFlags; latch with E1 and/or E2
   * based on enFlags.
   */
  virtual void sendByte(uint8_t v, uint8_t ctrlFlags, uint8_t enFlags);
  /* Send the high nibble of v with RW/RS and latch. */
  virtual void sendHighNibble(uint8_t v, uint8_t ctrlFlags, uint8_t enFlags);
  /* Read a byte from the NHD memory. */
  virtual uint8_t readByte(uint8_t ctrlFlags, uint8_t enFlag);
  /** Switch between NHD_MODE_WRITE and NHD_MODE_READ. */
  virtual void setBusMode(uint8_t busMode);
private:
  void _setEnable(uint8_t state, uint8_t enFlags);
  const uint8_t _EN1, _EN2, _RW, _RS, _DB7, _DB6, _DB5, _DB4;
};


/**
 * Structure that defines timing requirements for a particular display controller.
 */
typedef struct _ST7066UTiming {
  /** milliseconds to wait after system startup for the display to boot before sending commands. */
  const uint32_t bootTimeMillis;
  /** microseconds to delay after issuing CLEAR command. */
  const uint32_t clearDelayUs;
  /** microseconds to delay after issuing HOME command. */
  const uint32_t homeDelayUs;
  /** microseconds to delay after issuing any other command. */
  const uint32_t defaultDelayUs;
  /** microseconds to delay until the BUSY flag is valid to read. */
  const uint32_t busyFlagPollDelayUs;
  /**
   * Fastest timing we can push data through the bus and cycle the enable flag to high, low, and high again.
   * If using the I2C bus this is likely irrelevant since each state change has a 4us hold time, but
   * a direct pinout connection could violate this timing constraint unless guarded against.
   */
  const uint32_t busCycleTimeNanoseconds;

  constexpr _ST7066UTiming(uint32_t boot, uint32_t clear, uint32_t home, uint32_t _default, uint32_t busyFlag,
      uint32_t bus):
        bootTimeMillis(boot), clearDelayUs(clear), homeDelayUs(home), defaultDelayUs(_default),
        busyFlagPollDelayUs(busyFlag), busCycleTimeNanoseconds(bus) {
  };
} ST7066UTiming;

/**
 * Main implementation class for an LCD or OLED display integration based on the ST7066U or
 * compatible display controller.
 *
 * This is a configurable template class allowing you to define the number of rows and columns,
 * and can also operate in a "multi-screen" mode that stitches two screens/controllers together.
 * N_ROWS and N_COLS specify the total number of rows and columns in the conjoined display (e.g. 2x16
 * or 2x20). If there is only a single display, these refer to rows and columns of characters on
 * that display.
 *
 * A four line display split across two ST7066U controllers can be controlled as a single logical
 * device, where the first controller handles lines 0 and 1, and the second controller handles lines
 * 2 and 3. Both devices must be configured with a matching number of columns.
 *
 * In multi-controller mode, two enable / chip select lines are used to write to one or the other
 * controller, or for operations like "clear screen", both at once. In single-controller mode, only
 * the 'EN1' chip select line is used and EN2 is ignored.
 */
template<unsigned int N_ROWS, unsigned int N_COLS, unsigned int N_SCREENS=1>
class ST7066UController : public Print {
public:
  ST7066UController(const ST7066UTiming &timing);
  ~ST7066UController();

  void init(NhdByteSender *byteSender);  // setup lcd board state.
  void clear(); // clear screen and return home
  void home();  // return cursor to home position
  void setDisplayVisible(bool visible);        // turn display on or off
  void setCursor(bool visible, bool blinking); // turn cursor on or off
  void reset() { init(_byteSender); }; // reset LCD state.

  // Set position on row (e.g., 0--3), col (e.g., 0--39) across the screen(s).
  void setCursorPos(uint8_t row, uint8_t col);

  void setScrollingTTY(bool scroll); // Enable/disable scrolling display.

  virtual size_t write(uint8_t chr); // write 1 character thru the Print interface.

private:
  void _setCursorPos(uint8_t row, uint8_t col, bool updateDisplayFlags=true);
  void _sendCommand(uint8_t cmd, uint8_t enFlags, unsigned int delay_micros);
  void _setCursorDisplay(uint8_t displayNum);
  void _sendDisplayFlags();
  void _waitReady(unsigned int delay_micros);
  void _scrollScreen();

  NhdByteSender* _byteSender;

  uint8_t _pos; // row is [0--3] in ROW_MASK, col is [0--39] in COL_MASK.
  uint8_t _displayFlags; // state of display flags for both subscreens.

  const ST7066UTiming &_timing;
};


#define COL_MASK  ((uint8_t)0x3F)
#define COL_SHIFT ((uint8_t)0)

#define ROW_MASK  ((uint8_t)0xC0)
#define ROW_SHIFT ((uint8_t)6)


#define DISP_FLAG_D1 ((uint8_t)0x4) // display visible (subscreen 1)
#define DISP_FLAG_C1 ((uint8_t)0x2) // cursor vis
#define DISP_FLAG_B1 ((uint8_t)0x1) // blink

#define DISP_FLAG_D2 ((uint8_t)0x20) // display visible (subscreen 2)
#define DISP_FLAG_C2 ((uint8_t)0x10) // cursor vis
#define DISP_FLAG_B2 ((uint8_t)0x08) // blink

#define DISP_FLAG_SCROLL ((uint8_t)0x40) // flag to indicate scrolling behavior after last line

#define DISPLAY_BITS_MASK ((uint8_t)0x7)
#define DISPLAY_1_SHIFT ((uint8_t)0)
#define DISPLAY_2_SHIFT ((uint8_t)3)


#define LCD0440_BUS_WIDTH 4 // 4-bit interface mode.

// Bitmasks for 'flags' fields when transmitted with a nibble of user data
// over a combined 8 bit bus.
#define LCD_E1 0x80
#define LCD_E2 0x40
#define LCD_RW 0x20
#define LCD_RS 0x10

// Caller of NhdByteSender::sendByte() controls RW and RS through `flags`.
// The ByteSender manages E1 and E2 internally via `useE1` argument.
#define LCD_CTRL_FLAGS (LCD_RS | LCD_RW)
#define LCD_ENABLE_FLAGS (LCD_E1 | LCD_E2)

// Bitfield pattern for all enable lines.
// If N_SCREENS is 2, that means LCD_E1 and LCD_E2; otherwise just LCD_E1.
#define LCD_EN_ALL (((N_SCREENS) > 1) ? (LCD_E1 | LCD_E2) : (LCD_E1))

// LCD_RW signal values
#define LCD_RW_READ    (LCD_RW)
#define LCD_RW_WRITE   (0)

// LCD_RS signal values.
#define LCD_RS_COMMAND (0)
#define LCD_RS_DATA    (LCD_RS)

// Bit flags for LCD control operations
#define LCD_OP_CLEAR          0x1
#define LCD_OP_RETURN_HOME    0x2
#define LCD_OP_ENTRY_MODE_SET 0x4
#define LCD_OP_DISPLAY_ON_OFF 0x8
#define LCD_OP_DISPLAY_SHIFT  0x10
#define LCD_OP_FUNC_SET       0x20
#define LCD_OP_SET_CGRAM_ADDR 0x40
#define LCD_OP_SET_DDRAM_ADDR 0x80

// Possible combinations of LCD_OP_ENTRY_MODE_SET and flags defining 4 behaviors
#define LCD_ENTRY_MODE_CURSOR_RIGHT (LCD_OP_ENTRY_MODE_SET | 0x2)
#define LCD_ENTRY_MODE_CURSOR_LEFT  (LCD_OP_ENTRY_MODE_SET)
#define LCD_ENTRY_MODE_SHIFT_RIGHT  (LCD_OP_ENTRY_MODE_SET | 0x1)
#define LCD_ENTRY_MODE_SHIFT_LEFT   (LCD_OP_ENTRY_MODE_SET | 0x1 | 0x2)

// Flags to combine with LCD_OP_DISPLAY_ON_OFF
#define LCD_DISPLAY_ON   0x4
#define LCD_CURSOR_ON    0x2
#define LCD_CURSOR_BLINK 0x1

// Possible combination of LCD_OP_DISPLAY_SHIFT and flags defining 4 actions to take:
#define LCD_DISPLAY_SHIFT_CURSOR_LEFT   (LCD_OP_DISPLAY_SHIFT)
#define LCD_DISPLAY_SHIFT_CURSOR_RIGHT  (LCD_OP_DISPLAY_SHIFT | 0x4)
#define LCD_DISPLAY_SHIFT_DISPLAY_LEFT  (LCD_OP_DISPLAY_SHIFT | 0x8)
#define LCD_DISPLAY_SHIFT_DISPLAY_RIGHT (LCD_OP_DISPLAY_SHIFT | 0x8 | 0x4)

// Define the FUNCTION_SET operation flag along with the `DL` field set for
// the correct bus width
#if LCD0440_BUS_WIDTH == 4
#define LCD_OP_FUNC_SET_WITH_DATA_LEN (LCD_OP_FUNC_SET | FUNC_SET_4_BIT_BUS)
#elif LCD0440_BUS_WIDTH == 8
#define LCD_OP_FUNC_SET_WITH_DATA_LEN (LCD_OP_FUNC_SET | FUNC_SET_8_BIT_BUS)
#else
#error "LCD0440_BUS_WIDTH is not defined. Should be 4 or 8."
#endif

// Flags to combine with LCD_OP_FUNC_SET_WITH_DATA_LEN.
// Note that 2_LINES and FONT_5x11 are incompatible; setting 2_LINES forces 5x8 font.
#define FUNC_SET_8_BIT_BUS 0x10
#define FUNC_SET_4_BIT_BUS 0
#define FUNC_SET_2_LINES   0x8
#define FUNC_SET_1_LINE    0x0
#define FUNC_SET_FONT_5x8  0x0
#define FUNC_SET_FONT_5x11 0x4

// Address masks to use with LCD_OP_SET_CGRAM_ADDR and _SET_DDRAM_ADDR
#define CGRAM_ADDR_MASK 0x3F
#define DDRAM_ADDR_MASK 0x7F

// internal id numbers for the two sub-screens. enable-line E1 latches the top
// subscreen (rows 0 & 1), E2 latches for the bottom subscreen (rows 2 & 3).
#define DISPLAY_TOP    ((uint8_t)0)
#define DISPLAY_BOTTOM ((uint8_t)1)

#endif /* LCD_NHD_0440_H */
