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
#include<dbg.h>

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
template<unsigned int N_ROWS, unsigned int N_COLS, unsigned int N_SCREENS = 1>
class ST7066UController : public Print {
public:
  ST7066UController(const ST7066UTiming &timing):
    _timing(timing), _pos(0), _byteSender(NULL), _displayFlags(0) {

    if (N_SCREENS != 1 && N_SCREENS != 2) {
      DBGPRINTU("N_SCREENS must be 1 or 2; invalid argument: ", N_SCREENS);
    }
  };

  ~ST7066UController() {
    // discard reference to associated object. do not free in here, it was never ours.
    _byteSender = NULL;
  };

  // setup lcd board state.
  void init(NhdByteSender *byteSender) {
    _byteSender = byteSender;

    // Wait for millis() [time since boot] > the configured bootup timing.
    // The device needs to go thru its internal setup logic before it can accept commands.
    unsigned long start_time = millis();
    while (start_time < _timing.bootTimeMillis) {
      unsigned long remaining = _timing.bootTimeMillis - start_time + 1;
      delay(remaining);
      start_time = millis();
    }

    // Configure the receiver for 4-bit bus mode. If we're only connected to 4 bus lines, we can't
    // send the entire LCD_OP_FUNC_SET command since it'd take two I2C commands. Just send the high
    // nibble to configure the bus mode, then reinitialize.
    // According to the ST7066U datasheet (p.25), we actually start with an 8-bit bus config,
    // repeated 3x to clear any prior 4- or 8-bit bus state, then we downgrade to 4-bit mode.
    const uint8_t ctrlFlags = LCD_RW_WRITE | LCD_RS_COMMAND;

    for (uint8_t i = 0; i < 3; i++) {
      // Send 8-bit-at-once command to affirm 8-bit bus. db3..db0 are 'X' / don't care in this config.
      _byteSender->sendHighNibble(LCD_OP_FUNC_SET | FUNC_SET_8_BIT_BUS, ctrlFlags, LCD_EN_ALL);
      _waitReady(_timing.defaultDelayUs);
    }

    // Send command to shift to 4-bit-bus mode. We send this as an 8-bit command,
    // so we only send the high nibble.
    _byteSender->sendHighNibble(LCD_OP_FUNC_SET_WITH_DATA_LEN, ctrlFlags, LCD_EN_ALL);
    _waitReady(_timing.defaultDelayUs);

    // We are now in 4-bit mode. We want to configure other settings, so we transmit
    // both nibbles one after the other: the opcode & 4-bit bus flag, followed by line
    // count & font size.
    _byteSender->sendByte(LCD_OP_FUNC_SET_WITH_DATA_LEN | FUNC_SET_2_LINES | FUNC_SET_FONT_5x8,
        ctrlFlags, LCD_EN_ALL);
    _waitReady(_timing.defaultDelayUs);

    // Reset display flags in case they were previously corrupt.
    _displayFlags = 0;

    // Configure the display into a known, clean state.
    setDisplayVisible(true);
    setCursor(true, true);
    clear();

    // Set ENTRY_MODE to be CURSOR_RIGHT.
    _byteSender->sendByte(LCD_OP_ENTRY_MODE_SET | LCD_ENTRY_MODE_CURSOR_RIGHT, ctrlFlags, LCD_EN_ALL);
    _waitReady(_timing.defaultDelayUs);

    // The display is now ready for text.
  };

  // clear screen and return home
  void clear() {
    _sendCommand(LCD_OP_CLEAR, LCD_EN_ALL, _timing.clearDelayUs);
    setCursorPos(0, 0); // Reset Arduino knowledge of cursor & reset active display to TOP.
  };

  // return cursor to home position
  void home() {
    _sendCommand(LCD_OP_RETURN_HOME, LCD_EN_ALL, _timing.homeDelayUs);
    setCursorPos(0, 0); // Reset Arduino knowledge of cursor & reset active display to TOP.
  };

  // turn display on or off
  void setDisplayVisible(bool visible) {
    if (visible) {
      _displayFlags |= DISP_FLAG_D1 | DISP_FLAG_D2;
    } else {
      _displayFlags &= (~DISP_FLAG_D1) & (~DISP_FLAG_D2);
    }

    _sendDisplayFlags();
  };

  // turn cursor on or off
  void setCursor(bool visible, bool blinking) {
    // Only manipulate cursor for the active lcd subscreen (based on cursor row).
    // The inactive subscreen should always have no cursor shown.
    uint8_t vis_mask, blink_mask;
    if (getRow() < 2 || N_SCREENS < 2) {
      vis_mask = DISP_FLAG_C1;
      blink_mask = DISP_FLAG_B1;
    } else {
      vis_mask = DISP_FLAG_C2;
      blink_mask = DISP_FLAG_B2;
    }

    if (visible) {
      _displayFlags |= vis_mask;
    } else {
      _displayFlags &= ~vis_mask;
    }

    if (blinking) {
      _displayFlags |= blink_mask;
    } else {
      _displayFlags &= ~blink_mask;
    }

    _sendDisplayFlags();
  };

  void reset() { init(_byteSender); }; // reset LCD state.

  // Set position on row (e.g., 0--3), col (e.g., 0--39) across the screen(s).
  /**
   * Set the cursor position in the N_ROWSxN_COLS char screen.
   *
   * row is in e.g. [0, 1] or [0, 3]; col is e.g. [0, 19].
   *
   * In practice this means determining which of the subscreens we're on, moving the
   * cursor appropriately within the subscreen, and setting cursor visibility only on the
   * appropriate subscreen.
   */
  void setCursorPos(uint8_t row, uint8_t col) {
    _setCursorPos(row, col, true);
  };

  // Enable/disable scrolling display.
  void setScrollingTTY(bool scroll) {
    if (scroll) {
      _displayFlags |= DISP_FLAG_SCROLL;
    } else {
      _displayFlags &= ~DISP_FLAG_SCROLL;
    }
  };

  // write 1 character to the screen thru the Print interface.
  virtual size_t write(uint8_t chr) {
    if (chr == '\r') {
      setCursorPos(getRow(), 0);
      return 1;
    } else if (chr == '\n') {
      const uint8_t newRow = getRow() + 1;
      if (newRow >= N_ROWS) {
        if (_displayFlags & DISP_FLAG_SCROLL) {
          _scrollScreen();
        } else {
          setCursorPos(0, 0);  // Wrap back to the top.
        }
      } else {
        setCursorPos(newRow, 0); // Move to the next line.
      }
      return 1;
    }

    if (getCol() >= N_COLS) {
      // Prior write was to the last column on the screen, and we didn't handle
      // a '\n' or '\r' above, meaning we're writing to the "n+1st" column. (nope!)
      // Wrap to the next line before printing.
      const uint8_t nextRow = getRow() + 1;
      if (nextRow >= N_ROWS) {
        if (_displayFlags & DISP_FLAG_SCROLL) {
          _scrollScreen();
        } else {
          setCursorPos(0, 0); // Wrap back to the top.
        }
      } else {
        setCursorPos(nextRow, 0);
      }
    }

    // We're now in a valid location to write a character.
    constexpr uint8_t ctrlFlags = LCD_RW_WRITE | LCD_RS_DATA;
    // choose enable flag based on current row.
    const uint8_t enablePin = (_subscreenForRow(getRow()) == DISPLAY_TOP) ? LCD_E1 : LCD_E2;
    _byteSender->sendByte(chr, ctrlFlags, enablePin);
    _waitReady(_timing.defaultDelayUs);
    _incrementCol();

    return 1;
  };

  // Return row portion of _pos.
  inline uint8_t getRow() const {
    return (_pos & ROW_MASK) >> ROW_SHIFT;
  };

  // Return column portion of _pos.
  inline uint8_t getCol() const {
    return (_pos & COL_MASK) >> COL_SHIFT;
  };

private:
  /**
   * Set the cursor position in the N_ROWSxN_COLS char screen.
   *
   * If `updateDisplayFlags`, determine which of the subscreens we're on, move the
   * cursor appropriately within the subscreen, and set cursor visibility only on the
   * appropriate subscreen. Setting to 'false' will skip display flag updates but may
   * cause the display to go out of sync unless you move it back to the right subscreen.
   */
  void _setCursorPos(uint8_t row, uint8_t col, bool updateDisplayFlags=true) {
    const uint8_t subscreen = _subscreenForRow(row);
    // Choose e1 or e2 based on subscreen for row.
    const uint8_t enablePin = (subscreen == DISPLAY_TOP) ? LCD_E1 : LCD_E2;

    const uint8_t inScreenRow = (row >= 2) ? (row - 2) : row; // Row within subscreen.
    constexpr uint8_t LINE_1_RAM_OFFSET = 0; // First row is addrs 0...(N_COLS-1)
    constexpr uint8_t LINE_2_RAM_OFFSET = 0x40; // Second row is addrs 0x40...(0x40 + N_COLS - 1)
    const uint8_t addr = col + ((inScreenRow == 0) ? LINE_1_RAM_OFFSET : LINE_2_RAM_OFFSET);
    _sendCommand(LCD_OP_SET_DDRAM_ADDR | addr, enablePin, _timing.defaultDelayUs);

    if (updateDisplayFlags) {
      _setCursorDisplay(subscreen); // Make sure cursor is on the right subscreen.
    }
    _pos = _makePos(row, col); // Save this as our new position.
  };

  // Ensures the cursor is visible on the specified display subscreen.
  void _setCursorDisplay(uint8_t displayNum) {
    const uint8_t curDisplay = _subscreenForRow(getRow());
    if (curDisplay == displayNum) {
      return; // Nothing to do.
    }

    // Swap the active display.
    uint8_t d1 = (_displayFlags >> DISPLAY_1_SHIFT) & DISPLAY_BITS_MASK;
    uint8_t d2 = (_displayFlags >> DISPLAY_2_SHIFT) & DISPLAY_BITS_MASK;
    _displayFlags = (d1 << DISPLAY_2_SHIFT) | (d2 << DISPLAY_1_SHIFT) | (_displayFlags & DISP_FLAG_SCROLL);

    _sendDisplayFlags();
  };

  void _sendCommand(uint8_t cmd, uint8_t enFlags, unsigned int delay_micros) {
    constexpr uint8_t ctrlFlags = LCD_RW_WRITE | LCD_RS_COMMAND;
    _byteSender->sendByte(cmd, ctrlFlags, enFlags);
    if (delay_micros > 0) {
      _waitReady(delay_micros);
    }
  };

  // Send display and cursor vis flags to device.
  void _sendDisplayFlags() {
    uint8_t display1 = LCD_OP_DISPLAY_ON_OFF | ((_displayFlags >> DISPLAY_1_SHIFT) & DISPLAY_BITS_MASK);
    uint8_t display2 = LCD_OP_DISPLAY_ON_OFF | ((_displayFlags >> DISPLAY_2_SHIFT) & DISPLAY_BITS_MASK);
    // In a multi-display config, first command needs no delay; the 2nd delay handles both subscreens.
    const uint32_t display1Delay = N_SCREENS < 2 ? _timing.defaultDelayUs : 0;
    _sendCommand(display1, LCD_E1, display1Delay);
    if (N_SCREENS > 1) {
      _sendCommand(display2, LCD_E2, _timing.defaultDelayUs);
    }
  };

  /**
   * Wait until the display is ready / finished processing the command.
   */
  void _waitReady(unsigned int delay_micros) {
    unsigned long start_time = micros();
    unsigned long elapsed = 0;

    while (elapsed < delay_micros) {
      // TODO(aaron): Actually read the busy-flag field and wait for it to drop to zero
      delayMicroseconds(delay_micros - elapsed);
      unsigned long now = micros();
      if (now < start_time) {
        start_time = now;
      }
      elapsed = now - start_time;
    }
  };

  /**
   * Scroll all the lines up by 1.
   */
  void _scrollScreen() {
    constexpr uint8_t ctrlFlagsR = LCD_RW_READ | LCD_RS_DATA;
    constexpr uint8_t ctrlFlagsW = LCD_RW_WRITE | LCD_RS_DATA;

    for (uint8_t r = 1; r < N_ROWS; r++) {
      const uint8_t enFlagR = (_subscreenForRow(r) == DISPLAY_TOP) ? LCD_E1 : LCD_E2;
      const uint8_t enFlagW = (_subscreenForRow(r - 1) == DISPLAY_TOP) ? LCD_E1 : LCD_E2;

      // Buffer one row of char RAM locally.
      // NOTE(aaron): This read-all-then-write-all pattern makes use of the LCD's internal
      // cursor to minimize the number of setPosition() calls. If a 40 byte buffer is too
      // big for the stack, we could do this in blocks of 8 chars to compromise between
      // stack usage and I/O latency.
      uint8_t buffer[N_COLS];

      _setCursorPos(r, 0, false);
      _byteSender->setBusMode(NHD_MODE_READ);
      uint8_t lastRealPos = 0;
      for (uint8_t c = 0; c < N_COLS; c++) {
        // Read operation also moves the cursor 1 to the right.
        uint8_t v = _byteSender->readByte(ctrlFlagsR, enFlagR);
        _waitReady(_timing.defaultDelayUs);
        buffer[c] = v;
        if (v != ' ') {
          lastRealPos = c; // We have chars to copy out at least thru this position.
        }
      }

      _byteSender->setBusMode(NHD_MODE_WRITE);
      if (r == 1) {
        // We just read the 2nd row of the screen, to copy it to the
        // first row. Clear the upper subscreen (and reset to (0, 0)).
        _sendCommand(LCD_OP_CLEAR, LCD_E1, _timing.clearDelayUs);
      } else if (r == N_ROWS - 1) {
        // We just read the last row of the screen, to copy it to the
        // second-to-last row. Clear the bottom display first, to wipe
        // the last line out 0.5ms faster than we could set it byte-by-byte.
        uint8_t enableFlagForClear = (N_SCREENS > 1) ? LCD_E2 : LCD_E1;
        _sendCommand(LCD_OP_CLEAR, enableFlagForClear, _timing.clearDelayUs);
      } else {
        // Reset cursor position to prior row.
        _setCursorPos(r - 1, 0, false);
      }

      // Emit the buffer onto the previous line.
      for (uint8_t c = 0; c <= lastRealPos; c++) {
        if (buffer[c] == 0) {
          break; // No need to copy further in this line.
        }
        _byteSender->sendByte(buffer[c], ctrlFlagsW, enFlagW);
        _waitReady(_timing.defaultDelayUs);
      }
    }

    _setCursorPos(N_ROWS - 1, 0, false); // Return cursor to beginning of bottom line.
  };

  // Create a _pos field value from a row and column.
  inline uint8_t _makePos(uint8_t row, uint8_t col) const {
    return ((row << ROW_SHIFT) & ROW_MASK) | ((col << COL_SHIFT) & COL_MASK);
  };

  // Increment the 'col' field of pos by 1 and return the new 'col' value.
  // Note that this can set _pos to illegal values where col >= N_COLS.
  // The caller is responsible for detecting overflow.
  inline uint8_t _incrementCol() {
    // col is in low-order bits, so just increment.
    ++_pos;
    return getCol();
  };

  // Which of the two subscreens is a given row on?
  inline uint8_t _subscreenForRow(uint8_t row) const {
    return (row < 2 || N_SCREENS < 2) ? DISPLAY_TOP : DISPLAY_BOTTOM;
  };

  //// Actual private state tracking ////

  NhdByteSender* _byteSender;

  const ST7066UTiming &_timing;

  uint8_t _pos; // row is [0--3] in ROW_MASK, col is [0--39] in COL_MASK.
  uint8_t _displayFlags; // state of display flags for both subscreens.
};

#endif /* LCD_NHD_0440_H */
