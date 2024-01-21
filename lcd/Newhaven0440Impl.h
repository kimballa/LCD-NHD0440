// (c) Copyright 2024 Aaron Kimball
// This file is licensed under the BSD 3-clause open source license.
// See LICENSE.txt for full licensing terms.
//
// Newhaven 0440-specific implementation.

#ifndef _NEWHAVEN_0440_IMPL_H
#define _NEWHAVEN_0440_IMPL_H

#include "../LCD-NHD0440.h"

// Timing constants below are used only in the Newhaven 0440-specific implementation,
// but are retained as distinct constant #define's below for backward compatibility.

// Cannot operate the NHD0440 until millis() returns > BOOT_TIME_MILLIS
// (tIOL is min 40ms; we need to hold the IO pins *low* during this period.)
#define NHD_0440_BOOT_TIME_MILLIS ((unsigned long)50)

// Delay timing in microseconds for NHD0440 operations after communicating over the bus
// to the ST7066.

// Delay timing for 'clear' and 'home' commands
#define NHD_CLEAR_DELAY_US          1520
#define NHD_HOME_DELAY_US           1520
// Delay timing for all other commands, RAM reads and writes
#define NHD_DEFAULT_DELAY_US          37
// Despite only 37us needed for most commands, the busy flag isn't valid until T+80us
#define NHD_BUSY_FLAG_POLL_DELAY_US   80

// Fastest we can push data thru the bus and cycle the enable flag to high, low, and high again.
// If using the I2C bus this is irrelevant since each state change has a 4us hold time, but
// a direct pinout connection could violate this timing constraint unless guarded against.
#define NHD_BUS_CYCLE_TIME_NS       1200

// Parameters that define the Newhaven 0440 LCD's screen dimensions.
#define NHD_NUM_ROWS  (4)
#define NHD_NUM_COLS  (40)
#define NHD_SCREEN_COUNT (2)

// Main API to control the Newhaven 0440 series LCD
class NewhavenLcd0440 : ST7066UController<NHD_NUM_ROWS, NHD_NUM_COLS, NHD_SCREEN_COUNT> {
public:
  NewhavenLcd0440();
};

#endif /* _NEWHAVEN_0440_IMPL_H */
