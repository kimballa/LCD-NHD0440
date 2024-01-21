// (c) Copyright 2024 Aaron Kimball
// This file is licensed under the BSD 3-clause open source license.
// See LICENSE.txt for full licensing terms.
//
// Newhaven 0440-specific implementation.

#include "../LCD-NHD0440.h"
#include "Newhaven0440Impl.h"

// Timing structure with data specific to the Newhaven 0440.
constexpr ST7066UTiming nhd0440Timing(
  NHD_0440_BOOT_TIME_MILLIS,
  NHD_CLEAR_DELAY_US,
  NHD_HOME_DELAY_US,
  NHD_DEFAULT_DELAY_US,
  NHD_BUSY_FLAG_POLL_DELAY_US,
  NHD_BUS_CYCLE_TIME_NS);


// The Newhaven LCD's implementation specifies the timing from the '0440 datasheet, above.
NewhavenLcd0440::NewhavenLcd0440():
    ST7066UController<NHD_NUM_ROWS, NHD_NUM_COLS, NHD_SCREEN_COUNT>(nhd0440Timing) {};
