// (c) Copyright 2021 Aaron Kimball
// This file is licensed under the BSD 3-clause open source license.
// See LICENSE.txt for full licensing terms.

#include "LCD-NHD0440.h"
#include "util/nanodelay.h"

static const uint8_t enFlag_L_mask = 0x3F;   // Mask with enable flags held low.

I2C4BitNhdByteSender::I2C4BitNhdByteSender(): _i2cp(I2CParallel()) {
}

I2C4BitNhdByteSender::~I2C4BitNhdByteSender() {
}

void I2C4BitNhdByteSender::init(const uint8_t i2cAddr) {
  // Connect to the I2C interface.
  _i2cp.init(i2cAddr); 

  // The PCF8574 starts with all bits high (tri-stated and pulled up)
  // and the LCD wants all bits suppressed on startup; get them low ASAP.
  _i2cp.setByte(0);
}

void I2C4BitNhdByteSender::sendByte(uint8_t v, uint8_t ctrlFlags, uint8_t enFlags) {
  uint8_t out = 0;

  out |= (enFlags & LCD_ENABLE_FLAGS);
  out |= (ctrlFlags & LCD_CTRL_FLAGS);
  // Send the high nibble first.
  out |= (v >> 4) & 0x0F;
    
  _i2cp.setByte(out);
  _i2cp.waitForValid(); // establish EN as HIGH and wait for setup time on data.

  // Drop the enable flag to low to lock the high nibble in.
  _i2cp.setAnd(enFlag_L_mask);
  _i2cp.waitForValid(); // wait for hold time on data after falling edge of EN.

  out &= (LCD_ENABLE_FLAGS | LCD_CTRL_FLAGS); // Keep flags, ditch data nibble.
  out |= (v & 0xF); // send low nibble.
    
  _i2cp.setByte(out);
  _i2cp.waitForValid();

  // Drop the enable flag to low to lock the second nibble in.
  _i2cp.setAnd(enFlag_L_mask);
}

void I2C4BitNhdByteSender::sendHighNibble(uint8_t v, uint8_t ctrlFlags, uint8_t enFlags) {
  uint8_t out = 0;

  out |= (enFlags & LCD_ENABLE_FLAGS);
  out |= (ctrlFlags & LCD_CTRL_FLAGS);
  // Send the high nibble.
  out |= (v >> 4) & 0x0F;
    
  _i2cp.setByte(out);
  _i2cp.waitForValid();

  // Drop the enable flag to low to lock the high nibble in.
  _i2cp.setAnd(enFlag_L_mask);
}

uint8_t I2C4BitNhdByteSender::readByte(uint8_t ctrlFlags, uint8_t enFlag) {
  uint8_t send = 0;
  send |= (enFlag & LCD_ENABLE_FLAGS); // Warning: You must pass exactly 1 enable flag or they'll fight.
  send |= (ctrlFlags & LCD_CTRL_FLAGS);
  send |= 0xF; // Keep the 4 data lines set 'high' to enable reading.
  _i2cp.setByte(send); // Set up the READ command.
  _i2cp.waitForValid();

  // I2C wait time (4us) greatly exceeds t_DDR (100ns) so lines are now valid with high nibble.
  uint8_t scan = _i2cp.read(); // Read back current contents (high nibble).
  uint8_t v = (scan & 0xF) << 4;

  send &= enFlag_L_mask;
  _i2cp.setByte(send); // Drop EN flag to low; discard first nibble.
  _i2cp.waitForValid();

  send |= (enFlag & LCD_ENABLE_FLAGS); // set EN high again.
  _i2cp.setByte(send); // Set up the second half of the READ command.
  _i2cp.waitForValid(); // t_DDR elapses; low nibble now valid.
  scan = _i2cp.read();  // Read back current contents (low nibble).
  v |= (scan & 0xF);

  send &= enFlag_L_mask;
  _i2cp.setByte(send); // Drop EN flag to low, acknowledge/discard 2nd nibble. 
  // No need to do final delay since NewhavenLcd0440 obj will do a 37us wait.

  return v;
}

void I2C4BitNhdByteSender::setBusMode(uint8_t busMode) {
  if (busMode == NHD_MODE_READ) {
    // Set the low-order 4 bits of the bus (the non-control data lines) to HIGH
    // to enable reading. Control lines remain unaffected.
    _i2cp.setOr(0xF);
  }

  // Nothing to do for NHD_MODE_WRITE; the next write operation just
  // overwrites the 4 data lines.
}
