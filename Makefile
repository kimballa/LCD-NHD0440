# (c) Copyright 2021 Aaron Kimball

lib_name := LCD-NHD0440
libs := Wire i2cparallel PyArduinoDebug
src_dirs := . ./lcd

include ../arduino-makefile/arduino.mk
