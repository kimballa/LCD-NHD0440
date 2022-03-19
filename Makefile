# (c) Copyright 2021 Aaron Kimball

lib_name := LCD-NHD0440
libs := wire i2cparallel PyArduinoDebug
src_dirs := .

include ../arduino-makefile/arduino.mk
