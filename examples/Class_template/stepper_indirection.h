#pragma once

#include <TMC2208Stepper.h>
extern TMC2208Stepper<SoftwareSerial> driver_sw;
extern TMC2208Stepper<HardwareSerial> driver_hw;