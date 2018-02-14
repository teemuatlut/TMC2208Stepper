#include "stepper_indirection.h"

SoftwareSerial mySerial(10,11);
TMC2208Stepper<SoftwareSerial> driver_sw(&mySerial);
TMC2208Stepper<HardwareSerial> driver_hw(&Serial);
