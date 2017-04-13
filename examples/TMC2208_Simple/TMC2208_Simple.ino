// Author Teemu Mäntykallio, 2017-04-07

// Define pins
#define EN_PIN    13 															// LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN  54 															// Step on rising edge

#include <TMC2208Stepper.h>												// Include library
TMC2208Stepper driver = TMC2208Stepper(&Serial);	// Create driver and use
																									// HardwareSerial0 for communication

void setup() {
	Serial.begin(115200);														// Init used serial port
	while(!Serial);																	// Wait for port to be ready

	// Prepare pins
	pinMode(EN_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);

	driver.pdn_disable(1);													// Use PDN/UART pin for communication
	driver.I_scale_analog(0);												// Adjust current from the registers
	driver.rms_current(500);												// Set driver current 500mA
	driver.toff(0x2);																// Enable driver

	digitalWrite(13, LOW);													// Enable driver
}

void loop() {
	digitalWrite(STEP_PIN, !digitalRead(STEP_PIN)); // Step
	delay(10);
}
