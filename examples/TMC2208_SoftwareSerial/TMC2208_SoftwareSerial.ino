// Author Teemu Mäntykallio, 2017-04-07

// Define pins
#define EN_PIN    13 															// LOW: Driver enabled. HIGH: Driver disabled
#define DIR_PIN   11 															// Set stepping direction
#define STEP_PIN  54 															// Step on rising edge
#define TX_PIN 	  52															// SoftwareSerial
#define RX_PIN    50															// 

#include <SoftwareSerial.h>
SoftwareSerial tmc_sw = SoftwareSerial(RX_PIN, TX_PIN);

#include <TMC2208Stepper.h>												// Include library
TMC2208Stepper driver = TMC2208Stepper(&tmc_sw);	// Create driver that uses SoftwareSerial for communication

void setup() {
	tmc_sw.begin(115200);														// Init used serial port
	while(!tmc_sw);																	// Wait for port to be ready

	// Prepare pins
	pinMode(EN_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);

	driver.pdn_disable(1);													// Use PDN/UART pin for communication
	driver.rms_current(500, 0.5, 0.11);							// Set driver current = 500mA, 0.5 multiplier for hold current and RSENSE = 0.11.
	driver.toff(0x2);																// Enable driver

	digitalWrite(13, LOW);													// Enable driver
}

void loop() {
	digitalWrite(STEP_PIN, !digitalRead(STEP_PIN)); // Step
	delay(10);
}
