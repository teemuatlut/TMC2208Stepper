// Author Teemu Mantykallio, 2017-04-07

// Define pins
// 1st motor
#define EN_PIN_1    13												// LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_1  54												// Step on rising edge
#define CS_PIN_1    14												// driver UART select pin

// 2nd motor
#define EN_PIN_2    15												// LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_2  55												// Step on rising edge
#define CS_PIN_2    16												// driver UART select pin

#include <TMC2208Stepper.h>											// Include library

#define SHARED_UART	Serial1

TMC2208Stepper driver1 = TMC2208Stepper(&SHARED_UART, true, CS_PIN_1);	// Create drivers and use SHARED_UART for bi-directional communication
TMC2208Stepper driver2 = TMC2208Stepper(&SHARED_UART, true, CS_PIN_2);	// Create drivers and use SHARED_UART for bi-directional communication

void setup() {
	SHARED_UART.begin(115200);			// Init used serial port
	while(!SHARED_UART);				// Wait for port to be ready

    // Prepare pins
	pinMode(EN_PIN_1, OUTPUT);
	pinMode(STEP_PIN_1, OUTPUT);

	pinMode(EN_PIN_2, OUTPUT);
	pinMode(STEP_PIN_2, OUTPUT);

	// prepare drivers
	driver1.pdn_disable(1);				// Use PDN/UART pin for communication
	driver1.replyDelay = 2;				// Delay in ms to wait for driver to reply (on Due 1ms is enough)
	driver1.I_scale_analog(0);			// Adjust current from the registers
	driver1.rms_current(500);			// Set driver current 500mA
	driver1.toff(0x2);					// Enable driver
	driver1.microsteps(32);				// Set microsteps

	digitalWrite(EN_PIN_1, LOW);		// Enable driver


	driver2.pdn_disable(1);				// Use PDN/UART pin for communication
	driver2.replyDelay = 2;				// Delay in ms to wait for driver to reply (on Due 1ms is enough)
	driver2.I_scale_analog(0);			// Adjust current from the registers
	driver2.rms_current(500);			// Set driver current 500mA
	driver2.toff(0x2);					// Enable driver
	driver1.microsteps(16);				// Set microsteps

	digitalWrite(EN_PIN_2, LOW);		// Enable driver
}

void loop() {
	digitalWrite(STEP_PIN_1, !digitalRead(STEP_PIN_1));	// Step
	delay(2);
	digitalWrite(STEP_PIN_2, !digitalRead(STEP_PIN_2));	// Step
	delay(2);
}
