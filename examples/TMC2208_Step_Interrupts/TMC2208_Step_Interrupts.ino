// Author Teemu Mäntykallio, 2017-04-07

// Define pins
#define EN_PIN    13 															// LOW: Driver enabled. HIGH: Driver disabled
#define DIR_PIN   11 															// Set stepping direction
#define STEP_PIN  54 															// Step on rising edge

#define STEP_PORT PORTF
#define PORT_PIN  0 // PORTF0 <=> Pin 54 // ATMEGA

#include <TMC2208Stepper.h>												// Include library
TMC2208Stepper driver = TMC2208Stepper(&Serial);	// Create driver

ISR(TIMER1_COMPA_vect){
  STEP_PORT |= 1 << PORT_PIN;
  STEP_PORT &= ~(1 << PORT_PIN);
  //digitalWrite(STEP_PIN, !digitalRead(STEP_PIN)); Would work also, but slower
}

void setup() {
	Serial.begin(115200);														// Init used serial port
	while(!Serial);																	// Wait for port to be ready

	// Prepare pins
	pinMode(EN_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);

	driver.pdn_disable(1);													// Use PDN/UART pin for communication
	driver.rms_current(500, 0.5, 0.11);							// Set driver current = 500mA, 0.5 multiplier for hold current and RSENSE = 0.11.
	driver.toff(0x2);																// Enable driver

	// Set stepper interrupt
	cli();//stop interrupts
	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 1hz increments
	OCR1A = 520;// = (16*10^6) / (1*1024) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS11 bits for 8 prescaler
	TCCR1B |= (1 << CS11);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);
	sei();//allow interrupts

	digitalWrite(13, LOW);													// Enable driver
}

void loop() {}
