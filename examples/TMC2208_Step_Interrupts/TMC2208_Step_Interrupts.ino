// Author Teemu Mäntykallio, 2017-04-07

// Define pins
#define EN_PIN    38 // LOW: Driver enabled. HIGH: Driver disabled
#define DIR_PIN   55 // Set stepping direction
#define STEP_PIN  54 // Step on rising edge

#define STEP_PORT PORTF
#define PORT_PIN  0 // PORTF0 <=> Pin 54 // ATMEGA

#include <TMC2208Stepper.h>
TMC2208Stepper driver = TMC2208Stepper(&Serial1);  // Create driver

ISR(TIMER1_COMPA_vect){
  STEP_PORT |= 1 << PORT_PIN;
  STEP_PORT &= ~(1 << PORT_PIN);
  //digitalWrite(STEP_PIN, !digitalRead(STEP_PIN)); Would work also, but slower
}

void setup() {
  driver.beginSerial(115200);
  driver.push();

  // Prepare pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  driver.pdn_disable(true);     // Use PDN/UART pin for communication
  driver.I_scale_analog(false); // Use internal voltage reference
  driver.rms_current(500);      // Set driver current = 500mA, 0.5 multiplier for hold current and RSENSE = 0.11.
  driver.toff(2);               // Enable driver in software

  // Set stepper interrupt
  cli();      // Stop interrupts
              // Set timer1 interrupt at 1kHz:
  TCCR1A = 0; // Set entire TCCR1A register to 0
  TCCR1B = 0; // Same for TCCR1B
  TCNT1  = 0; // Initialize counter value to 0
              // Set compare match register for 1hz increments
  OCR1A = 520;//  = (16*10^6) / (1*1024) - 1 (must be <65536)
              // Turn on CTC mode
  TCCR1B |= (1 << WGM12);
              // Set CS11 bits for 8 prescaler
  TCCR1B |= (1 << CS11);
              // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();      // Allow interrupts

  digitalWrite(EN_PIN, LOW); // Enable driver
}

void loop() {}
