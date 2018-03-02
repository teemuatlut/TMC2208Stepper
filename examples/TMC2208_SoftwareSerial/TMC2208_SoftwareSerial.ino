// Author Teemu Mäntykallio, 2017-04-07

// Define pins
#define EN_PIN    38  // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN  54  // Step on rising edge
#define RX_PIN    63  // SoftwareSerial pins
#define TX_PIN    40  //

#include <TMC2208Stepper.h>

// Create driver that uses SoftwareSerial for communication
TMC2208Stepper driver = TMC2208Stepper(RX_PIN, TX_PIN);

void setup() {
  driver.beginSerial(115200);
  // Push at the start of setting up the driver resets the register to default
  driver.push();
  // Prepare pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  driver.pdn_disable(true);     // Use PDN/UART pin for communication
  driver.I_scale_analog(false); // Use internal voltage reference
  driver.rms_current(500);      // Set driver current = 500mA, 0.5 multiplier for hold current and RSENSE = 0.11.
  driver.toff(2);               // Enable driver in software

  digitalWrite(EN_PIN, LOW);    // Enable driver in hardware
}

void loop() {
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
  delay(1);
}
