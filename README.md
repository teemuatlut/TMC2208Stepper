# TMC2130Stepper
Arduino library for Trinamic TMC2208 Stepper driver


examples/TMC2208_Simple
```cpp
#define EN_PIN    13
#define DIR_PIN   11
#define STEP_PIN  54

#include <TMC2208Stepper.h>
TMC2208Stepper driver = TMC2208Stepper(&Serial);

void setup() {
	Serial.begin(115200);
	while(!DRIVER);

	pinMode(EN_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);

	driver.pdn_disable(1);
	driver.setCurrent(500, 0.11, 0.5);
	driver.toff(0x2);

	digitalWrite(13, LOW);
}

void loop() {
	digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
	delay(10);
}
```

