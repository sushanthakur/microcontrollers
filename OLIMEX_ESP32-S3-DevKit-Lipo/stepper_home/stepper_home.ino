#include <AccelStepper.h>

#define switchPin 35
#define dirPin 4
#define stepPin 3

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

const unsigned long debounceDelay = 100;

void homeStepper();

void setup() {
  Serial.begin(115200);
  pinMode(switchPin, INPUT);

  stepper.setMaxSpeed(2000);
  stepper.setSpeed(1500);

  homeStepper();
}

void loop() {
  stepper.moveTo(-3500);
  stepper.run();
}

void homeStepper() {
  Serial.println("Starting homing...");

  // Move toward the limit switch until triggered
  stepper.setSpeed(2000); // Set initial speed for homing
  while (true) {
    stepper.runSpeed();

    if (digitalRead(switchPin) == HIGH) {
      delayMicroseconds(debounceDelay);
      if (digitalRead(switchPin) == HIGH) {
        Serial.println("Switch triggered!");
        break;
      }
    }
  }

  // Move away from the switch for a fixed duration
  stepper.setSpeed(-1000); // Reverse direction
  unsigned long startTime = millis();
  unsigned long interval = 200;

  while (millis() - startTime < interval) {
    stepper.runSpeed();
  }

  // Move back toward the switch slowly for precise homing
  stepper.setSpeed(1500); // Move toward switch again
  while (true) {
    stepper.runSpeed();

    if (digitalRead(switchPin) == HIGH) {
      delayMicroseconds(debounceDelay);
      if (digitalRead(switchPin) == HIGH) {
        Serial.println("Switch triggered!");
        break;
      }
    }
  }

  // Set the current position as zero and configure acceleration
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(5000);
}