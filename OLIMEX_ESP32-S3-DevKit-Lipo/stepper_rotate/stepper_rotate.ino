#include <AccelStepper.h>

#define STEP_PIN 4
#define DIR_PIN 5

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  stepper.setMaxSpeed(2000);
  stepper.setSpeed(2000);
}

void loop() {
  stepper.runSpeed(); //This does not block the code
}   