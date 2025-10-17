#include <ESP32Servo.h>

Servo servo1;

void setup() {
  Serial.begin(115200);
  servo1.attach(13);
  servo1.write(2);
}

void loop() {
  int x = analogRead(34);
  Serial.println(x);
  if(x<625 ){
    pressSpace();
  }
}

void pressSpace() {
  servo1.write(12);
  delay(20);
  servo1.write(2);
}