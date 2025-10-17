#include <Wire.h>

void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1<<bus);
  Wire.endTransmission();
  Serial.print(bus);
}

void Setup() {
  Serial.begin(115200);

  Wire.begin();

  
}