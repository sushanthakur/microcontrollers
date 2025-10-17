#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println();
  Serial.println("I2C Device Scanner");
}

void loop() {
  int address, error, devices=0;
  for(address=1; address<128; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    // Serial.println(error);
    if (error == 0){
      Serial.print("Address = 0x");
      if (address<16){
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
    devices++;
  }
  if(devices == 0){
    Serial.println("No devices found");
  }
  delay(5000);
}
