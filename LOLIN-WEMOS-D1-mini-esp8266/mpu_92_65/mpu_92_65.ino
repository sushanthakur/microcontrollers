#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial.println();

  Wire.begin();
  delay(500);
  mpu.initialize();

}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float accelerationX = ax / 16384.0;
  float accelerationY = ay / 16384.0;
  float accelerationZ = az / 16384.0;

  int16_t rx, ry, rz;
  mpu.getRotation(&rx, &ry, &rz);
  float rotationX = rx / 16384.0;
  float rotationY = ry / 16384.0;
  float rotationZ = rz / 16384.0;

  Serial.print("RX="); Serial.print(rotationX,2); Serial.print(", ");
  Serial.print("RY="); Serial.print(rotationY,2); Serial.print(", ");
  Serial.print("RZ="); Serial.print(rotationZ,2);
  
  Serial.print("----");
  Serial.print("AX="); Serial.print(accelerationX,2); Serial.print(", ");
  Serial.print("AY="); Serial.print(accelerationY,2); Serial.print(", ");
  Serial.print("AZ="); Serial.print(accelerationZ,2);
  Serial.println();

  delay(50);
}







