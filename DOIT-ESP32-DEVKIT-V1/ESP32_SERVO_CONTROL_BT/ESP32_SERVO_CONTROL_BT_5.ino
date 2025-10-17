
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

int SWITCH_ON_ANGLE_18 = 100;
int SWITCH_OFF_ANGLE_18 = 150;

int SWITCH_ON_ANGLE_5 = 80;
int SWITCH_OFF_ANGLE_5 = 25;

Servo servo18;
Servo servo5;

int servo18Pin = 13;
int servo5Pin = 12;

int state = 0;

BluetoothSerial SerialBT;

void setup() {
  SerialBT.begin("ESP32");
  servo18.attach(servo18Pin);
  servo5.attach(servo5Pin);
}

void loop() {
  state = 0;
  state = SerialBT.parseInt();
  switch(state) {
    case 11:
      on(18);
      off(5);
      SerialBT.println();
      break;
    case 10:
      off(18);
      SerialBT.println();
      break;
    case 21:
      on(5);
      off(18);
      SerialBT.println();
      break;
    case 20:
      off(5);
      SerialBT.println();
      break;
  }
  delay(30);
  servo18.detach();
  servo5.detach();
}

void on(int n) {
  if(n==18) {
    servo18.attach(servo18Pin);
    servo18.write(SWITCH_ON_ANGLE_18);
    SerialBT.print("18W on. ");
  }
  else if(n==5){
    servo5.attach(servo5Pin);
    servo5.write(SWITCH_ON_ANGLE_5);
    SerialBT.print("5W on. ");
  }
}

void off(int n) {
  if(n==18){
    servo18.attach(servo18Pin);
    servo18.write(SWITCH_OFF_ANGLE_18);
    SerialBT.print("18w off. ");
  }
  else if(n==5){
    servo5.attach(servo5Pin);
    servo5.write(SWITCH_OFF_ANGLE_5);
    SerialBT.print("5W off. ");
  }
}