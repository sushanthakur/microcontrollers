#include "LiquidCrystal_I2C.h"

int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows); 

// lcd.init(): initializes the display
// lcd.backlight(): turns the LCD backlight on
// lcd.setCursor(int column, int row): sets the cursor to the specified column and row
// lcd.print(String message): displays the message on the display
// lcd.clear(): clears the display

void printHello(void);

void setup(){
  Serial.begin(115200);
  Serial.println();
  
  lcd.init();
  lcd.backlight();
  printHello();
}

void loop(){
  
}

void printHello(void){
  char str[] = {'H','e','l','l','o',',',' ','W','o','r','l','d','!'};
  for(int i=0; i < sizeof(str); i++){
    lcd.setCursor(i, 0);
    lcd.write(str[i]);
    delay(50);
  }
}









