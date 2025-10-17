from machine import Pin, I2c
import time

led = Pin(2, Pin.OUT)

while(1):
  led.on()
  time.sleep_ms(1000)
  led.off()
  time.sleep_ms(1000)
