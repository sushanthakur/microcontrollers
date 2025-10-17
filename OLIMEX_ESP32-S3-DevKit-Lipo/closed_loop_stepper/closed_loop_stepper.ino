#include <AS5600.h>
#include <AccelStepper.h>
#include "freertos/semphr.h"

#define I2C_SDA 15
#define I2C_SCL 16
#define DIR_PIN 4
#define STEP_PIN 5
#define STEP_SPEED 2000
#define STEPS_PER_REV 800
#define ENCODER_RESOLUTION 4096
#define MAX_ANGLE_CHANGE 90
#define ANGLE_UPDATE_MS 2

AS5600 as5600;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
SemaphoreHandle_t posMutex;
TaskHandle_t angle_h;
TaskHandle_t step_h;

int target_pos = 0;
int current_pos = 0;

void step_t(void *pvParameters) 
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  while (true) {
    stepper.run();
    if (xSemaphoreTake(posMutex, portMAX_DELAY) == pdTRUE) {
      current_pos = stepper.currentPosition();
      xSemaphoreGive(posMutex);
    }
    // vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void angle_t(void *pvParameters)
{
  static uint32_t last_read = 0;
  static int last_angle = 0;
  
  while (true) {
    if (millis() - last_read >= ANGLE_UPDATE_MS) {
      last_read = millis();
      int angle = 0;
      
      if (as5600.isConnected()) {
        int cumulative = as5600.getCumulativePosition();
        int rev = as5600.getRevolutions();
        angle = map(cumulative - ENCODER_RESOLUTION * rev, 0, ENCODER_RESOLUTION, 0, 360) + 360 * rev;
        
        if (abs(angle - last_angle) < MAX_ANGLE_CHANGE) {
          last_angle = angle;
        }
        
        if (xSemaphoreTake(posMutex, portMAX_DELAY) == pdTRUE) {
            current_pos = angle;
            int del = target_pos - current_pos;
            if (del != 0) {
              stepper.moveTo(stepper.currentPosition() + (del * STEPS_PER_REV / 360));
            }
            xSemaphoreGive(posMutex);
          }
      } else {
        Serial.println("AS5600 connection lost!");
      }
      
      Serial.printf("Angle: %d, Pos: %d, Distance: %d\n", 
                    last_angle, stepper.currentPosition(), stepper.distanceToGo());
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeout(5);

  // Initialize AS5600
  if (!as5600.begin(8)) {
    Serial.println("AS5600 initialization failed!");
    while (1);
  }
  
  if (!as5600.isConnected()) {
    Serial.println("AS5600 not connected!");
    while (1);
  }
  
  as5600.setDirection(AS5600_CLOCK_WISE);
  as5600.resetPosition();

  esp_log_level_set("i2c.master", ESP_LOG_NONE); // Suppress all I2C master logs

  // Initialize mutex
  posMutex = xSemaphoreCreateMutex();
  if (posMutex == NULL) {
    Serial.println("Mutex creation failed!");
    while (1);
  }

  // Create tasks
  xTaskCreatePinnedToCore(
    angle_t,
    "angle_t",
    10000,
    NULL,
    0,
    &angle_h,
    0
  );

  xTaskCreatePinnedToCore(
    step_t,
    "step_t",
    10000,
    NULL,
    1,
    &step_h,
    1
  );

  // Stepper setup
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(STEP_SPEED);
  stepper.setSpeed(STEP_SPEED);
  stepper.setAcceleration(500);

  delay(1000);
}

void loop() {
  vTaskSuspend(NULL);
}
