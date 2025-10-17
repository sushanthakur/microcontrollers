// Used Libraries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <AS5600.h>

// Verbosity
#define DEBUG true
#define WARN true
#define ERROR true

// ESP CONFIG
#define BAUD 115200
#define I2C_SDA 15
#define I2C_SCL 16
#define AS5600_CLK_SPEED 100000 //Hz
#define ANGLE_UPDATE_MS 5

#define ENCODER_RES 4095

#define TCA_ADDR 0x70
#define AS5600_ADDR 0x36

// TIME CONST
#define DEBOUNCE_DELAY_HOME 100 // microseconds
#define REVERSE_INTERVAL 200 // miliseconds	

// STEPPER PARAM
#define STEPPER_NUM 8
#define STEPS_PER_REV 800 
#define STEPPER_MAX_SPEED 2000
#define STEPPER_RUN_SPEED 1000
#define STEPPER_ACEL 5000

AS5600 as5600;

// Task Handlers
TaskHandle_t serial_h;
TaskHandle_t home_steppers_h;
TaskHandle_t move_steppers_h;
TaskHandle_t check_angles_h;
TaskHandle_t grip_control_h;

// Flags
bool all_homed_flag = false;
bool stepper_homed_ind_flag[STEPPER_NUM] = {false};
bool new_pos_flag = false;
bool grip_update_flag = false;

// Pins
const int dir_pins[STEPPER_NUM] = { 4,5,6,7,17,18,8,3 };
const int step_pins[STEPPER_NUM] = { 9,10,11,12,13,14,21,47 };
const int hall_pins[STEPPER_NUM] = { 48,45,35,36,39,40,41,42 };

// Gripper Pins 
const int grip_dir_pin = 38;
const int grip_step_pin = 37;
const int grip_hall_pin = 40;
const int grip_fsr_pin = 41; 

// Fixed Params
const float min_shaft_angles[STEPPER_NUM] = { -1.57, -1.57, -3.14, -2.09, -3.14, -2.09, -3.14, -3.14 };
const float max_shaft_angles[STEPPER_NUM] = { 3.14, 1.57, 3.14, 2.09, 3.14, 2.09, 3.14, 3.14 };
const float reduction_ratios[STEPPER_NUM] = { 0.0, 17.25405444, 17.25405444, 17.25405444, 17.25405444, 17.25405444, 1 };
const float default_position[STEPPER_NUM] = { 0, 0.78, 0, 1.57, 0, 0.78, 0, 0};

// Changable Params
float target_shaft_angles[STEPPER_NUM] = {0.0};
float target_stepper_angles[STEPPER_NUM] = {0.0};
float target_stepper_steps[STEPPER_NUM] = {0.0};

// float currnet_shaft_angles[STEPPER_NUM] = {0.0};
float current_stepper_angles[STEPPER_NUM] = {0.0};

float feedback_stepper_angles[STEPPER_NUM] = {0.0};

// Stepper Initial Setup
AccelStepper stepper[STEPPER_NUM] = {
    AccelStepper(AccelStepper::DRIVER, step_pins[0], dir_pins[0]),
    AccelStepper(AccelStepper::DRIVER, step_pins[1], dir_pins[1]),
    AccelStepper(AccelStepper::DRIVER, step_pins[2], dir_pins[2]),
    AccelStepper(AccelStepper::DRIVER, step_pins[3], dir_pins[3]),
    AccelStepper(AccelStepper::DRIVER, step_pins[4], dir_pins[4]),
    AccelStepper(AccelStepper::DRIVER, step_pins[5], dir_pins[5]),
    AccelStepper(AccelStepper::DRIVER, step_pins[6], dir_pins[6]),
	AccelStepper(AccelStepper::DRIVER, step_pins[7], dir_pins[7]),
};
MultiStepper multi;

// TCA9548A multiplexer, bus selector function
void TCA9548A(uint8_t bus){
    Wire.beginTransmission(TCA_ADDR); // TCA9548A address
    Wire.write(1 << bus); // send byte to select bus
    Wire.endTransmission();
    Serial.print(bus);
}

void TCA_disable(){
	Wire.beginTransmission(TCA_ADDR);
	Wire.write(0);
	Wire.endTransmission();
}

// Fast string-to-integer conversion
inline int32_t fastAtoi(const char *str) {
	int32_t val = 0;
	while (*str >= '0' && *str <= '9') {
		val = val * 10 + (*str++ - '0');
	}
	return val;
}

/*
    =====================
	Utility Functions
    =====================
*/

void shaft_angles_to_steps() {
    // convert shaft angles to its respective stepper steps
    for(int i=0; i<STEPPER_NUM; i++){
		target_stepper_angles[i] = target_shaft_angles[i] * reduction_ratios[i];
		target_stepper_steps[i] = target_stepper_angles[i] * STEPS_PER_REV;
    }
}

void steppers_move_n_check() {
    // Reset stepper_homed_ind_flag array
    for (int i=0; i<STEPPER_NUM; i++) {
		stepper_homed_ind_flag[i] = false;
    }

    while (!all_homed_flag) {
		for (int i=0; i<STEPPER_NUM; i++){
			if (!stepper_homed_ind_flag[i]){
				stepper[i].runSpeed();
			}
			if(digitalRead(hall_pins[i]) == HIGH) {
				delayMicroseconds(DEBOUNCE_DELAY_HOME);
				if(digitalRead(hall_pins[i]) == HIGH) {
					stepper_homed_ind_flag[i] = true;
				}
			}
		}
		all_homed_flag = true;
		for (int i=0; i<STEPPER_NUM; i++) {
			if (!stepper_homed_ind_flag[i]) {
			all_homed_flag = false;
			}
		}
    }
}

void as5600_setup(){
	// I2C setup
	Wire.begin(I2C_SDA, I2C_SCL);
	for(int ch=0; ch<STEPPER_NUM; ch++){
		TCA9548A(ch);
		delay(1);

		as5600.begin(AS5600_ADDR);
		if (as5600.isConnected()) {
			Serial.print(ch); 
			Serial.println(": Connected");

			as5600.setDirection(AS5600_CLOCK_WISE);
			as5600.resetCumulativePosition();
		}
		else {
			Serial.print(ch);
			Serial.println(": AS5600 not found.");
		}

	}
	TCA_disable();
	esp_log_level_set("i2c.master", ESP_LOG_NONE);
}

/* 
    =========================
	TASKS
    =========================
*/
void home_steppers_t( void *pvParameters ) {
    while(true){
		if( !all_homed_flag ){
		
			steppers_move_n_check();
		
			// Make the stepper rotate in opposite direction
			for (int i=0; i<STEPPER_NUM; i++) {
			stepper[i].setSpeed(-STEPPER_RUN_SPEED); 
			}
		
			// Move in opposite direction for 'reverse_interval' time period
			unsigned long reverse_start_time = millis();
			while(millis() - reverse_start_time < REVERSE_INTERVAL){
				for(int i=0; i<STEPPER_NUM; i++){
					stepper[i].runSpeed();
				}
			}
		
			steppers_move_n_check();
		
			for(int i=0; i<STEPPER_NUM; i++){
				stepper[i].setCurrentPosition(0);
				stepper[i].setMaxSpeed(STEPPER_MAX_SPEED);
				stepper[i].setAcceleration(STEPPER_ACEL);
			}
			as5600_setup();
		}
    }
}

void serial_t( void *pvParameters ) {
    while(true){
		while(Serial.available() > 0) {
			char buffer[128];
			size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
			buffer[len] = '\0';
			String read_string = String(buffer);
			read_string.trim();
		
			if (read_string == "MOVE") {
				// READ ANGLES FROM SERIAL
				char buffer[128];
				size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
				buffer[len] = '\0';
				int index = 0;
				char *token = strtok(buffer, ",");
				while (token != NULL && index < STEPPER_NUM ) {
					float angle = fastAtoi(token);
					target_shaft_angles[index] = constrain(angle, min_shaft_angles[index], max_shaft_angles[index]);
					index++;
					token = strtok(NULL, ",");
				}
				if (index != STEPPER_NUM) {
					if (DEBUG) {
						Serial.println("DEBUG: ERROR: Incomplete arm position data");
					}				
				}
				shaft_angles_to_steps();
				new_pos_flag = true;
			} 
			else if (read_string == "HOME") {
				all_homed_flag = false;
			}
			else if (read_string == "DEFAULT"){
			// move steppers to default position
			}
		}
    }
}

void move_steppers_t( void *pvParameters ) {
    while(true){
		if ( new_pos_flag ){
			for( int i=0; i<STEPPER_NUM; i++ ){
				stepper[i].moveTo(target_stepper_steps[i]);
			}
			// multi.run();
			if ( !multi.run() ) {
				new_pos_flag = false;
			}
		}
    }
}

void check_angles_t( void *pvParameters ) {
	while (true) {
		if (!new_pos_flag){
			static uint32_t last_read = 0;
			static int last_angle = 0;
			if ( millis() - last_read >= ANGLE_UPDATE_MS ){
				last_read = millis();
				for ( int ch=0; ch<STEPPER_NUM; ch++ ){
					TCA9548A(ch);
					if (as5600.isConnected()){
						int cumulative = as5600.getCumulativePosition();
						int rev = as5600.getRevolutions();
						// feedback_stepper_angles[ch] = map(float(cumulative - ENCODER_RES * rev), 0, ENCODER_RES, 0, 360.0) + 360.0*rev;
						feedback_stepper_angles[ch] = ((float)(cumulative - ENCODER_RES * rev) * 360.0) / ENCODER_RES + 360.0 * rev;
						
						float del = target_stepper_angles[ch] - feedback_stepper_angles[ch];
						if (fabs(del) <= 5) {
							target_shaft_angles[ch] += del;
							new_pos_flag = true;
						}
					} 
					else {
						Serial.print(ch);
						Serial.println(": AS5600 connection lost");
					}
				}
			}
		}
		vTaskDelay(5/portTICK_PERIOD_MS);
	}
}

void grip_control_t( void *pvParameters ) {
    while(true){
		if(grip_update_flag){
			// logic to be implemented later    
		}
    }
}

void setup(){

    Serial.begin(115200);

    // Stepper Setup
    for(int i=0; i<STEPPER_NUM; i++){
		pinMode(dir_pins[i], OUTPUT);
		pinMode(step_pins[i], OUTPUT);
		pinMode(hall_pins[i], INPUT);
		stepper[i].setMaxSpeed(STEPPER_MAX_SPEED);
		stepper[i].setSpeed(STEPPER_RUN_SPEED);
		multi.addStepper(stepper[i]);
    }

    // Task Setup
    xTaskCreatePinnedToCore (
		home_steppers_t,
		"home_steppers_t",
		10000, // Stack Size
		NULL,
		0,
		&home_steppers_h,
		0
    );

    xTaskCreatePinnedToCore (
		serial_t,
		"serial_t",
		10000, // Stack Size
		NULL,
		0,
		&serial_h,
		0
    );

    xTaskCreatePinnedToCore (
		move_steppers_t,
		"move_steppers_t",
		10000, // Stack Size
		NULL,
		0,
		&move_steppers_h,
		1
    );

    xTaskCreatePinnedToCore (
		check_angles_t,
		"check_angles_t",
		10000, // Stack Size
		NULL,
		0,
		&check_angles_h,
		1
    );

    xTaskCreatePinnedToCore (
		grip_control_t,
		"grip_control_t",
		10000, // Stack Size
		NULL,
		0,
		&grip_control_h,
		0
    );

}

void loop(){
    vTaskSuspend(NULL);    // no use of this task
}