/*
Main firmware for iLight 2024 Liminal Minimal
Framework: Arduino with FreeRTOS (real time operating system)
Hardware: ESP32 DEVKIT C or ESP32 DEVKIT V1, Liminal Minimal PCB, VL53L1X ToF sensors
Author: Brandon Tay Kaiheng (Contact: brandontay6@gmail.com)
*/

// Comment out to disable debug prints 
// Debug prints will slow down actual task execution due to multiple cores competing for single UART resource
#define DEBUG

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// Global Variables
// Number of sensors connected (adjust accordingly)
const uint8_t sensor_count = 1;

// PWM parameters
const uint32_t PWM_freq = 5000; // PWM freq in Hz
const uint8_t PWM_channel = 0;  
const uint8_t PWM_res = 8;      // PWM resolution in number of bits

// IO pin for PWM (LED driver)
const uint8_t LEDPin = 19;

// IO pins connected to XSHUT pin (ToF sensor reset)
const uint8_t xshutPins[] = {4, 5, 2, 17, 0, 16};

// Sensor distance threshold
// uint16_t sensor_threshold = 950;
uint16_t sensor_threshold = 10;

// // I2C pins 
// const uint8_t SDA = 21;
// const uint8_t SCL = 22;

// RTOS task handlers
static TaskHandle_t handle_1;
static TaskHandle_t handle_2;

// Instantiate VL53L1X ToF sensor class objects
VL53L1X sensors[sensor_count];


// Function set up sensor
// Each sensor is turned on and off using the XSHUT pin wired to a digital pin on the ESP32
// Initialize each sensor
// Address is changed digitally by writing via I2C once during the setup() section
// After all unique addresses are assigned, begin continuous mode routine for each sensor
void sensor_init(const uint8_t count, const uint8_t xshut_arr[6]) {
  // Reset all sensors by driving XSHUT pin to logic LOW
  for (int i=0; i < count; i++) {
    pinMode(xshut_arr[i], OUTPUT);
    digitalWrite(xshut_arr[i], LOW);
  }

  delay(10);

  // Enable, initialize and start each sensor
  for (int i=0; i < count; i++) {
    pinMode(xshut_arr[i], INPUT); // Force all XSHUT pins to logic HIGH
    delay(10);

    // VL53L1X configuration
    sensors[i].setTimeout(500);      // Set timeout for sensor reading if sensor not ready
    sensors[i].setROICenter(199);    // Set Region-of-Interest center for field of vision (default SPAD 199)
    sensors[i].setROISize(10, 10);   // Set Region-of-interest size for sensor (16x16 default, 4x4 min)

    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor");
      ESP.restart();
      Serial.println(i);
      while (1);
    }

    // Default address for all sensors are 0x29
    // Change each sensors' address to a unique value for I2C communication, starting from 0x2A
    sensors[i].setAddress(0x2A + i);

    // Begin continuous ranging mode
    sensors[i].startContinuous(50); // Sample at 50ms intervals
  }
}

// Function for idle state LED control
void pulse_idle() {
  // Increase the LED brightness 
  for(int dutyCycle = 30; dutyCycle <= 120; dutyCycle++){   
    ledcWrite(PWM_channel, dutyCycle);
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
  // delay(1000);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Decrease the LED brightness 
  for (int dutyCycle = 120; dutyCycle >= 30; dutyCycle--) {
    ledcWrite(PWM_channel, dutyCycle);   
    // delay(30);
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
  // delay(1000);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Function for awakened state LED control
void pulse_awaken() {
  // Increase the LED brightness 
  for (int dutyCycle = 30; dutyCycle <= 255; dutyCycle++) {   
    ledcWrite(PWM_channel, dutyCycle);
    // delay(20);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
  // delay(1000);
  vTaskDelay(1500 / portTICK_PERIOD_MS);

  // Decrease the LED brightness 
  for (int dutyCycle = 255; dutyCycle >= 30; dutyCycle--) {
    ledcWrite(PWM_channel, dutyCycle);   
    // delay(5);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
  // delay(1000);
  //vTaskDelay(1000 / portTICK_PERIOD_MS);
}


// RTOS Task Functions
void task_1(void* pvParameters) {
  while (1) {
    // task function
    pulse_idle();
    #ifdef DEBUG
      Serial.println("Task 1 active");
      vTaskDelay(50 /portTICK_PERIOD_MS);
    #endif
  }
}

void task_2(void* pvParameters) {
  while (1) {
    // task function 
    for (int i=0; i < sensor_count; i++) {
      uint16_t sensor_val = sensors[i].read();

      // Check for time outs (I2C connection loss etc.)
      if (sensors[i].timeoutOccurred()) {
        ESP.restart();
      }

      #ifdef DEBUG
        Serial.println("Task 2 active");
        vTaskDelay(50 / portTICK_PERIOD_MS);
      #endif

      // Check if threshold is crossed (person has triggered sensor)
      // If triggered, suspend task_1, call pulse_awaken() sequence, resume task_1
      if (sensor_val < sensor_threshold) {
        //vTaskDelay(20);
        //uint16_t sensor_val = sensors[i].read();
        if (sensor_val < sensor_threshold) {
          vTaskSuspend(handle_1);
          pulse_awaken();
          vTaskDelay(20 / portTICK_PERIOD_MS);
          vTaskResume(handle_1);
          #ifdef DEBUG
            Serial.println("Sensor triggered");
            vTaskDelay(500 / portTICK_PERIOD_MS);
          #endif
        }
      }

      #ifdef DEBUG
        Serial.print(sensor_val);
        if (sensors[i].timeoutOccurred()) { 
          Serial.print(" TIMEOUT"); 
          }
          Serial.print('\t');
      #endif
    }
  }
}


// Set up function, run once
void setup() {
  // Begin Serial UART
  Serial.begin(115200);

  // I2C configuration
  Wire.begin();
  Wire.setClock(100000); // Set I2C speed in Hz

  // PWM configuration (using built in ESP32 led-c)
  ledcSetup(PWM_channel, PWM_freq, PWM_res);
  ledcAttachPin(LEDPin, PWM_channel);

  // Initialize sensors
  sensor_init(sensor_count, xshutPins);

  // Create RTOS tasks

  // Task 1: Idle pulsing 
  xTaskCreatePinnedToCore(
    task_1,    // Task function
    "task 1",  // Label
    2048,      // Stack memory size
    NULL,      // Task input parameter
    1,         // Priority
    &handle_1, // Pointer to task handle
    0          // CPU Core
  );

// Task 2: Read sensor array 

  xTaskCreatePinnedToCore(
    task_2,    // Task function
    "task 2",  // Label
    4096,      // Stack memory size
    NULL,      // Task input parameter
    1,         // Priority
    &handle_2, // Pointer to task handle
    1          // CPU Core
  );
}

// Unused, required for Arduino framework preprocessor to compile properly
void loop() {

}
