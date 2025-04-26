/*
Features:
- Reads acceleration and gyroscope data from an MPU6050 sensor.
- Uses a Kalman filter to smooth and correct sensor readings.
- Reads ambient light levels using an LDR sensor.
- Communicates sensor data wirelessly via Bluetooth.
- Sends processed values over serial for debugging.
Applications:
- Motion tracking and gesture recognition.
- Balancing robots and stabilization systems.
- Smart lighting systems based on ambient light levels.
- Wearable motion analysis and health tracking.
- Wireless remote control based on tilt movements.
*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"
Adafruit_MPU6050 mpu;  // Create MPU6050 sensor object
BluetoothSerial SerialBT;  // Create Bluetooth serial object
const int LDRSensor = 34;  // Define the LDR sensor pin
// Kalman filter variables
float x_angle = 0, y_angle = 0; // Filtered angle estimates
float x_bias = 0, y_bias = 0;   // Gyroscope bias correction
float P[2][2] = { { 1, 0 }, { 0, 1 } };  // Error covariance matrix
// Kalman filter tuning parameters
// Adjust these for different levels of noise filtering and responsiveness
// Quick and Less Smooth Response
float q_angle = 0.01;    // Trust new angle estimates more
float q_bias = 0.01;     // Faster correction of gyroscope drift
float r_measure = 0.01;  // Trust sensor readings more (less smoothing)
// // Alternative: Optimum Response
// const float q_angle = 0.001; // Process noise
// const float q_bias = 0.003;
// const float r_measure = 0.03; // Measurement noise
// // Alternative: Slow and smooth response
// float q_angle = 0.0001;   // Trust new angle estimates more
// float q_bias = 0.0005;    // Faster correction of gyroscope drift
// float r_measure = 0.05;   // Trust sensor readings more (less smoothing)
void setup() {
Serial.begin(115200);  // Initialize Serial Monitor
SerialBT.begin("ESP32_BT");  // Initialize Bluetooth with device name "ESP32_BT"
// Initialize MPU6050 sensor
if (!mpu.begin()) {
  Serial.println("MPU6050 not found!");  
  while (1) delay(10);  // Halt execution if MPU6050 is not detected
}
// Set MPU6050 configuration
mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // Set accelerometer range
mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // Set gyroscope range
mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);    // Apply a low-pass filter
delay(100);  // Allow settings to take effect
}
// Kalman filter function to smooth sensor data
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias) {
float rate = newRate - bias;  // Remove bias from gyroscope rate
angle += dt * rate;  // Estimate new angle
// Update estimation error covariance
P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + q_angle);
P[0][1] -= dt * P[1][1];
P[1][0] -= dt * P[1][1];
P[1][1] += q_bias * dt;
// Compute Kalman gain
float S = P[0][0] + r_measure;
float K[2] = { P[0][0] / S, P[1][0] / S };
// Update estimates with measurement
float y = newAngle - angle;
angle += K[0] * y;
bias += K[1] * y;
// Update error covariance matrix
P[0][0] -= K[0] * P[0][0];
P[0][1] -= K[0] * P[0][1];
P[1][0] -= K[1] * P[0][0];
P[1][1] -= K[1] * P[0][1];
return angle;  // Return the filtered angle
}
void loop() {
sensors_event_t a, g, temp;  // Variables to store sensor readings
mpu.getEvent(&a, &g, &temp); // Get sensor data
int ldrValue = analogRead(LDRSensor);  // Read LDR sensor value
int outputValue = (ldrValue < 2000) ? 0 : 1;  // Determine light or dark condition
// Calculate time difference for Kalman filter
static unsigned long prevTime = millis();
float dt = (millis() - prevTime) / 1000.0;  // Convert to seconds
prevTime = millis();
// Apply Kalman filter to smooth sensor data
float filteredX = kalmanFilter(a.acceleration.x, g.gyro.x, dt, x_angle, x_bias);
float filteredY = kalmanFilter(a.acceleration.y, g.gyro.y, dt, y_angle, y_bias);
// Format data for Bluetooth transmission
String btData = String(filteredX, 2) + "," + String(filteredY, 2) + "," + String(outputValue);
// Send data over Bluetooth and Serial for debugging
SerialBT.println(btData);
Serial.println(btData);
// Small delay to stabilize loop execution
// delay(1); // Uncomment if needed
}
