#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

VL53L0X sensor;

/////////////////////// Inputs/Outputs ///////////////////////
int Analog_in = 9;
Servo myservo;  // Create a servo object to control a servo, later attached to D9
////////////////////////////////////////////////////////////

//////////////////////// Variables //////////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;  // Variables for time control
float distance_previous_error, distance_error;
int period = 50;  // Refresh rate period of the loop is 50ms
////////////////////////////////////////////////////////////

/////////////////// Moving Average Variables ////////////////
const int MA_SIZE = 4;  // Size of the moving average buffer
float distanceBuffer[MA_SIZE] = {0};  // Buffer to hold distance readings
int bufferIndex = 0;  // Index to track the current buffer position
////////////////////////////////////////////////////////////

/////////////////// PID Constants ///////////////////////////
float kp = 8;      // Mine was 8
float ki = 0.2;    // Mine was 0.2
float kd = 3100;   // Mine was 3100
float distance_setpoint = 24;  // Should be the distance from the sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Serial.println("PRINT");
  Wire.begin();
  myservo.attach(9);  // Attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(1500);  // Initialize servo to neutral position

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize the sensor!");
    while (1) {}
  }

  sensor.startContinuous();
  time = millis();
  Serial.println("PRINT2");
}

float movingAverage(float newDistance) {
  // Update the buffer with the new distance reading
  distanceBuffer[bufferIndex] = newDistance;
  bufferIndex = (bufferIndex + 1) % MA_SIZE;

  // Calculate the sum of all values in the buffer
  float sum = 0;
  for (int i = 0; i < MA_SIZE; i++) {
    sum += distanceBuffer[i];
  }

  // Return the average value
  return sum / MA_SIZE;
}

void loop() {
  if (millis() > time + period) {
    time = millis();
    float newDistance = sensor.readRangeContinuousMillimeters() / 10.0;
    distance = movingAverage(newDistance);  // Use the moving average

    Serial.println("," + String(distance));

    distance_error = distance_setpoint - distance;
    PID_p = kp * distance_error;
    float dist_difference = distance_error - distance_previous_error;
    PID_d = kd * ((distance_error - distance_previous_error) / period);

    if (-4 < distance_error && distance_error < 4) {
      PID_i = PID_i + (ki * distance_error);
    } else {
      PID_i = 0;
    }

    PID_total = PID_p + PID_i + PID_d;
    PID_total = map(PID_total, -150, 150, 1000, 2000);

    // Constrain the pulse width to typical servo limits (1000 to 2000 microseconds)
    PID_total = constrain(PID_total, 1000, 2000);

    myservo.writeMicroseconds(PID_total);
    distance_previous_error = distance_error;
  }
}

 