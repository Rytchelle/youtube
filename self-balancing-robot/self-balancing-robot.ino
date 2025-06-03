// Libraries
#include <Wire.h>
#include <MPU6050_light.h>

// Stepper motor pins
#define DIR_PIN 2
#define STEP_PIN 4
#define DIR_PIN2 12
#define STEP_PIN2 14

// MPU6050 object
MPU6050 mpu(Wire);

// PID variables
double setpoint = 0;  // Desired angle of 0 degrees
double input, output;

// PID gain control
double Kp = 2350.0;
double Ki = 8.0;
double Kd = 12.0;

// Manual PID control vars
double error, lastError = 0;
double integral = 0;
double derivative;
double P, I, D;
double dt = 0.005;  // 5ms in seconds

// Timing variables
unsigned long lastPIDUpdate = 0;
const unsigned long PIDInterval = 15;  // ms

// Stepper variables
int motorSpeed = 0;
unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;

// Task handles
TaskHandle_t pidTaskHandle;

// Function prototypes
void pidLoop(void *parameter);
void motorControl();

void setup() {
  Serial.begin(115200);

  // Initialise MPU6050
  Wire.begin();
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
  Serial.println("MPU6050 initialized successfully");

  // Calibrate MPU6050
  mpu.calcOffsets(true, true);

  // Initialise steppers
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);

  // Create a task for the PID loop on core 0
  xTaskCreatePinnedToCore(pidLoop, "PID Task", 10000, NULL, 2, &pidTaskHandle, 0);
  delay(4000);
}

void loop() {
  // Motor control logic runs on core 1
  motorControl();
}

void pidLoop(void *parameter) {
  while (true) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastPIDUpdate >= PIDInterval) {
      lastPIDUpdate = currentMillis;

      // Update MPU6050 angle
      mpu.update();
      double angle = mpu.getAngleX();  // Read the X-axis angle

      // Smoothing
      static double smoothedAngle = 0.0;
      smoothedAngle += 0.03 * (angle - smoothedAngle);

      // Manual PID calculation
      input = smoothedAngle;
      error = setpoint - input;

      // Termos PID
      P = Kp * error;
      integral += error * dt;

      // Limit the value of the integral term to avoid windup
      if (integral > 30) integral = 30;
      if (integral < -30) integral = -30;

      I = Ki * integral;
      derivative = (error - lastError) / dt;
      D = Kd * derivative;
      lastError = error;

      output = P + I + D;
      motorSpeed = (int)output;

      // Set motor directions based on PID output
      if (motorSpeed > 0) {
        digitalWrite(DIR_PIN, HIGH);
        digitalWrite(DIR_PIN2, HIGH);
      } else if (motorSpeed < 0) {
        digitalWrite(DIR_PIN, LOW);
        digitalWrite(DIR_PIN2, LOW);
      }

      // Debug
      Serial.print("Angle: ");
      Serial.print(input);
      Serial.print(" | P: ");
      Serial.print(P);
      Serial.print(" I: ");
      Serial.print(I);
      Serial.print(" D: ");
      Serial.print(D);
      Serial.print(" | Output: ");
      Serial.print(output);
      Serial.print(" | Motor Speed: ");
      Serial.println(motorSpeed);
    }

    delay(1);
  }
}

void motorControl() {
  // Step pulses
  if (abs(motorSpeed) > 0) {
    unsigned long stepInterval1 = 1000000 / abs(motorSpeed);
    if (micros() - lastStepTime1 >= stepInterval1) {
      lastStepTime1 = micros();
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
    }
  }

  if (abs(motorSpeed) > 0) {
    unsigned long stepInterval2 = 1000000 / abs(motorSpeed);  // Microseconds per step
    if (micros() - lastStepTime2 >= stepInterval2) {
      lastStepTime2 = micros();
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(2);  // Pulse width
      digitalWrite(STEP_PIN2, LOW);
    }
  }
}