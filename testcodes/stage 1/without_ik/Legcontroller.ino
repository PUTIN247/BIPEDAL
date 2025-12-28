#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "LegController.h"

// I2C Pins
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Instantiate the leg controller (Hip=0, Knee=1, Ankle=2)
LegController leg(&pwm, 0, 1, 2);

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(200);

  // --- CONFIGURATION ---
  // Set offsets if your servos aren't perfectly straight at 90
  leg.setOffsets(0.0, 0.0, 0.0);
  
  // Set speed (2.5x slower than base for safety)
  leg.setSpeed(2.5);

  Serial.println("Initializing Leg...");
  leg.begin(); // Moves to Home
  delay(1000);
}

void loop() {
  // Run the cycle continuously
  leg.stepCycle();
}
