/*
  Stage1_IK_Test.ino
  Main controller for Single Leg IK Testing
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h" // Include our new custom library

// I2C Setup
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Servo Driver Object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- LEG CONFIGURATION ---
// Define channels: Hip=0, Knee=1, Ankle=2
BipedLeg leftLeg(&pwm, 0, 1, 2);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Stage 1 IK Test...");

  // Init Hardware
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(50); // 50Hz for servos
  delay(200);

  // --- CALIBRATION ---
  // Adjust these if 90 degrees isn't perfectly straight on your robot
  // (HipOffset, KneeOffset, AnkleOffset)
  leftLeg.setOffsets(0.0, 0.0, 0.0);

  // Move to Start Position safely
  Serial.println("Homing...");
  leftLeg.moveToSmooth(0, -15.0, 1000); 
  delay(1000);
}

void loop() {
  // --- SINGLE LEG GAIT TEST ---
  
  // 1. Lift
  leftLeg.moveToSmooth(-2.0, -12.0, 300);

  // 2. Swing Forward
  leftLeg.moveToSmooth(5.0, -12.0, 300);

  // 3. Place Down
  leftLeg.moveToSmooth(5.0, -15.0, 200);

  // 4. Stance (Drag Back - The Power Stroke)
  // This line MUST be perfectly horizontal for stable walking
  leftLeg.moveToSmooth(-2.0, -15.0, 600);

  delay(100);
}
