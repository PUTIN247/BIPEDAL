/*
  Stage1_IK_Test.ino (Arduino Uno Version)
  Main controller for Single Leg IK Testing
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h" // Include our new custom library

// NOTE: On Arduino Uno, I2C is fixed to:
// SDA = A4
// SCL = A5

// Servo Driver Object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- LEG CONFIGURATION ---
// Define channels: Hip=0, Knee=1, Ankle=2
BipedLeg leftLeg(&pwm, 0, 1, 2);

void setup() {
  Serial.begin(115200); // Make sure your Serial Monitor is set to 115200 baud
  Serial.println("Initializing Stage 1 IK Test (Uno)...");

  // Init Hardware
  Wire.begin(); // Arduino Uno uses default pins (A4/A5) automatically
  
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
