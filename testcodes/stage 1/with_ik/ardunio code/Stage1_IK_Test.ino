/*
  Stage1_IK_Test.ino (Arduino Uno Version)
  With Detailed Step Logging
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h" 

// NOTE: On Arduino Uno, I2C is fixed to: SDA = A4, SCL = A5
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define channels: Hip=0, Knee=1, Ankle=2
BipedLeg rightLeg(&pwm, 0, 1, 2);

void setup() {
  Serial.begin(115200); 
  Serial.println("Initializing Stage 1 IK Test (Uno)...");

  Wire.begin(); 
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(200);

  // Apply Offsets (Adjust if needed)
  rightLeg.setOffsets(0.0, 0.0, 0.0);

  // --- HOMING ---
  Serial.println("Homing...");
  rightLeg.moveToSmooth(-2, -15.0, 1000); 
  delay(500);
  
  // LOG HOME POSITION
  printStepInfo("HOME POSITION");
}

void loop() {
  // --- SINGLE LEG GAIT TEST ---

  // 1. Lift
  rightLeg.moveToSmooth(-4.0, -13.0, 300);
  printStepInfo("STEP 1: LIFT");

  // 2. Swing Forward
  rightLeg.moveToSmooth(3.0, -13.0, 300);
  printStepInfo("STEP 2: SWING FORWARD");

  // 3. Place Down
  rightLeg.moveToSmooth(3.0, -15.0, 200);
  printStepInfo("STEP 3: PLACE DOWN");

  // 4. Stance (Drag Back - The Power Stroke)
  rightLeg.moveToSmooth(-4.0, -15.0, 600);
  printStepInfo("STEP 4: STANCE");
  
  // Wait before next cycle
  delay(1000); 
}

// --- HELPER FUNCTION FOR LOGGING ---
void printStepInfo(String stepName) {
  Serial.println(stepName);
  Serial.print("Hip Servo Angle:   "); Serial.print(rightLeg.getHipAngle()); Serial.println("°");
  Serial.print("Knee Servo Angle:  "); Serial.print(rightLeg.getKneeAngle()); Serial.println("°");
  Serial.print("Ankle Servo Angle: "); Serial.print(rightLeg.getAnkleAngle()); Serial.println("°");
  Serial.println("-----------------------");
}
