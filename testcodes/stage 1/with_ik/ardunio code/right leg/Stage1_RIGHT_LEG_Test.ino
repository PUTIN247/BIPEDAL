/*
  Stage1_RIGHT_LEG_Test.ino
  Gait Test for RIGHT LEG (Mirrored) with Logging
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h" 

// NOTE: On Arduino Uno, I2C is fixed to: SDA = A4, SCL = A5
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- RIGHT LEG CONFIGURATION ---
// Channels: Hip=4, Knee=5, Ankle=6
// The 'true' at the end tells the library this is the RIGHT leg
BipedLeg rightLeg(&pwm, 4, 5, 6, true); 

void setup() {
  Serial.begin(115200); 
  Serial.println("Initializing Stage 1 - RIGHT LEG Test...");

  Wire.begin(); 
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(200);

  // Calibration Offsets (Tune these if the leg isn't straight)
  rightLeg.setOffsets(0.0, 0.0, 0.0);

  // --- HOMING ---
  Serial.println("Homing...");
  rightLeg.moveToSmooth(0, -17.50, 1000); 
  delay(500);
  
  printStepInfo("HOME POSITION");
}

void loop() {
  // --- SINGLE LEG GAIT TEST (RIGHT LEG) ---
  
  // 1. Lift
  rightLeg.moveToSmooth(-2.0, -16.0, 300);
  printStepInfo("STEP 1: LIFT");

  // 2. Swing Forward
  rightLeg.moveToSmooth(5.0, -16.0, 300);
  printStepInfo("STEP 2: SWING FORWARD");

  // 3. Place Down
  rightLeg.moveToSmooth(5.0, -17.50, 200);
  printStepInfo("STEP 3: PLACE DOWN");

  // 4. Stance (Drag Back - The Power Stroke)
  rightLeg.moveToSmooth(-2.0, -17.50, 600);
  printStepInfo("STEP 4: STANCE");
  
  // Wait before next cycle
  delay(1000); 
}

// Helper function to print feedback
void printStepInfo(String stepName) {
  Serial.println(stepName);
  Serial.print("Hip Angle:   "); Serial.print(rightLeg.getHipAngle()); Serial.println("°");
  Serial.print("Knee Angle:  "); Serial.print(rightLeg.getKneeAngle()); Serial.println("°");
  Serial.print("Ankle Angle: "); Serial.print(rightLeg.getAnkleAngle()); Serial.println("°");
  Serial.println("-----------------------");
}
