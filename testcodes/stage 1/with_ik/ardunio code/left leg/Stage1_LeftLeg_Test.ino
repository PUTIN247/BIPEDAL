/*
  Stage1_LeftLeg_Test.ino
  Specific code for testing the Left Leg (Mirrored)
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h" 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- CONFIGURATION ---
// Channels: Hip=4, Knee=5, Ankle=6 (CHANGE IF NEEDED)
// The 'true' at the end enables LEFT LEG MIRROR MODE
BipedLeg leftLeg(&pwm, 4, 5, 6, true);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Stage 1: Left Leg Test...");

  Wire.begin(); 
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(200);

  // Apply Offsets (Start with 0, adjust after first run)
  // If the leg isn't straight at 90 deg, add values here: (Hip, Knee, Ankle)
  leftLeg.setOffsets(0.0, 0.0, 0.0);

  // --- HOMING ---
  Serial.println("Homing...");
  // Move to safe stance: X=0 (Centered), Y=-15cm (Height)
  leftLeg.moveToSmooth(0, -15.0, 1000); 
  delay(1000);
  
  printStepInfo("HOME POSITION");
}

void loop() {
  // --- SINGLE LEG GAIT TEST (LEFT) ---
  // The coordinates are the same as Right Leg, 
  // but the Library handles the servo mirroring automatically.

  // 1. Lift
  leftLeg.moveToSmooth(-4.0, -13.0, 300);
  printStepInfo("STEP 1: LIFT");

  // 2. Swing Forward
  leftLeg.moveToSmooth(3.0, -13.0, 300);
  printStepInfo("STEP 2: SWING FORWARD");

  // 3. Place Down
  leftLeg.moveToSmooth(3.0, -15.0, 200);
  printStepInfo("STEP 3: PLACE DOWN");

  // 4. Stance (Drag Back - Power Stroke)
  leftLeg.moveToSmooth(-4.0, -15.0, 600);
  printStepInfo("STEP 4: STANCE");

  delay(1000); 
}

// --- LOGGING HELPER ---
void printStepInfo(String stepName) {
  Serial.println(stepName);
  Serial.print("Hip Angle:   "); Serial.print(leftLeg.getHipAngle()); Serial.println("°");
  Serial.print("Knee Angle:  "); Serial.print(leftLeg.getKneeAngle()); Serial.println("°");
  Serial.print("Ankle Angle: "); Serial.print(leftLeg.getAnkleAngle()); Serial.println("°");
  Serial.println("-----------------------");
}
