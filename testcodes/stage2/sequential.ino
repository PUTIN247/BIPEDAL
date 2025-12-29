#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- CONFIGURATION ---
// Left Leg Channels (From Stage 1)
#define L_HIP 0
#define L_KNEE 1
#define L_ANKLE 2

// Right Leg Channels (NEW - Please verify connections)
#define R_HIP 4
#define R_KNEE 5
#define R_ANKLE 6

// Gait Parameters
#define STEP_HEIGHT 2.0   // cm (Keep low for stability)
#define STRIDE_LEN  5.0   // cm (Forward movement)
#define STAND_H    -15.0  // cm (Standing Height)
#define SPEED_MS    800   // Duration of one step (Slower = Safer)

// Instantiate Both Legs
BipedLeg leftLeg(&pwm, L_HIP, L_KNEE, L_ANKLE);
BipedLeg rightLeg(&pwm, R_HIP, R_KNEE, R_ANKLE);

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);

  // 1. Calibration (Apply your offsets here!)
  // Format: (Hip, Knee, Ankle)
  leftLeg.setOffsets(0, 0, 0); 
  rightLeg.setOffsets(0, 0, 0); 

  // 2. Start Pose: Stand Tall
  Serial.println("Assuming Start Pose...");
  // Both feet directly under hips
  leftLeg.moveToSmooth(0, STAND_H, 1000);
  rightLeg.moveToSmooth(0, STAND_H, 1000);
  delay(2000); // Wait for user to stabilize robot
}

void loop() {
  Serial.println(">>> STEP 1: Right Leg Forward, Left Leg Push <<<");
  // A. Shift Weight / Prep (Left moves back slightly to push body)
  leftLeg.moveToSmooth(-STRIDE_LEN, STAND_H, SPEED_MS);
  
  // B. Swing Right Leg
  // Lift
  rightLeg.moveToSmooth(0, STAND_H + STEP_HEIGHT, SPEED_MS/2);
  // Forward & Down
  rightLeg.moveToSmooth(STRIDE_LEN, STAND_H, SPEED_MS/2);
  
  delay(100);

  Serial.println(">>> STEP 2: Left Leg Forward, Right Leg Push <<<");
  
  // A. Push Body with Right Leg (Move Right Leg back relative to hip)
  rightLeg.moveToSmooth(-STRIDE_LEN, STAND_H, SPEED_MS);

  // B. Swing Left Leg
  // Lift (From its back position)
  leftLeg.moveToSmooth(-STRIDE_LEN, STAND_H + STEP_HEIGHT, SPEED_MS/2); // Lift in place
  // Swing Forward & Down
  leftLeg.moveToSmooth(STRIDE_LEN, STAND_H, SPEED_MS/2);

  delay(100);
}
