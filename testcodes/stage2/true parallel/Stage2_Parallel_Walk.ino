#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- HARDWARE CONFIG ---
// Left Leg
#define L_HIP 0
#define L_KNEE 1
#define L_ANKLE 2
// Right Leg
#define R_HIP 4
#define R_KNEE 5
#define R_ANKLE 6

// --- GAIT SETTINGS ---
// Walking Parameters
#define STAND_HEIGHT -15.0  // cm (Running height)
#define STEP_HEIGHT   4.0   // cm (Higher clearance for long feet)
#define STRIDE_LEN    5.0   // cm (Total step length)
#define STEP_DURATION 800   // ms (Slower = Smoother/Safer)

// Objects
BipedLeg leftLeg(&pwm, L_HIP, L_KNEE, L_ANKLE, false);
BipedLeg rightLeg(&pwm, R_HIP, R_KNEE, R_ANKLE, true);

// State Machine
enum GaitState {
  STATE_STAND,
  STATE_STEP1_R_LIFT,  // Right Leg Lifts & Moves Forward
  STATE_STEP1_R_PLACE, // Right Leg Lowers
  STATE_STEP2_L_LIFT,  // Left Leg Lifts & Moves Forward
  STATE_STEP2_L_PLACE  // Left Leg Lowers
};

GaitState currentState = STATE_STAND;
unsigned long stateStartTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println(">>> Stage 2: Parallel Motion Init <<<");

  pwm.begin();
  pwm.setPWMFreq(50);

  // --- 1. CALIBRATION (INSERT YOUR OFFSETS HERE) ---
  // leftLeg.setOffsets(hip, knee, ankle);
  leftLeg.setOffsets(0, 0, 0); 
  rightLeg.setOffsets(0, 0, 0); 

  // --- 2. INITIAL POSE ---
  // Move smoothly to Home Position
  leftLeg.setTarget(0, STAND_HEIGHT, 2000);
  rightLeg.setTarget(0, STAND_HEIGHT, 2000);
  
  stateStartTime = millis();
}

void loop() {
  // --- A. UPDATE SERVOS (Must call this FAST and OFTEN) ---
  // This is what makes the motion "Parallel"
  bool leftDone = leftLeg.update();
  bool rightDone = rightLeg.update();

  // --- B. GAIT LOGIC (The "Brain") ---
  // We only switch states when both legs have finished their current move
  if (leftDone && rightDone) {
    nextGaitState();
  }
}

void nextGaitState() {
  switch (currentState) {
    
    case STATE_STAND:
      // Start Walking sequence
      Serial.println("Start Walking...");
      currentState = STATE_STEP1_R_LIFT;
      
      // TRIGGER MOTION:
      // Right Leg: Swing Forward (Lifted)
      // Left Leg:  Push Backward (On Ground) - simultaneous!
      rightLeg.setTarget(STRIDE_LEN/2, STAND_HEIGHT + STEP_HEIGHT, STEP_DURATION);
      leftLeg.setTarget(-STRIDE_LEN/2, STAND_HEIGHT, STEP_DURATION);
      break;

    case STATE_STEP1_R_LIFT:
      // Right leg is in air, forward. Now put it down.
      // Left leg continues pushing back slightly or holds (simplified)
      currentState = STATE_STEP1_R_PLACE;
      
      rightLeg.setTarget(STRIDE_LEN/2, STAND_HEIGHT, STEP_DURATION/2); // Quick place
      leftLeg.setTarget(-STRIDE_LEN/2, STAND_HEIGHT, STEP_DURATION/2); // Keep supporting
      break;

    case STATE_STEP1_R_PLACE:
      // Right is now planted forward. Left is back.
      // TIME TO SWITCH: Left Swing Forward, Right Push Backward
      currentState = STATE_STEP2_L_LIFT;
      
      leftLeg.setTarget(STRIDE_LEN/2, STAND_HEIGHT + STEP_HEIGHT, STEP_DURATION);
      rightLeg.setTarget(-STRIDE_LEN/2, STAND_HEIGHT, STEP_DURATION);
      break;

    case STATE_STEP2_L_LIFT:
      // Left leg is in air, forward. Now put it down.
      currentState = STATE_STEP2_L_PLACE;
      
      leftLeg.setTarget(STRIDE_LEN/2, STAND_HEIGHT, STEP_DURATION/2); // Quick place
      rightLeg.setTarget(-STRIDE_LEN/2, STAND_HEIGHT, STEP_DURATION/2); // Keep supporting
      break;

    case STATE_STEP2_L_PLACE:
      // Cycle Complete. Loop back to start Right Swing again.
      currentState = STATE_STEP1_R_LIFT;
      
      // Note: Coordinates are relative to Hip. 
      // To "Push", we move the foot backward relative to hip.
      rightLeg.setTarget(STRIDE_LEN/2, STAND_HEIGHT + STEP_HEIGHT, STEP_DURATION);
      leftLeg.setTarget(-STRIDE_LEN/2, STAND_HEIGHT, STEP_DURATION);
      break;
  }
}