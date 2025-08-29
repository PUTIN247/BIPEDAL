#include <Servo.h>
#include <math.h>

// =============================================================================
// USER-CONFIGURABLE PARAMETERS (Adjust these based on physical testing)
// =============================================================================

// Physical robot constants
#define LINK_1_LENGTH 9.5f  // Thigh length in cm
#define LINK_2_LENGTH 9.5f  // Shank length in cm

// Servo pins
#define HIP_SERVO_PIN 9
#define KNEE_SERVO_PIN 10
#define ANKLE_SERVO_PIN 11

// Servo calibration (Adjust for your specific servos and mounting)
#define HIP_OFFSET 0.0f     // Degrees offset for hip servo
#define KNEE_OFFSET 0.0f    // Degrees offset for knee servo
#define ANKLE_OFFSET 0.0f   // Degrees offset for ankle servo

// Standard standing pose (θ_home from calculations)
#define HOME_HIP_ANGLE -37.9f
#define HOME_KNEE_ANGLE -104.2f
#define HOME_ANKLE_ANGLE 142.1f

// Joint limits (Prevent damage from over-rotation)
#define HIP_MIN_ANGLE -90.0f
#define HIP_MAX_ANGLE 90.0f
#define KNEE_MIN_ANGLE -130.0f
#define KNEE_MAX_ANGLE 0.0f
#define ANKLE_MIN_ANGLE 90.0f
#define ANKLE_MAX_ANGLE 180.0f

// Movement parameters (Adjust for speed/stability tradeoff)
#define HOMING_SPEED 10     // Degrees per second for homing
#define MOVEMENT_SPEED 60   // Degrees per second for normal operation
#define STEP_HEIGHT 3.0f    // Max foot lift height in cm
#define STEP_LENGTH 8.0f    // Step distance in cm
#define LIFT_DURATION 500   // ms to lift foot
#define SWING_DURATION 1000 // ms for forward swing
#define PLANT_DURATION 500  // ms to plant foot

// Homing parameters
#define HOMING_DURATION 2000 // ms to spend homing each joint
#define HOMING_DIRECTION_HIP -1 // Direction to move for homing (-1 or 1)
#define HOMING_DIRECTION_KNEE -1
#define HOMING_DIRECTION_ANKLE -1

// =============================================================================
// SYSTEM COMPONENTS
// =============================================================================

Servo hipServo;
Servo kneeServo;
Servo ankleServo;

// Current joint angles
float currentHipAngle = 0;
float currentKneeAngle = 0;
float currentAnkleAngle = 0;

// Current foot position
float currentX = 0;
float currentY = 0;

// State machine variables
enum State { 
  STATE_HOMING, 
  STATE_MOVE_TO_HOME, 
  STATE_STANDING, 
  STATE_LIFT_FOOT, 
  STATE_SWING, 
  STATE_PLANT_FOOT,
  STATE_SHIFT_COM
};

State currentState = STATE_HOMING;
unsigned long stateStartTime = 0;

// =============================================================================
// KINEMATICS MODULE
// =============================================================================

/**
 * Calculate inverse kinematics for target foot position
 * Returns true if position is reachable, false otherwise
 */
bool calculateIK(float x, float y, float &hipAngle, float &kneeAngle, float &ankleAngle) {
  // Calculate distance from hip to target
  float D_sq = x * x + y * y;
  float D = sqrt(D_sq);
  
  // Check if target is reachable
  float max_reach = LINK_1_LENGTH + LINK_2_LENGTH;
  float min_reach = abs(LINK_1_LENGTH - LINK_2_LENGTH);
  
  if (D > max_reach || D < min_reach) {
    return false; // Position unreachable
  }
  
  // Calculate knee angle using Law of Cosines
  float cos_theta2 = (D_sq - LINK_1_LENGTH*LINK_1_LENGTH - LINK_2_LENGTH*LINK_2_LENGTH) / 
                     (2 * LINK_1_LENGTH * LINK_2_LENGTH);
  
  // Clamp to valid range for acos
  if (cos_theta2 > 1.0) cos_theta2 = 1.0;
  if (cos_theta2 < -1.0) cos_theta2 = -1.0;
  
  // Knee-forward solution (digitigrade shape)
  float theta2_rad = -acos(cos_theta2);
  
  // Calculate hip angle
  float gamma_rad = atan2(x, y);
  float L1_plus_L2cos = LINK_1_LENGTH + LINK_2_LENGTH * cos(theta2_rad);
  float L2_sin = LINK_2_LENGTH * sin(theta2_rad);
  float beta_rad = atan2(L2_sin, L1_plus_L2cos);
  
  float theta1_rad = gamma_rad - beta_rad;
  
  // Calculate ankle angle (keep foot flat)
  float theta3_rad = -(theta1_rad + theta2_rad);
  
  // Convert to degrees
  hipAngle = theta1_rad * 180.0 / M_PI;
  kneeAngle = theta2_rad * 180.0 / M_PI;
  ankleAngle = theta3_rad * 180.0 / M_PI;
  
  return true;
}

/**
 * Calculate forward kinematics from joint angles
 */
void calculateFK(float hipAngle, float kneeAngle, float ankleAngle, float &x, float &y) {
  // Convert to radians
  float theta1_rad = hipAngle * M_PI / 180.0;
  float theta2_rad = kneeAngle * M_PI / 180.0;
  
  // Calculate foot position
  x = LINK_1_LENGTH * sin(theta1_rad) + LINK_2_LENGTH * sin(theta1_rad + theta2_rad);
  y = LINK_1_LENGTH * cos(theta1_rad) + LINK_2_LENGTH * cos(theta1_rad + theta2_rad);
}

// =============================================================================
// SERVO CONTROL MODULE
// =============================================================================

/**
 * Set servo angles with safety limits
 */
void setServoAngles(float hipAngle, float kneeAngle, float ankleAngle) {
  // Apply offsets and clamp to limits
  float hipOutput = constrain(hipAngle + HIP_OFFSET, HIP_MIN_ANGLE, HIP_MAX_ANGLE);
  float kneeOutput = constrain(kneeAngle + KNEE_OFFSET, KNEE_MIN_ANGLE, KNEE_MAX_ANGLE);
  float ankleOutput = constrain(ankleAngle + ANKLE_OFFSET, ANKLE_MIN_ANGLE, ANKLE_MAX_ANGLE);
  
  // Command servos
  hipServo.write(hipOutput);
  kneeServo.write(kneeOutput);
  ankleServo.write(ankleOutput);
  
  // Update current angles
  currentHipAngle = hipAngle;
  currentKneeAngle = kneeAngle;
  currentAnkleAngle = ankleAngle;
  
  // Update current foot position
  calculateFK(hipAngle, kneeAngle, ankleAngle, currentX, currentY);
}

/**
 * Move servos smoothly to target angles over duration
 */
void moveServosSmoothly(float targetHip, float targetKnee, float targetAnkle, unsigned long duration) {
  unsigned long startTime = millis();
  float startHip = currentHipAngle;
  float startKnee = currentKneeAngle;
  float startAnkle = currentAnkleAngle;
  
  while (millis() - startTime < duration) {
    float progress = (float)(millis() - startTime) / duration;
    
    // Linear interpolation
    float hipAngle = startHip + (targetHip - startHip) * progress;
    float kneeAngle = startKnee + (targetKnee - startKnee) * progress;
    float ankleAngle = startAnkle + (targetAnkle - startAnkle) * progress;
    
    setServoAngles(hipAngle, kneeAngle, ankleAngle);
    delay(20); // Control update rate
  }
  
  // Ensure final position is exactly the target
  setServoAngles(targetHip, targetKnee, targetAnkle);
}

// =============================================================================
// HOMING MODULE
// =============================================================================

/**
 * Home servos by moving toward mechanical stops
 */
void homeServos() {
  // Move all servos in homing direction
  hipServo.write(constrain(90 + HOMING_DIRECTION_HIP * 90, 0, 180));
  kneeServo.write(constrain(90 + HOMING_DIRECTION_KNEE * 90, 0, 180));
  ankleServo.write(constrain(90 + HOMING_DIRECTION_ANKLE * 90, 0, 180));
  
  // Wait for homing duration
  delay(HOMING_DURATION);
  
  // Stop servos (assuming they've hit mechanical stops)
  hipServo.write(90);
  kneeServo.write(90);
  ankleServo.write(90);
  
  // Set current angles to unknown (will be calibrated when moving to home)
  currentHipAngle = 0;
  currentKneeAngle = 0;
  currentAnkleAngle = 0;
}

// =============================================================================
// GAIT GENERATION MODULE
// =============================================================================

/**
 * Execute a complete walking step
 */
void executeStep() {
  // 1. Lift foot
  float liftX = currentX;
  float liftY = currentY - STEP_HEIGHT;
  
  float hipLift, kneeLift, ankleLift;
  if (calculateIK(liftX, liftY, hipLift, kneeLift, ankleLift)) {
    moveServosSmoothly(hipLift, kneeLift, ankleLift, LIFT_DURATION);
  }
  
  // 2. Swing forward
  float swingX = currentX + STEP_LENGTH;
  float swingY = currentY;
  
  float hipSwing, kneeSwing, ankleSwing;
  if (calculateIK(swingX, swingY, hipSwing, kneeSwing, ankleSwing)) {
    moveServosSmoothly(hipSwing, kneeSwing, ankleSwing, SWING_DURATION);
  }
  
  // 3. Plant foot
  float plantX = currentX;
  float plantY = currentY + STEP_HEIGHT; // Return to ground level
  
  float hipPlant, kneePlant, anklePlant;
  if (calculateIK(plantX, plantY, hipPlant, kneePlant, anklePlant)) {
    moveServosSmoothly(hipPlant, kneePlant, anklePlant, PLANT_DURATION);
  }
}

// =============================================================================
// MAIN ARDUINO FUNCTIONS
// =============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Bipedal Leg Controller Initializing...");
  
  // Attach servos
  hipServo.attach(HIP_SERVO_PIN);
  kneeServo.attach(KNEE_SERVO_PIN);
  ankleServo.attach(ANKLE_SERVO_PIN);
  
  // Start homing procedure
  currentState = STATE_HOMING;
  stateStartTime = millis();
  
  Serial.println("Setup complete. Starting homing sequence.");
}

void loop() {
  switch (currentState) {
    case STATE_HOMING:
      Serial.println("Homing servos...");
      homeServos();
      currentState = STATE_MOVE_TO_HOME;
      stateStartTime = millis();
      break;
      
    case STATE_MOVE_TO_HOME:
      Serial.println("Moving to home position...");
      setServoAngles(HOME_HIP_ANGLE, HOME_KNEE_ANGLE, HOME_ANKLE_ANGLE);
      currentState = STATE_STANDING;
      stateStartTime = millis();
      Serial.println("In home position. Ready for movement.");
      break;
      
    case STATE_STANDING:
      // Wait for standing, then initiate step
      if (millis() - stateStartTime > 2000) { // Wait 2 seconds
        currentState = STATE_LIFT_FOOT;
        stateStartTime = millis();
      }
      break;
      
    case STATE_LIFT_FOOT:
      Serial.println("Lifting foot...");
      // Lift foot implementation would go here
      currentState = STATE_SWING;
      stateStartTime = millis();
      break;
      
    case STATE_SWING:
      Serial.println("Swinging leg...");
      executeStep();
      currentState = STATE_STANDING;
      stateStartTime = millis();
      break;
      
    default:
      currentState = STATE_STANDING;
      break;
  }
  
  // Add small delay to prevent overwhelming the servo control
  delay(10);
}
