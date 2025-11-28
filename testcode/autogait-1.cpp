#include <Wire.h>                        // Library for I2C communication
#include <Adafruit_PWMServoDriver.h>     // Library for PCA9685 servo driver

// Define I2C pins for ESP32 (default I2C can also work, but we fix pins here)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Create a PCA9685 object (default address = 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo driver settings
const int PWM_FREQ = 50;      // Servo frequency = 50 Hz (standard for hobby servos)
const int SERVOMIN = 150;     // Minimum pulse length count (corresponds to 0°)
const int SERVOMAX = 600;     // Maximum pulse length count (corresponds to 180°)

// Assign PCA9685 channels for each servo
const uint8_t HIP_CH   = 0;   // Servo connected to channel 0
const uint8_t KNEE_CH  = 1;   // Servo connected to channel 1
const uint8_t ANKLE_CH = 2;   // Servo connected to channel 2

// Define "Home" or neutral position (all servos at 0°)
float hipHome   = 0.0;
float kneeHome  = 0.0;
float ankleHome = 90.0; // ankle servo at 90 degree angle

// Joint movement limits (safety constraints)
const float HIP_MIN   = 0.0;
const float HIP_MAX   = 180.0;
const float KNEE_MIN  = 0.0;
const float KNEE_MAX  = 90.0;     // Knee cannot bend beyond 90° (physical constraint)
const float ANKLE_MIN = 0.0;
const float ANKLE_MAX = 180.0;

// Default smooth movement steps (higher = smoother but slower)
const int DEFAULT_STEPS = 20;  //devide angle movement from one to another into number of steps 

// Durations for each phase of gait (in milliseconds)
const int LIFT_TIME_MS    = 350;   // Lift leg
const int SWING_TIME_MS   = 500;   // Move leg forward
const int PLACE_TIME_MS   = 300;   // Put leg down
const int RECOVER_TIME_MS = 300;   // Return to home
const int CYCLE_DELAY_MS  = 150;   // Small pause between steps

// Convert angle (0°–180°) to PCA9685 pulse length
int angleToPulseCounts(float angle_deg) {
  if (angle_deg < 0) angle_deg = 0;       // Clamp to valid range
  if (angle_deg > 180) angle_deg = 180;
  return map((int)angle_deg, 0, 180, SERVOMIN, SERVOMAX); // Map angle to pulse range
}

// Function to set servo angle safely with constraints
void setServoAngle(uint8_t channel, float angle_deg) {
  // Apply constraints depending on joint
  if (channel == KNEE_CH) {
    angle_deg = constrain(angle_deg, KNEE_MIN, KNEE_MAX);
  } else if (channel == HIP_CH) {
    angle_deg = constrain(angle_deg, HIP_MIN, HIP_MAX);
  } else if (channel == ANKLE_CH) {
    angle_deg = constrain(angle_deg, ANKLE_MIN, ANKLE_MAX);
  }

  // Convert angle to PWM pulse and send to servo
  int pulse = angleToPulseCounts(angle_deg);
  pwm.setPWM(channel, 0, pulse);  //tells the pca that to which angle it should go high and low.. low at pulse (end angle )
}

// Smooth transition of joints from one pose to another
// This interpolates angles gradually in "steps" for natural motion
void moveLegSmooth(float hipFrom, float hipTo, 
                   float kneeFrom, float kneeTo,
                   float ankleFrom, float ankleTo, 
                   int duration_ms, int steps = DEFAULT_STEPS) {
  if (steps <= 0) steps = DEFAULT_STEPS;   // Safety: prevent zero-step motion
  for (int i = 1; i <= steps; ++i) {
    float t = (float)i / (float)steps;     // Interpolation factor (0 → 1)
    
    // Linear interpolation between start and end positions
    float hipAngle   = hipFrom + (hipTo - hipFrom) * t;
    float kneeAngle  = kneeFrom + (kneeTo - kneeFrom) * t;
    float ankleAngle = ankleFrom + (ankleTo - ankleFrom) * t;

    // Apply interpolated angles
    setServoAngle(HIP_CH, hipAngle);
    setServoAngle(KNEE_CH, kneeAngle);
    setServoAngle(ANKLE_CH, ankleAngle);

    delay(duration_ms / steps); // Small delay for smoothness
  }
}

// Define one walking cycle for a single leg
void stepCycleSingleLeg() {
  // Start from home position
  float hip0   = hipHome;
  float knee0  = kneeHome;
  float ankle0 = ankleHome;

  // Phase 1: Lift leg (bend knee, raise ankle slightly)
  float ankleLift  = max(ANKLE_MIN, ankle0 - 18.0);   // Raise ankle (negative = upward tilt)
  float kneeLift   = min(KNEE_MAX, knee0 + 35.0);     // Bend knee up to 35° (within 90° limit)
  float hipPrep    = hip0;                            // Hip stays neutral for lift
  moveLegSmooth(hip0, hipPrep, knee0, kneeLift, ankle0, ankleLift, LIFT_TIME_MS);

  // Phase 2: Swing leg forward
  float hipForward = constrain(hip0 + 25.0, HIP_MIN, HIP_MAX); // Move hip forward by 25°
  moveLegSmooth(hipPrep, hipForward, kneeLift, kneeLift, ankleLift, ankleLift, SWING_TIME_MS);

  // Phase 3: Place leg down (straighten knee & ankle)
  moveLegSmooth(hipForward, hipForward, kneeLift, knee0, ankleLift, ankle0, PLACE_TIME_MS);

  // Phase 4: Recover to home position
  moveLegSmooth(hipForward, hip0, knee0, knee0, ankle0, ankle0, RECOVER_TIME_MS);

  delay(CYCLE_DELAY_MS); // Small pause before repeating
}

void setup() {
  // Initialize I2C for PCA9685
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(200);

  // Move all joints to home position on startup
  setServoAngle(HIP_CH, hipHome);
  setServoAngle(KNEE_CH, kneeHome);
  setServoAngle(ANKLE_CH, ankleHome);
  delay(800); // Wait to stabilize before starting gait
}

void loop() {
  // Repeat single leg step cycle continuously
  stepCycleSingleLeg();
}
