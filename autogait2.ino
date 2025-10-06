/* autogait_improved.ino
   Modified for:
   - Servo mechanical "straight up" = 90 degrees.
   - Calibration offsets (set later by user).
   - Slower, smoother motions (SPEED_MULTIPLIER + finer interpolation).
   - Preserves original gait phases & safety constraints.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// I2C pins for ESP32 (may use default I2C if preferred)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int PWM_FREQ = 50;      // 50Hz for hobby servos
const int SERVOMIN = 150;     // Min pulse count (tuned to your servos)
const int SERVOMAX = 600;     // Max pulse count

// PCA9685 channels (set to your wiring)
const int HIP_CH   = 0;
const int KNEE_CH  = 1;
const int ANKLE_CH = 2;

// Physical joint angle limits (degrees) - keep these as in your original code
const float HIP_MIN   = -45.0;
const float HIP_MAX   = 45.0;
const float KNEE_MIN  = 0.0;
const float KNEE_MAX  = 90.0;
const float ANKLE_MIN = -30.0;
const float ANKLE_MAX = 30.0;

// IMPORTANT: Mechanical neutral: when all three servos read 90°, the leg is straight up (vertical).
// So we will treat servo-angle 90° as the mechanical straight position (body rectangle's breadth horizontal).

// --- HOME / CALIBRATION ---
// You will calibrate true home later. For now, assume base home positions that correspond to "straight leg up"
// (these are servo-reading angles — you will add per-servo calibration offsets below).
// Use 90 for the straight-up reading, then apply offsets to find the real home.
float hipHomeBase   = 90.0; // means servo at 90 -> hip joint vertical
float kneeHomeBase  = 90.0; // servo 90 -> knee straight (aligned)
float ankleHomeBase = 90.0; // servo 90 -> ankle neutral (straight)

/* Calibration offsets (deg)
   After you measure real required outputs, set these to tune each servo's home.
   For example: hipCalibOffset = -5 means the actual hip home is (hipHomeBase + (-5)) = 85 deg.
*/
float hipCalibOffset   = 0.0;
float kneeCalibOffset  = 0.0;
float ankleCalibOffset = 0.0;

// Combined / effective home angles used by the gait
float hipHome;
float kneeHome;
float ankleHome;

// --- Timing & smoothness control ---
// Increase SPEED_MULTIPLIER to slow down the motions (e.g. 2.0 => twice as slow).
// Set to 1.0 for original speed, <1 faster (NOT recommended if servos are already too fast)
const float SPEED_MULTIPLIER = 2.5; // recommended: 2.0..4.0 for visible slow motions

// Interpolation steps (higher -> smoother but takes more time); combine with SPEED_MULTIPLIER
const int BASE_STEPS = 80;     // original-ish; increase for smoother motion
const int MIN_STEPS  = 20;

// Base phase durations (ms) — these will be multiplied by SPEED_MULTIPLIER
const int LIFT_TIME_MS    = 220;  // phase 1
const int SWING_TIME_MS   = 240;  // phase 2
const int PLACE_TIME_MS   = 200;  // phase 3
const int RECOVER_TIME_MS = 200;  // phase 4
const int CYCLE_DELAY_MS  = 120;  // pause between cycles

// Function forward declarations
void setServoPulse(int n, double pulse);
void setServoAngle(int channel, float angleDeg);
float angleConstrain(float a, float lo, float hi);
void moveLegSmooth(float hipFrom, float hipTo, float kneeFrom, float kneeTo, float ankleFrom, float ankleTo, int duration_ms);
void stepCycleSingleLeg();

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(200);

  // Compute effective home positions (calibrated)
  // Note: hipHomeBase etc. are servo-reading angles (0-180). We keep them that way.
  hipHome   = hipHomeBase   + hipCalibOffset;
  kneeHome  = kneeHomeBase  + kneeCalibOffset;
  ankleHome = ankleHomeBase + ankleCalibOffset;

  // Constrain to physical limits (converted relative to mechanical neutral: 90 as straight)
  // We store and use angles as "servo-angle" (0..180), but limits were in joint-centered degrees.
  // Convert joint-limits to servo-angle space by adding 90 (since 90 is straight/up).
  // Example: HIP_MIN = -45 deg -> servo-angle min = 90 + (-45) = 45 deg.
  hipHome   = constrain(hipHome, 90.0 + HIP_MIN, 90.0 + HIP_MAX);
  kneeHome  = constrain(kneeHome, 90.0 + KNEE_MIN, 90.0 + KNEE_MAX);
  ankleHome = constrain(ankleHome, 90.0 + ANKLE_MIN, 90.0 + ANKLE_MAX);

  Serial.println("Starting autogait_improved");
  Serial.print("Effective homes (servo angles): ");
  Serial.print("Hip: "); Serial.print(hipHome);
  Serial.print(" Knee: "); Serial.print(kneeHome);
  Serial.print(" Ankle: "); Serial.println(ankleHome);

  // Move to home gently on startup
  setServoAngle(HIP_CH, hipHome);
  setServoAngle(KNEE_CH, kneeHome);
  setServoAngle(ANKLE_CH, ankleHome);
  delay(800);
}

void loop() {
  stepCycleSingleLeg();
}

// --- Helper: map degrees (0..180) to PWM pulse counts and write to PCA9685 ---
void setServoAngle(int channel, float angleDeg) {
  // Constrain to [0..180]
  float a = constrain(angleDeg, 0.0, 180.0);

  // Map to pulse width count
  // Linear map: angle 0 => SERVOMIN, angle 180 => SERVOMAX
  long pulse = map((int)a, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

// Convenience: ensure angle within given joint limits (joint-space)
float angleConstrain(float servoAngle, float jointLo, float jointHi) {
  // servoAngle is expressed as servo-reading angle (0..180)
  // jointLo/jointHi are in joint-centered degrees (e.g., KNEE_MIN = 0). Convert and constrain.
  float lo = 90.0 + jointLo;
  float hi = 90.0 + jointHi;
  return constrain(servoAngle, lo, hi);
}

// Smooth multi-joint move using linear interpolation
void moveLegSmooth(float hipFrom, float hipTo, float kneeFrom, float kneeTo, float ankleFrom, float ankleTo, int duration_ms) {
  // Scale duration by SPEED_MULTIPLIER
  float durationScaled = duration_ms * SPEED_MULTIPLIER;

  // Compute steps (higher steps for smoother motion). Keep a minimum.
  int steps = max(MIN_STEPS, BASE_STEPS);
  // If user slows down a lot, increase steps proportionally for smoothness
  steps = int(steps * SPEED_MULTIPLIER);
  steps = max(steps, MIN_STEPS);

  // Per-step increments
  float hipStep   = (hipTo   - hipFrom)   / float(steps);
  float kneeStep  = (kneeTo  - kneeFrom)  / float(steps);
  float ankleStep = (ankleTo - ankleFrom) / float(steps);

  // Step delay (ms)
  float stepDelay = durationScaled / float(steps);

  float hip = hipFrom;
  float knee = kneeFrom;
  float ankle = ankleFrom;

  for (int i = 0; i <= steps; ++i) {
    // Constrain each joint to physical limits (servo-angle space)
    float hipCon   = angleConstrain(hip, HIP_MIN, HIP_MAX);
    float kneeCon  = angleConstrain(knee, KNEE_MIN, KNEE_MAX);
    float ankleCon = angleConstrain(ankle, ANKLE_MIN, ANKLE_MAX);

    setServoAngle(HIP_CH, hipCon);
    setServoAngle(KNEE_CH, kneeCon);
    setServoAngle(ANKLE_CH, ankleCon);

    hip   += hipStep;
    knee  += kneeStep;
    ankle += ankleStep;

    delay((int)stepDelay);
  }
}

// Single-leg step cycle: same sequence as original but using the improved timing & home handling
void stepCycleSingleLeg() {
  // Effective servo-angle home values already computed in setup()
  float hip0   = hipHome;
  float knee0  = kneeHome;
  float ankle0 = ankleHome;

  // Phase 1: Lift leg (bend knee, raise ankle slightly)
  float ankleLift = max(90.0 + ANKLE_MIN, ankle0 - 18.0);   // raise (servo-angle decrease = upward tilt)
  float kneeLift  = min(90.0 + KNEE_MAX, knee0 + 35.0);    // bend knee (servo-angle increases)
  float hipPrep   = hip0;                                 // hip neutral during lift
  moveLegSmooth(hip0, hipPrep, knee0, kneeLift, ankle0, ankleLift, LIFT_TIME_MS);

  // Phase 2: Swing leg forward (hip forward by ~25 deg in joint-space)
  float hipForward = constrain(hip0 + 25.0, 90.0 + HIP_MIN, 90.0 + HIP_MAX);
  moveLegSmooth(hipPrep, hipForward, kneeLift, kneeLift, ankleLift, ankleLift, SWING_TIME_MS);

  // Phase 3: Place leg down (straighten knee & ankle)
  moveLegSmooth(hipForward, hipForward, kneeLift, knee0, ankleLift, ankle0, PLACE_TIME_MS);

  // Phase 4: Recover to home position
  moveLegSmooth(hipForward, hip0, knee0, knee0, ankle0, ankle0, RECOVER_TIME_MS);

  delay((int)(CYCLE_DELAY_MS * SPEED_MULTIPLIER));
}
