/* autogait_ultrasmooth.ino
   Features:
   - 90° servo = mechanical vertical (straight leg)
   - Adjustable calibration offsets
   - Smooth cubic easing motion (fluid)
   - Time-based control (no jerks)
   - Same gait logic, limits, and speed behavior
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int PWM_FREQ = 50;
const int SERVOMIN = 150;
const int SERVOMAX = 600;

// PCA9685 channels (adjust to your wiring)
const int HIP_CH   = 0;
const int KNEE_CH  = 1;
const int ANKLE_CH = 2;

// Physical joint limits (degrees from straight)
const float HIP_MIN   = -45.0;
const float HIP_MAX   = 45.0;
const float KNEE_MIN  = 0.0;
const float KNEE_MAX  = 90.0;
const float ANKLE_MIN = -30.0;
const float ANKLE_MAX = 30.0;

// Home calibration setup
float hipHomeBase   = 90.0;
float kneeHomeBase  = 90.0;
float ankleHomeBase = 90.0;

float hipCalibOffset   = 0.0;
float kneeCalibOffset  = 0.0;
float ankleCalibOffset = 0.0;

float hipHome, kneeHome, ankleHome;

// Speed and smoothness settings
const float SPEED_MULTIPLIER = 2.5;
const int   SERVO_REFRESH_MS = 15;   // time between updates (smaller = smoother, 10–20 recommended)

// Timing per phase (base durations, scaled by SPEED_MULTIPLIER)
const int LIFT_TIME_MS    = 220;
const int SWING_TIME_MS   = 240;
const int PLACE_TIME_MS   = 200;
const int RECOVER_TIME_MS = 200;
const int CYCLE_DELAY_MS  = 120;

// --- Function declarations ---
void setServoAngle(int ch, float angle);
float angleConstrain(float servoAngle, float jointLo, float jointHi);
void moveLegSmoothCubic(float h1, float h2, float k1, float k2, float a1, float a2, int duration);
void stepCycleSingleLeg();
float easeInOutCubic(float t);

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(200);

  hipHome   = hipHomeBase   + hipCalibOffset;
  kneeHome  = kneeHomeBase  + kneeCalibOffset;
  ankleHome = ankleHomeBase + ankleCalibOffset;

  hipHome   = constrain(hipHome, 90.0 + HIP_MIN, 90.0 + HIP_MAX);
  kneeHome  = constrain(kneeHome, 90.0 + KNEE_MIN, 90.0 + KNEE_MAX);
  ankleHome = constrain(ankleHome, 90.0 + ANKLE_MIN, 90.0 + ANKLE_MAX);

  Serial.println("Starting autogait_ultrasmooth");
  Serial.print("Home positions: ");
  Serial.print("H="); Serial.print(hipHome);
  Serial.print(" K="); Serial.print(kneeHome);
  Serial.print(" A="); Serial.println(ankleHome);

  // move to home slowly
  moveLegSmoothCubic(hipHome, hipHome, kneeHome, kneeHome, ankleHome, ankleHome, 600);
}

void loop() {
  stepCycleSingleLeg();
}

// ---- Servo and angle utilities ----
void setServoAngle(int ch, float angle) {
  angle = constrain(angle, 0, 180);
  long pulse = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(ch, 0, pulse);
}

float angleConstrain(float servoAngle, float jointLo, float jointHi) {
  float lo = 90.0 + jointLo;
  float hi = 90.0 + jointHi;
  return constrain(servoAngle, lo, hi);
}

// ---- Cubic easing function ----
//  t in [0,1]  ->  returns eased fraction [0,1]
float easeInOutCubic(float t) {
  if (t < 0.5) return 4 * t * t * t;
  t = 1 - t;
  return 1 - 4 * t * t * t;
}

// ---- Motion function (ultra smooth) ----
void moveLegSmoothCubic(float h1, float h2, float k1, float k2, float a1, float a2, int duration_ms) {
  unsigned long startT = millis();
  unsigned long scaledDur = (unsigned long)(duration_ms * SPEED_MULTIPLIER);
  
  while (true) {
    unsigned long now = millis();
    float t = (float)(now - startT) / scaledDur;
    if (t > 1.0) t = 1.0;

    float e = easeInOutCubic(t);
    float hip   = h1 + (h2 - h1) * e;
    float knee  = k1 + (k2 - k1) * e;
    float ankle = a1 + (a2 - a1) * e;

    hip   = angleConstrain(hip, HIP_MIN, HIP_MAX);
    knee  = angleConstrain(knee, KNEE_MIN, KNEE_MAX);
    ankle = angleConstrain(ankle, ANKLE_MIN, ANKLE_MAX);

    setServoAngle(HIP_CH, hip);
    setServoAngle(KNEE_CH, knee);
    setServoAngle(ANKLE_CH, ankle);

    if (t >= 1.0) break;
    delay(SERVO_REFRESH_MS);
  }
}

// ---- Gait logic (same as before) ----
void stepCycleSingleLeg() {
  float hip0   = hipHome;
  float knee0  = kneeHome;
  float ankle0 = ankleHome;

  // Phase 1: Lift leg
  float ankleLift = max(90.0 + ANKLE_MIN, ankle0 - 18.0);
  float kneeLift  = min(90.0 + KNEE_MAX, knee0 + 35.0);
  float hipPrep   = hip0;
  moveLegSmoothCubic(hip0, hipPrep, knee0, kneeLift, ankle0, ankleLift, LIFT_TIME_MS);

  // Phase 2: Swing leg forward
  float hipForward = constrain(hip0 + 25.0, 90.0 + HIP_MIN, 90.0 + HIP_MAX);
  moveLegSmoothCubic(hipPrep, hipForward, kneeLift, kneeLift, ankleLift, ankleLift, SWING_TIME_MS);

  // Phase 3: Place leg down
  moveLegSmoothCubic(hipForward, hipForward, kneeLift, knee0, ankleLift, ankle0, PLACE_TIME_MS);

  // Phase 4: Recover
  moveLegSmoothCubic(hipForward, hip0, knee0, knee0, ankle0, ankle0, RECOVER_TIME_MS);

  delay((int)(CYCLE_DELAY_MS * SPEED_MULTIPLIER));
}

