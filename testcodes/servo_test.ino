#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- CONFIGURATION ---
#define SERVO_PIN 0      // The channel you are testing
#define SERVO_MIN 150    // Your verified Min Pulse
#define SERVO_MAX 600    // Your verified Max Pulse

// Speed Settings
#define SPEED_FAST_DELAY 0  // 0ms = Full Speed (The problem)
#define SPEED_SLOW_DELAY 15 // 15ms per degree = Smooth Motion (The solution)

void setup() {
  Serial.begin(115200);
  Serial.println(">>> Starting Inertia Test <<<");

  pwm.begin();
  pwm.setPWMFreq(50);  // Standard Analog Servo Frequency

  // Move to start position immediately
  setServoAngle(SERVO_PIN, 0);
  delay(1000);
}

void loop() {
  // 1. DEMONSTRATION: The Problem (Snap Movement)
  Serial.println("Performing FAST move (Watch for overshoot)...");
  // Jumping directly from 0 to 90
  setServoAngle(SERVO_PIN, 90); 
  delay(1000); // Wait for the shake to settle
  
  // Reset
  setServoAngle(SERVO_PIN, 0);
  delay(1000);

  // 2. DEMONSTRATION: The Solution (Smooth Interpolation)
  Serial.println("Performing SMOOTH move (Control)...");
  
  // We manually control the loop to move 0 -> 90
  moveSmooth(SERVO_PIN, 0, 90, SPEED_SLOW_DELAY);
  
  Serial.println("Done. Pause...");
  delay(2000);
  
  // Move back smoothly
  moveSmooth(SERVO_PIN, 90, 0, SPEED_SLOW_DELAY);
  delay(1000);
}

// --- HELPER FUNCTIONS ---

// The Solution: Broken down movement
void moveSmooth(int channel, int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int i = startAngle; i <= endAngle; i++) {
      setServoAngle(channel, i);
      delay(stepDelay); // This controls the speed!
    }
  } else {
    for (int i = startAngle; i >= endAngle; i--) {
      setServoAngle(channel, i);
      delay(stepDelay);
    }
  }
}

// Basic driver function
void setServoAngle(int channel, int angle) {
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}
