#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PCA9685 object at default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo pulse length limits (adjust if needed for your servo)
#define SERVOMIN  150  // Minimum pulse length count (0°)
#define SERVOMAX  600  // Maximum pulse length count (180°)

int servoChannel = 0; // PCA9685 channel (0-15)

void setup() {
  Serial.begin(115200);
  Serial.println("Moving servo to 90 degrees...");

  pwm.begin();
  pwm.setPWMFreq(50); // Analog servos run at ~50 Hz

  delay(10);

  // Move servo to 90 degrees
  setServoAngle(servoChannel, 90);
}

void loop() {
  // Nothing here, servo stays at 90°
}

// Function to map angle (0-180) to pulse and set servo
void setServoAngle(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}
