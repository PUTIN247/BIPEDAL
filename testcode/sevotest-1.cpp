#include <ESP32Servo.h>   // servo library for ESP32

Servo myservo;           // create a Servo object

int servoPin = 18;       // GPIO pin the servo signal is on
int angle = 20;          // current angle (degrees)
int direction = 1;       // +1 = increasing angle, -1 = decreasing angle
int minAngle = 20;       // lower bound (degrees)
int maxAngle = 160;      // upper bound (degrees)
int stepDelay = 80;      // ms delay between steps (controls speed)

void setup() {
  Servo::useTimer(0);          // select hardware timer 0 for PWM (must before attach)
  myservo.attach(servoPin);    // attach servo to pin (start PWM output)
  myservo.write(angle);        // set initial servo position (20°)
  delay(500);                  // wait 500 ms so servo reaches initial position
}

void loop() {
  angle += direction;          // move the "target" angle by one step (±1°)

  if (angle >= maxAngle) {     // if we've hit or exceeded upper bound
    angle = maxAngle;          // clamp to the bound
    direction = -1;            // reverse direction (start moving down)
  } else if (angle <= minAngle) { // if we hit or go below lower bound
    angle = minAngle;          // clamp to lower bound
    direction = 1;             // reverse direction (start moving up)
  }

  myservo.write(angle);        // command the servo to go to 'angle' degrees
  delay(stepDelay);            // wait before next step (blocks CPU)
}
