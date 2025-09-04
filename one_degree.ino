#include <Servo.h>

const int servoPin = 3;   // Servo pin
Servo servo;

// Unknown starting angle variable, set this to what you believe servo's current angle is before running
float currentAngle;      

float targetAngle = 45.0;     // Final target angle after moving to zero first

float progress = 0.0;         // Easing progress tracker
bool movingToZero = true;     // First phase: move to zero
bool finished = false;        // Movement finished flag

const int stepDelay =   1000;     // Delay between updates (ms)
const float moveStep = 1.0;   // Max degrees per update
const float minStep = 0.5;    // Min degrees per update

void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);

  // Initialize currentAngle here from any assumed starting angle or let it be manually set before running
  currentAngle = 90.0;  // Change this to the actual servo angle if known, or any guess
  
  servo.write((int)currentAngle);
  Serial.print("Starting Angle: ");
  Serial.println((int)currentAngle);
}

void loop() {
  if (!finished) {
    float desiredTarget = movingToZero ? 0.0 : targetAngle;
    float diff = desiredTarget - currentAngle;

    if (abs(diff) > moveStep) {
      float frac = progress / (abs(diff) + 1.0);
      if (frac > 1.0) frac = 1.0;

      float step = moveStep * sin(frac * (PI / 2.0));

      if (abs(step) < minStep && abs(diff) > minStep) {
        step = (diff > 0) ? minStep : -minStep;
      }

      currentAngle += (diff > 0) ? step : -step;

      if (currentAngle < 0.0) currentAngle = 0.0;
      if (currentAngle > 180.0) currentAngle = 180.0;

      servo.write((int)currentAngle);
      progress += abs(step);

      Serial.print("Current Angle: ");
      Serial.println((int)currentAngle);
    }
    else {
      // Snap to target and switch phase or finish
      currentAngle = desiredTarget;
      servo.write((int)currentAngle);
      Serial.print("Current Angle: ");
      Serial.println((int)currentAngle);

      if (movingToZero) {
        movingToZero = false;
        progress = 0.0;  // Reset progress for next move
      }
      else {
        finished = true;
        Serial.println("Movement Finished.");
      }
    }
  }
  else {
    // Hold position indefinitely
    delay(1000);
  }
  delay(stepDelay);
}
