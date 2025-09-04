#include <Servo.h>

Servo myServo;

int angle = 0;

void setup() {
  myServo.attach(9);
  Serial.begin(9600);

  Serial.println("Moving to initial position: 0");
  myServo.write(0);
  delay(500);

  for (angle = 0; angle <= 50; angle = angle + 10) {
    myServo.write(angle);
    Serial.print("Current Angle: ");
    Serial.println(angle);
    delay(150);
  }

  for (angle = 52; angle <= 180; angle = angle + 2) {
    myServo.write(angle);
    Serial.print("Current Angle: ");
    Serial.println(angle);
    delay(40);
  }

  Serial.println("Sequence Complete.");
}

void loop() {
  // The loop is empty because the entire sequence runs once in setup().
}
