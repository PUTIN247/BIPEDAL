#include <Servo.h>

Servo myServo;
int angle = 0;

void setup() {
  myServo.attach(9);
  Serial.begin(9600);
}

void loop() {
  Serial.println("Starting new sequence...");
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

  // A brief pause before the loop repeats and snaps the servo back to 0
  Serial.println("Sequence complete. Repeating...");
  delay(1000);
}
