// Include the Servo library to control the servo motor
#include <Servo.h>

// Create a servo object named 'myServo'
Servo myServo;

// A variable to store the servo's current angle
int currentAngle = 0;

void setup() {
  // Attach the servo object to digital pin 9 on the Arduino.
  // Pin 9 is a PWM (~) pin. You can change this to any other PWM pin.
  myServo.attach(9);

  // Start serial communication at 9600 bits per second for debugging
  Serial.begin(9600);
  Serial.println("Servo MG995R Test: 1 degree per second");

  // Move the servo to the starting position (0 degrees)
  myServo.write(currentAngle);
  Serial.println("Setting servo to initial position: 0 degrees");
  
  // Wait a second for the servo to reach the initial position before starting the main loop
  delay(1000); 
}

void loop() {
  // Check if the servo has reached its final destination (180 degrees)
  if (currentAngle <= 180) {
    // Command the servo to move to the 'currentAngle' position
    myServo.write(currentAngle);

    // Print the current angle to the Serial Monitor
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);

    // Increment the angle by one degree for the next loop iteration
    currentAngle++;

    // Wait for 1000 milliseconds (1 second) before the next move.
    // This delay creates the "one degree per second" speed.
    delay(1000);
  }
  // Once the angle is > 180, the code in the 'if' statement will no longer run,
  // and the servo will remain at the 180-degree position.
}
