#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h"

// NOTE: On Arduino Uno, I2C is fixed to:
// SDA = A4
// SCL = A5

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define Channels: Hip=0, Knee=1, Ankle=2
BipedLeg leftLeg(&pwm, 0, 1, 2);

// Current Joint Angles (Degrees relative to 90 neutral)
float currentHip = 0;
float currentKnee = 0;
float currentAnkle = 0;

// Step size per key press
float stepSize = 5.0; 

void setup() {
    Serial.begin(115200); // Make sure Serial Monitor matches this speed
    
    // CHANGE: Uno uses default pins (A4/A5), no arguments needed
    Wire.begin(); 
    
    pwm.begin();
    pwm.setPWMFreq(50);

    // Apply your Offsets here if needed
    leftLeg.setOffsets(0.0, 0.0, 0.0);

    Serial.println("=== STAGE 1: MANUAL VIRTUAL JOYSTICK (UNO) ===");
    Serial.println("Controls:");
    Serial.println("  Hip:   W (+) / S (-)");
    Serial.println("  Knee:  E (+) / D (-)");
    Serial.println("  Ankle: R (+) / F (-)");
    Serial.println("  Reset: X");
    Serial.println("----------------------------------------");
    
    updateServos(); // Move to start position
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        // --- CONTROL LOGIC ---
        switch (cmd) {
            // HIP CONTROL
            case 'w': currentHip += stepSize; break;
            case 's': currentHip -= stepSize; break;

            // KNEE CONTROL
            case 'e': currentKnee += stepSize; break;
            case 'd': currentKnee -= stepSize; break;

            // ANKLE CONTROL
            case 'r': currentAnkle += stepSize; break;
            case 'f': currentAnkle -= stepSize; break;

            // RESET
            case 'x': 
                currentHip = 0;
                currentKnee = 0; 
                currentAnkle = 0; 
                Serial.println("Resetting to Home.");
                break;
        }

        // --- CRITICAL SAFETY LIMITS ---
        // Hip: -45 to 45
        currentHip = constrain(currentHip, -45.0, 45.0);
        
        // Knee: 0 to 90 (NEVER EXCEED 90 per constraints)
        currentKnee = constrain(currentKnee, 0.0, 90.0);
        
        // Ankle: -40 to 40
        currentAnkle = constrain(currentAnkle, -40.0, 40.0);

        // --- UPDATE SERVOS ---
        updateServos();
        printStatus();
    }
}

void updateServos() {
    // We add 90 because 0 degrees in your logic = 90 degrees on the physical servo (Neutral)
    leftLeg.setServoRaw(0, 90.0 + currentHip);
    leftLeg.setServoRaw(1, 90.0 + currentKnee);
    leftLeg.setServoRaw(2, 90.0 + currentAnkle);
}

void printStatus() {
    Serial.print("Hip: "); Serial.print(currentHip);
    Serial.print(" | Knee: "); Serial.print(currentKnee);
    Serial.print(" | Ankle: "); Serial.println(currentAnkle);
}
