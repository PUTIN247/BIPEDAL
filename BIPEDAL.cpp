#include <ServoEasing.h>
#include <Servo.h>

#define LINK_1_LENGTH 9.5f
#define LINK_2_LENGTH 9.5f

#define HIP_SERVO_PIN   9
#define KNEE_SERVO_PIN  10
#define ANKLE_SERVO_PIN 11

ServoEasing HipServo;
ServoEasing KneeServo;
ServoEasing AnkleServo;

#define STEP_HEIGHT      3.0f
#define STEP_LENGTH      8.0f
#define GAIT_CYCLE_TIME  2000
#define NUM_WAYPOINTS    50

#define HOME_X 0.0f
#define HOME_Y 16.45f

float currentX = HOME_X;
float currentY = HOME_Y;

bool calculateIK(float x, float y, float &hipAngle, float &kneeAngle, float &ankleAngle) {
    float D_sq = x * x + y * y;
    float D = sqrt(D_sq);

    if (D > (LINK_1_LENGTH + LINK_2_LENGTH) |

| D < abs(LINK_1_LENGTH - LINK_2_LENGTH)) {
        return false;
    }

    float cos_theta2 = (D_sq - LINK_1_LENGTH * LINK_1_LENGTH - LINK_2_LENGTH * LINK_2_LENGTH) / (2 * LINK_1_LENGTH * LINK_2_LENGTH);
    
    if (cos_theta2 < -1.0) cos_theta2 = -1.0;
    if (cos_theta2 > 1.0) cos_theta2 = 1.0;

    float theta2_rad = acos(cos_theta2);

    float gamma_rad = atan2(x, y);
    float cos_beta = (D_sq + LINK_1_LENGTH * LINK_1_LENGTH - LINK_2_LENGTH * LINK_2_LENGTH) / (2 * D * LINK_1_LENGTH);

    if (cos_beta < -1.0) cos_beta = -1.0;
    if (cos_beta > 1.0) cos_beta = 1.0;
    
    float beta_rad = acos(cos_beta);
    
    float theta1_rad = gamma_rad - beta_rad;

    float theta3_rad = - (theta1_rad + theta2_rad);

    hipAngle = theta1_rad * 180.0 / PI;
    kneeAngle = theta2_rad * 180.0 / PI;
    ankleAngle = theta3_rad * 180.0 / PI;

    return true;
}

void moveLegTo(float targetX, float targetY, int moveDuration) {
    float hip_deg, knee_deg, ankle_deg;

    if (calculateIK(targetX, targetY, hip_deg, knee_deg, ankle_deg)) {
        float servo_hip_target = 90.0 - hip_deg;
        float servo_knee_target = knee_deg;
        float servo_ankle_target = 90.0 + ankle_deg;

        HipServo.setEaseTo(servo_hip_target);
        KneeServo.setEaseTo(servo_knee_target);
        AnkleServo.setEaseTo(servo_ankle_target);

        synchronizeAllServosAndStartInterrupt(moveDuration);
        while(areAllServosRunning());

        currentX = targetX;
        currentY = targetY;
    } else {
        Serial.print("Move failed: Unreachable position (");
        Serial.print(targetX); Serial.print(", "); Serial.print(targetY);
        Serial.println(")");
    }
}

void executeWalkCycle() {
    Serial.println("--- Starting Walk Cycle ---");

    Serial.println("Phase 1: Lift Off");
    moveLegTo(currentX, currentY - STEP_HEIGHT, GAIT_CYCLE_TIME / 6);

    Serial.println("Phase 2: Swing Forward");
    float startX = currentX;
    float startY = currentY;
    float endX = startX + STEP_LENGTH;

    for (int i = 1; i <= NUM_WAYPOINTS; i++) {
        float fraction = (float)i / NUM_WAYPOINTS;
        float interpX = startX + (endX - startX) * fraction;
        float interpY = startY - sin(fraction * PI) * STEP_HEIGHT; 
        
        float hip_deg, knee_deg, ankle_deg;
        if (calculateIK(interpX, interpY, hip_deg, knee_deg, ankle_deg)) {
          float servo_hip_target = 90.0 - hip_deg;
          float servo_knee_target = knee_deg;
          float servo_ankle_target = 90.0 + ankle_deg;
          
          HipServo.setEaseTo(servo_hip_target);
          KneeServo.setEaseTo(servo_knee_target);
          AnkleServo.setEaseTo(servo_ankle_target);
        }
    }
    synchronizeAllServosAndStartInterrupt(GAIT_CYCLE_TIME * 2 / 3);
    while(areAllServosRunning());
    currentX = endX;
    currentY = startY;

    Serial.println("Phase 3: Touch Down");
    moveLegTo(currentX, HOME_Y, GAIT_CYCLE_TIME / 6);

    delay(1000);

    Serial.println("Returning to Home Position");
    moveLegTo(HOME_X, HOME_Y, 1000);
    delay(2000);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\n3-DOF Bipedal Leg Controller Initializing...");

    HipServo.attach(HIP_SERVO_PIN, 90);
    KneeServo.attach(KNEE_SERVO_PIN, 0);
    AnkleServo.attach(ANKLE_SERVO_PIN, 90);
    
    setSpeedForAllServos(120); 

    Serial.println("Servos attached. Moving to home position...");

    moveLegTo(HOME_X, HOME_Y, 2000);

    Serial.println("Initialization Complete. Ready to walk.");
    delay(1000);
}

void loop() {
    executeWalkCycle();
}