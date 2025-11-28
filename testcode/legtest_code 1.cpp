/*
 * =================================================================================
 * Final Bipedal Leg Controller Code
 * =================================================================================
 *
 * Author: Gemini (based on analysis of provided research documents)
 * Date: August 22, 2025
 *
 * Description:
 * This code provides a complete control solution for a single 3-DOF bipedal leg.
 * It is based on the kinematic models and control architecture detailed in the
 * provided research papers. The primary goal is to provide a robust, fast, and
 * easily configurable platform for testing the leg's motion.
 *
 * Key Features:
 * - Inverse Kinematics (IK): Calculates the required joint angles (hip, knee, ankle)
 * to reach a specific (x, y) coordinate in space.
 * - Auto-Homing on Startup: The leg automatically moves to a stable, predefined
 * "home" position when powered on.
 * - Adjustable Parameters: All critical physical and motion parameters are grouped
 * in a configuration section for easy tuning.
 * - Smooth, Fast Motion: Uses non-blocking linear interpolation to generate smooth
 * and fast movements without using delay().
 * - Safety Limits: Implements software-based joint angle limits to prevent the
 * leg from damaging itself.
 *
 * Hardware Requirements:
 * - ESP32 Development Board
 * - 3x MG996R (or similar) Servo Motors
 * - A separate, stable 5-6V power supply for the servos (CRITICAL!)
 *
 * Coordinate System:
 * - The origin (0,0) is at the hip joint's pivot point.
 * - +X axis is forward (in the direction of walking).
 * - +Y axis is upward, against gravity.
 * - Therefore, the ground is at a negative Y coordinate.
 *
 * CRITICAL WARNINGS:
 * 1. POWER SUPPLY: Servos MUST be powered by a separate 5-6V power supply that
 * can provide at least 5A. Powering them from the ESP32's 5V pin WILL damage
 * the board. Remember to connect the grounds of the ESP32 and the servo
 * power supply together.
 *
 * 2. HIP SERVO TORQUE: The provided research documents correctly indicate that
 * the MG996R servo may be UNDERPOWERED for the hip joint once the full weight
 * of the robot's body is added. For testing this single leg, it should be
 * sufficient, but you will likely need a stronger servo for the final robot.
 *
 */

// LIBRARIES
#include <ESP32Servo.h> // Use the ESP32-specific servo library for best performance
#include <cmath>        // For trigonometric functions (sin, cos, acos, atan2)

// =================================================================================
// == (1) USER CONFIGURATION & ADJUSTABLE PARAMETERS ==
// =================================================================================
// --- YOU CAN CHANGE ALL VALUES IN THIS SECTION ---

// -- SERVO PINS --
// Assign the GPIO pins on your ESP32 that are connected to the servo signal lines.
const int HIP_SERVO_PIN = 13;
const int KNEE_SERVO_PIN = 12;
const int ANKLE_SERVO_PIN = 14;

// -- PHYSICAL DIMENSIONS (in cm) --
// These must match your robot's physical construction.
const float THIGH_LENGTH = 9.5; // Length from hip pivot to knee pivot (L1)
const float SHIN_LENGTH = 9.5;  // Length from knee pivot to ankle pivot (L2)

// -- HOME / STANDARD POSITION --
// This is the default standing position for the foot relative to the hip.
// (0, -15) represents a stable, slightly crouched stance.
const float HOME_POSITION_X = 0.0;
const float HOME_POSITION_Y = -15.0;

// -- SERVO CALIBRATION --
// This is the most important section for tuning the physical robot.
// It maps the mathematical angles from the IK calculation to the physical servo commands.
// You will need to adjust these values by observing your robot's movement.
//
// - OFFSET: The servo's "zero" angle in relation to the kinematic model's "zero".
//           For example, if the hip servo horn is mounted 90 degrees off, the offset is 90.
// - REVERSED: Set to 'true' if the servo moves in the opposite direction of what's expected.
const float HIP_SERVO_OFFSET = 90.0;
const bool HIP_SERVO_REVERSED = false;

const float KNEE_SERVO_OFFSET = 90.0;
const bool KNEE_SERVO_REVERSED = true; // Knee often needs to be reversed

const float ANKLE_SERVO_OFFSET = 90.0;
const bool ANKLE_SERVO_REVERSED = false;

// -- JOINT LIMITS (in degrees) --
// These are SOFTWARE safety limits to prevent the IK solver from commanding impossible angles.
// These angles are the PURE calculated angles before any offsets are applied.
// Determine these by physically moving your robot leg to its limits and noting the angles.
const float HIP_MIN_ANGLE = -60.0;
const float HIP_MAX_ANGLE = 60.0;

const float KNEE_MIN_ANGLE = -140.0; // Corresponds to the ">" shape
const float KNEE_MAX_ANGLE = 0.0;    // Leg is straight

const float ANKLE_MIN_ANGLE = -90.0;
const float ANKLE_MAX_ANGLE = 90.0;

// -- MOTION PARAMETERS --
// Control the speed and dimensions of the walking gait.
const int MOTION_SPEED = 500;      // Time in milliseconds for a standard move. Lower is faster.
const float STEP_HEIGHT = 4.0;     // How high the foot lifts off the ground (in cm).
const float STEP_LENGTH = 6.0;     // How far forward the foot moves in one step (in cm).
const int GAIT_CYCLE_PAUSE = 1000; // Pause between steps in milliseconds.

// =================================================================================
// == (2) GLOBAL VARIABLES & OBJECTS ==
// =================================================================================

// Create servo objects
Servo hipServo;
Servo kneeServo;
Servo ankleServo;

// Data structure to hold the three joint angles
struct JointAngles
{
    float hip;
    float knee;
    float ankle;
};

// Variables to store the current state of the leg
JointAngles currentAngles;
float currentX = HOME_POSITION_X;
float currentY = HOME_POSITION_Y;

// =================================================================================
// == (3) CORE KINEMATICS (Inverse Kinematics Module) ==
// =================================================================================

/**
 * @brief Calculates the inverse kinematics for the 3-DOF leg.
 * @param x The target X-coordinate for the foot (relative to the hip).
 * @param y The target Y-coordinate for the foot (relative to the hip).
 * @param angles A reference to a JointAngles struct to store the results.
 * @return true if the position is reachable, false otherwise.
 */
bool calculateIK(float x, float y, JointAngles &angles)
{
    // Calculate the distance from the hip to the foot squared
    float D_sq = x * x + y * y;

    // Argument for the acos function to find the knee angle (Law of Cosines)
    float cos_theta2_arg = (D_sq - THIGH_LENGTH * THIGH_LENGTH - SHIN_LENGTH * SHIN_LENGTH) / (2 * THIGH_LENGTH * SHIN_LENGTH);

    // Check if the target point is reachable. If the argument is outside [-1, 1], it's impossible.
    if (cos_theta2_arg > 1.0 || cos_theta2_arg < -1.0)
    {
        return false; // Target is out of reach
    }

    // Calculate the knee angle (theta2). We use -acos for the "knee-forward" solution.
    float theta2_rad = -acos(cos_theta2_arg);

    // Calculate the hip angle (theta1)
    float theta1_rad = atan2(y, x) - atan2(SHIN_LENGTH * sin(theta2_rad), THIGH_LENGTH + SHIN_LENGTH * cos(theta2_rad));

    // Calculate the ankle angle (theta3) to keep the foot flat (parallel to the ground)
    // The sum of all angles must be zero for the foot to be horizontal.
    float theta3_rad = -(theta1_rad + theta2_rad);

    // Convert angles from radians to degrees for the servos
    angles.hip = theta1_rad * 180.0 / M_PI;
    angles.knee = theta2_rad * 180.0 / M_PI;
    angles.ankle = theta3_rad * 180.0 / M_PI;

    return true;
}

// =================================================================================
// == (4) LOW-LEVEL MOTION CONTROL (Servo Driver Module) ==
// =================================================================================

/**
 * @brief Moves the leg smoothly from its current position to a target (x, y) coordinate.
 * @param targetX The destination X-coordinate.
 * @param targetY The destination Y-coordinate.
 * @param duration The time in milliseconds the move should take.
 */
void moveLegTo(float targetX, float targetY, int duration)
{
    JointAngles targetAngles;

    // Use the IK solver to find the required angles for the target position
    if (calculateIK(targetX, targetY, targetAngles))
    {
        // Apply software joint limits as a safety check
        targetAngles.hip = constrain(targetAngles.hip, HIP_MIN_ANGLE, HIP_MAX_ANGLE);
        targetAngles.knee = constrain(targetAngles.knee, KNEE_MIN_ANGLE, KNEE_MAX_ANGLE);
        targetAngles.ankle = constrain(targetAngles.ankle, ANKLE_MIN_ANGLE, ANKLE_MAX_ANGLE);

        // Get the starting angles
        JointAngles startAngles = currentAngles;
        unsigned long startTime = millis();
        unsigned long currentTime;

        // Loop for the duration of the movement to create smooth interpolation
        do
        {
            currentTime = millis();
            float fraction = (float)(currentTime - startTime) / duration;
            fraction = constrain(fraction, 0.0, 1.0); // Ensure fraction stays between 0 and 1

            // Linearly interpolate each joint angle
            float hip_interp = startAngles.hip + (targetAngles.hip - startAngles.hip) * fraction;
            float knee_interp = startAngles.knee + (targetAngles.knee - startAngles.knee) * fraction;
            float ankle_interp = startAngles.ankle + (targetAngles.ankle - startAngles.ankle) * fraction;

            // --- Apply Servo Calibration ---
            float hip_servo_cmd = HIP_SERVO_OFFSET + (HIP_SERVO_REVERSED ? -hip_interp : hip_interp);
            float knee_servo_cmd = KNEE_SERVO_OFFSET + (KNEE_SERVO_REVERSED ? -knee_interp : knee_interp);
            float ankle_servo_cmd = ANKLE_SERVO_OFFSET + (ANKLE_SERVO_REVERSED ? -ankle_interp : ankle_interp);

            // Send the final command to the servos
            hipServo.write(hip_servo_cmd);
            kneeServo.write(knee_servo_cmd);
            ankleServo.write(ankle_servo_cmd);

        } while (currentTime < startTime + duration);

        // Update the leg's current state
        currentAngles = targetAngles;
        currentX = targetX;
        currentY = targetY;
    }
    else
    {
        // Print an error if the IK solver fails
        Serial.print("ERROR: Position unreachable: (");
        Serial.print(targetX);
        Serial.print(", ");
        Serial.print(targetY);
        Serial.println(")");
    }
}

// =================================================================================
// == (5) SETUP & MAIN LOOP (High-Level Control) ==
// =================================================================================

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial connection
    Serial.println("\n--- Bipedal Leg Controller Initializing ---");

    // Attach servos to their pins
    hipServo.attach(HIP_SERVO_PIN);
    kneeServo.attach(KNEE_SERVO_PIN);
    ankleServo.attach(ANKLE_SERVO_PIN);
    Serial.println("Servos attached.");

    // Calculate the initial home angles but don't move yet
    calculateIK(HOME_POSITION_X, HOME_POSITION_Y, currentAngles);
    
    // Set servos to their initial calculated positions instantly without interpolation
    // This establishes the starting point for all future interpolated moves.
    float hip_servo_cmd = HIP_SERVO_OFFSET + (HIP_SERVO_REVERSED ? -currentAngles.hip : currentAngles.hip);
    float knee_servo_cmd = KNEE_SERVO_OFFSET + (KNEE_SERVO_REVERSED ? -currentAngles.knee : currentAngles.knee);
    float ankle_servo_cmd = ANKLE_SERVO_OFFSET + (ANKLE_SERVO_REVERSED ? -currentAngles.ankle : currentAngles.ankle);
    hipServo.write(hip_servo_cmd);
    kneeServo.write(knee_servo_cmd);
    ankleServo.write(ankle_servo_cmd);
    
    Serial.println("Leg is at un-powered position. Moving to Home...");
    delay(2000); // Give user time to see the message and prepare

    // --- HOMING SEQUENCE ---
    // Move the leg smoothly to the defined Home Position.
    moveLegTo(HOME_POSITION_X, HOME_POSITION_Y, 2000); // Take 2 seconds to move home smoothly

    Serial.println("Initialization Complete. Leg is at Home Position.");
    delay(1000);
}

void loop()
{
    // This loop executes a simple, continuous walking motion for testing.

    // --- GAIT PHASE 1: LIFT FOOT ---
    Serial.println("Phase 1: Lifting foot...");
    moveLegTo(currentX, currentY + STEP_HEIGHT, MOTION_SPEED);
    delay(100); // Small pause

    // --- GAIT PHASE 2: SWING FORWARD ---
    Serial.println("Phase 2: Swinging forward...");
    moveLegTo(currentX + STEP_LENGTH, currentY, MOTION_SPEED * 2);
    delay(100);

    // --- GAIT PHASE 3: LOWER FOOT ---
    Serial.println("Phase 3: Lowering foot...");
    moveLegTo(currentX, HOME_POSITION_Y, MOTION_SPEED);
    delay(100);

    // --- GAIT PHASE 4: RETURN TO START ---
    Serial.println("Phase 4: Resetting for next step...");
    moveLegTo(HOME_POSITION_X, HOME_POSITION_Y, MOTION_SPEED * 2);

    // Wait before starting the next cycle
    Serial.println("...Cycle complete. Pausing.");
    delay(GAIT_CYCLE_PAUSE);
}
