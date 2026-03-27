// =============================================================================
//  Stage1_GaitTest.ino  —  BIPDEAL Stage-1: Single-Leg Cubic Bézier Gait Test
// =============================================================================
//
//  Purpose
//  ───────
//  Verify that one leg executes a smooth, continuous gait cycle using a
//  Cubic Bézier trajectory generator. This replaces the sinusoidal approach
//  and eliminates the velocity discontinuities observed with discrete-segment
//  motion.
//
//  Gait summary
//  ───────────────────────────────────────────────────────────────────────────
//  φ ∈ [0.0, 0.5)  SWING   foot airborne on Bézier arch
//                          P0(−4,−15) → peak(~0,−13) → P3(3,−15)
//                          horizontal velocity = 0 at lift-off and touch-down
//
//  φ ∈ [0.5, 1.0)  STANCE  foot on ground, reversed Bézier x-profile
//                          x: 3 → −4 cm   (equal-magnitude, opposite-direction)
//                          y: −15 cm constant
//
//  Velocity at both phase boundaries is zero in x → no jitter, no foot-drag.
//
//  Startup sequence
//  ───────────────────────────────────────────────────────────────────────────
//  1. I2C bus and PCA9685 initialised.
//  2. Calibration offsets applied to the leg.
//  3. Leg moves directly to home position (−2, −15) cm.
//  4. Sketch waits HOME_SETTLE_MS for the servo to physically arrive.
//  5. Gait loop starts; φ is derived from millis() for clean wrap-around.
//
//  Serial telemetry  (115200 baud)
//  ───────────────────────────────────────────────────────────────────────────
//  When SERIAL_PLOT is defined, each update tick streams CSV:
//      phi, x_cm, y_cm, hip_deg, knee_deg, ankle_deg
//  This feeds directly into the Arduino IDE Serial Plotter.
//  Comment out #define SERIAL_PLOT to reduce CPU load in production.
//
//  Hardware wiring (adjust channel constants below to match your build)
//  ───────────────────────────────────────────────────────────────────────────
//  PCA9685  I2C address  0x40
//  Channel  0  →  Hip servo
//  Channel  1  →  Knee servo
//  Channel  2  →  Ankle servo
//
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Bipedal.h"
#include "Trajectory.h"

// =============================================================================
//  ── CONFIGURATION  (edit to match your hardware and desired gait) ───────────
// =============================================================================

// ---- PCA9685 I2C address ------------------------------------------------
#define PCA9685_ADDR  0x40

// ---- PCA9685 oscillator frequency (Hz) ---------------------------------
// Measure with a scope for accuracy; typical boards are 25–27 MHz.
#define OSC_FREQUENCY  27000000UL

// ---- Servo channels on the PCA9685 (adjust to your wiring) -------------
constexpr uint8_t CH_HIP    = 0;
constexpr uint8_t CH_KNEE   = 1;
constexpr uint8_t CH_ANKLE  = 2;

// ---- Leg side -----------------------------------------------------------
// false = left leg  (default for Stage 1)
// true  = right leg (mirrors x-axis inside BipedLeg; swap when testing the right)
constexpr bool LEG_MIRROR_X = false;

// ---- Calibration offsets (degrees) -------------------------------------
// Adjust each offset until the joint physically reads 0° (leg straight,
// vertical) when commanded to home position. Re-tune after any reassembly.
constexpr float CAL_HIP_OFFSET    =  0.0f;
constexpr float CAL_KNEE_OFFSET   =  0.0f;
constexpr float CAL_ANKLE_OFFSET  =  0.0f;

// ---- Gait timing --------------------------------------------------------
// Total duration of one complete gait cycle (swing + stance) in milliseconds.
// Increase for slower, safer motion; decrease for faster stepping.
// Tested range on Arduino Uno: 1500 ms – 4000 ms.
constexpr uint32_t GAIT_CYCLE_MS = 2000UL;     // 2 s per full cycle

// Servo update interval in milliseconds (= 1 / update_rate).
// 20 ms → 50 Hz, matching the PCA9685 PWM frequency.
// Includes ~1–2 ms I2C overhead per 3-servo write on Arduino Uno.
constexpr uint32_t UPDATE_INTERVAL_MS = 20UL;

// ---- Home-position settle time -----------------------------------------
// How long (ms) to hold the home position on startup before gait begins.
// Gives the servo time to physically travel from its power-on position.
constexpr uint32_t HOME_SETTLE_MS = 1000UL;

// ---- Serial telemetry --------------------------------------------------
// Define SERIAL_PLOT to stream φ + foot coordinates + joint angles each tick.
// Comment this line out to eliminate Serial overhead in production.
#define SERIAL_PLOT

// =============================================================================
//  ── GLOBAL OBJECTS ──────────────────────────────────────────────────────────
// =============================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

BipedLeg  leg(pwm, CH_HIP, CH_KNEE, CH_ANKLE, LEG_MIRROR_X);

Trajectory traj;    // constructed with defaultParams(); overridden in setup()

// =============================================================================
//  ── SETUP ───────────────────────────────────────────────────────────────────
// =============================================================================
void setup() {

    Serial.begin(115200);
    while (!Serial) { /* wait for USB CDC on Leonardo/Micro; nop on Uno */ }

    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════════╗"));
    Serial.println(F("║  BIPDEAL — Stage 1: Cubic Bézier Gait Test       ║"));
    Serial.println(F("╚══════════════════════════════════════════════════╝"));
    Serial.println(F(""));

    // ── Initialise I2C and PCA9685 ───────────────────────────────────────
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(OSC_FREQUENCY);
    pwm.setPWMFreq(50);     // 50 Hz — standard servo refresh rate
    delay(10);              // let PCA9685 stabilise after power-on

    // ── Apply calibration offsets ────────────────────────────────────────
    leg.setOffsets(CAL_HIP_OFFSET, CAL_KNEE_OFFSET, CAL_ANKLE_OFFSET);

    // ── Configure trajectory parameters ──────────────────────────────────
    //
    //  Using specification defaults. To change at runtime call:
    //      Trajectory::Params p = traj.getParams();
    //      p.p1y = -13.5f;   // lower the arch peak, e.g. to reduce knee angle
    //      traj.setParams(p);
    //
    //  GEOMETRY RULES — see Trajectory.h for full details:
    //    p1x must equal p0x  → vertical lift-off tangent
    //    p2x must equal p3x  → vertical touch-down tangent
    //    p0y and p3y must equal groundY  (both on the ground line)
    //
    Trajectory::Params p = Trajectory::defaultParams();
    traj.setParams(p);

    // ── Print confirmed trajectory parameters ─────────────────────────────
    Serial.println(F("  Trajectory: Cubic Bézier"));
    Serial.println(F("  ─────────────────────────────────────────────────"));
    Serial.print(F("  P0 (lift-off)    = (")); Serial.print(p.p0x, 1);
    Serial.print(F(", ")); Serial.print(p.p0y, 1); Serial.println(F(") cm"));

    Serial.print(F("  P1 (swing ctrl)  = (")); Serial.print(p.p1x, 1);
    Serial.print(F(", ")); Serial.print(p.p1y, 1); Serial.println(F(") cm"));

    Serial.print(F("  P2 (land ctrl)   = (")); Serial.print(p.p2x, 1);
    Serial.print(F(", ")); Serial.print(p.p2y, 1); Serial.println(F(") cm"));

    Serial.print(F("  P3 (touch-down)  = (")); Serial.print(p.p3x, 1);
    Serial.print(F(", ")); Serial.print(p.p3y, 1); Serial.println(F(") cm"));

    Serial.println(F("  ─────────────────────────────────────────────────"));
    Serial.print(F("  Step length      = "));
    Serial.print(p.p3x - p.p0x, 1); Serial.println(F(" cm"));

    Serial.print(F("  Arch height      = "));
    Serial.print(p.p0y - p.p1y, 1); Serial.println(F(" cm  (ground − peakY)"));

    Serial.print(F("  Ground line      = "));
    Serial.print(p.p0y, 1); Serial.println(F(" cm"));

    Serial.print(F("  Home position    = ("));
    Serial.print(p.homeX, 1); Serial.print(F(", "));
    Serial.print(p.homeY, 1); Serial.println(F(") cm"));

    Serial.print(F("  Cycle period     = ")); Serial.print(GAIT_CYCLE_MS);
    Serial.println(F(" ms"));

    Serial.print(F("  Update rate      = "));
    Serial.print(1000UL / UPDATE_INTERVAL_MS); Serial.println(F(" Hz"));
    Serial.println(F(""));

    // ── Move to home position ─────────────────────────────────────────────
    //
    //  Drive the leg directly to the balanced standing pose before gait.
    //  HOME_SETTLE_MS gives the servo time to physically reach the target.
    //
    FootPosition home = traj.getHomePosition();
    Serial.print(F("  Moving to home ("));
    Serial.print(home.x, 1); Serial.print(F(", "));
    Serial.print(home.y, 1); Serial.println(F(") cm  …"));

    leg.moveTo(home.x, home.y);
    delay(HOME_SETTLE_MS);

    Serial.println(F("  Home reached.  Starting gait cycle …"));
    Serial.println(F(""));

#ifdef SERIAL_PLOT
    // CSV header — matches Arduino IDE Serial Plotter column format
    Serial.println(F("phi,x_cm,y_cm,hip_deg,knee_deg,ankle_deg"));
#endif
}

// =============================================================================
//  ── MAIN LOOP ───────────────────────────────────────────────────────────────
// =============================================================================
void loop() {

    // ── Non-blocking rate limiter ─────────────────────────────────────────
    //
    //  Run the control update at exactly UPDATE_INTERVAL_MS intervals.
    //  Using subtraction (now − lastUpdate) handles millis() rollover
    //  safely without any special case.
    //
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();

    if (now - lastUpdate < UPDATE_INTERVAL_MS) return;
    lastUpdate = now;

    // ── Compute normalised gait phase φ ∈ [0, 1) ─────────────────────────
    //
    //  millis() rolls over after ~49.7 days.
    //  The modulo operation yields a clean, monotonically-increasing phase
    //  that wraps from 1 back to 0 without any explicit counter reset.
    //
    float phi = static_cast<float>(now % GAIT_CYCLE_MS)
              / static_cast<float>(GAIT_CYCLE_MS);

    // ── Get Bézier foot target for this phase ─────────────────────────────
    FootPosition fp = traj.getFootPosition(phi);

    // ── Drive the leg  (IK + servo write inside moveTo) ───────────────────
    leg.moveTo(fp.x, fp.y);

    // ── Optional serial telemetry ─────────────────────────────────────────
    //
    //  calculateIK() is called a second time here purely for telemetry.
    //  Cost: ~0.4 ms on Uno at 16 MHz — acceptable at 50 Hz.
    //  Remove the #define SERIAL_PLOT block above to eliminate this overhead.
    //
#ifdef SERIAL_PLOT
    IKResult ik = leg.calculateIK(fp.x, fp.y);

    Serial.print(phi, 3);       Serial.print(F(","));
    Serial.print(fp.x, 2);      Serial.print(F(","));
    Serial.print(fp.y, 2);      Serial.print(F(","));
    Serial.print(ik.hip,   1);  Serial.print(F(","));
    Serial.print(ik.knee,  1);  Serial.print(F(","));
    Serial.println(ik.ankle, 1);
#endif
}
