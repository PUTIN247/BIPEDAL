// =============================================================================
//  Stage1_GaitTest.ino  —  BIPDEAL Stage-1: Single-Leg Sinusoidal Gait Test
// =============================================================================
//
//  Purpose
//  ───────
//  Verify that one leg executes a smooth, continuous gait cycle using
//  the sinusoidal Trajectory generator. This replaces the original
//  discrete-segment approach that caused servo jitter at phase boundaries.
//
//  Gait summary
//  ───────────────────────────────────────────────────────────────────────
//  φ ∈ [0.0, 0.5)  SWING   foot airborne, moves forward  (x: −3.5 → +3.5 cm)
//  φ ∈ [0.5, 1.0)  STANCE  foot on ground, moves backward (x: +3.5 → −3.5 cm)
//
//  Startup sequence
//  ───────────────────────────────────────────────────────────────────────
//  1. PCA9685 initialised
//  2. Leg moves directly to home position  (−2, −15)  and holds for
//     HOME_SETTLE_MS to let the servo reach the target physically.
//  3. Gait loop starts; φ is derived from millis() so it wraps cleanly.
//
//  Hardware wiring (adjust channel constants below to match your build)
//  ───────────────────────────────────────────────────────────────────────
//  PCA9685  I2C address  0x40
//  Channel  0  →  Hip servo
//  Channel  1  →  Knee servo
//  Channel  2  →  Ankle servo
//
//  Serial monitor  (115200 baud)
//  ───────────────────────────────────────────────────────────────────────
//  During gait, the sketch streams CSV-formatted telemetry:
//    phi, x_cm, y_cm, hip_deg, knee_deg, ankle_deg
//  Comment out the SERIAL_PLOT block in loop() to reduce CPU load.
//
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BipedLeg.h"
#include "Trajectory.h"

// =============================================================================
//  ── CONFIGURATION  (edit these to match your hardware & desired gait) ──────
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
// false = left leg (default for Stage 1)
// true  = right leg (mirrors the x-axis inside BipedLeg)
constexpr bool LEG_MIRROR_X = false;

// ---- Calibration offsets (degrees) -------------------------------------
// Increase/decrease each offset until the joint reads 0° (straight, vertical)
// at the home position. Re-run after any mechanical reassembly.
constexpr float CAL_HIP_OFFSET    =  0.0f;
constexpr float CAL_KNEE_OFFSET   =  0.0f;
constexpr float CAL_ANKLE_OFFSET  =  0.0f;

// ---- Gait timing --------------------------------------------------------
// Total duration of one complete gait cycle in milliseconds.
// Increase for slower, safer motion; decrease for faster walking.
// Range tested on Arduino Uno: 1500 ms – 4000 ms
constexpr uint32_t GAIT_CYCLE_MS    = 2000UL;   // 2 s per full cycle

// Servo update interval in milliseconds → servo refresh rate.
// 20 ms = 50 Hz, which matches the PCA9685 PWM frequency and is safe for
// Arduino Uno including I2C overhead (~1–2 ms per 3-servo write).
constexpr uint32_t UPDATE_INTERVAL_MS = 20UL;

// ---- Home-position settle time -----------------------------------------
// Time to hold the home position on startup before the gait loop begins.
// Gives the servo time to physically reach the target from rest.
constexpr uint32_t HOME_SETTLE_MS = 1000UL;

// ---- Serial telemetry --------------------------------------------------
// Define SERIAL_PLOT to stream φ + foot + joint angles each update tick.
// Comment this line out to eliminate Serial overhead in production.
#define SERIAL_PLOT

// =============================================================================
//  ── GLOBAL OBJECTS ──────────────────────────────────────────────────────────
// =============================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

BipedLeg leg(pwm, CH_HIP, CH_KNEE, CH_ANKLE, LEG_MIRROR_X);

Trajectory traj;   // constructed with defaultParams(); overridden in setup()

// =============================================================================
//  ── SETUP ───────────────────────────────────────────────────────────────────
// =============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { /* wait for USB CDC on Leonardo/Micro; nop on Uno */ }

    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════╗"));
    Serial.println(F("║  BIPDEAL — Stage 1: Sinusoidal Gait Test     ║"));
    Serial.println(F("╚══════════════════════════════════════════════╝"));

    // ── Initialise I2C and PCA9685 ───────────────────────────────────────
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(OSC_FREQUENCY);
    pwm.setPWMFreq(50);     // 50 Hz — standard servo refresh rate
    delay(10);              // let PCA9685 stabilise

    // ── Apply calibration offsets ────────────────────────────────────────
    leg.setOffsets(CAL_HIP_OFFSET, CAL_KNEE_OFFSET, CAL_ANKLE_OFFSET);

    // ── Configure trajectory parameters ──────────────────────────────────
    //
    //  All values from the Stage-1 specification.
    //  To change them at runtime, call traj.setParams(p) with a new Params.
    //
    //  stepLength  = 7.0 cm   → xMin = −3.5 cm, xMax = +3.5 cm
    //  stepHeight  = 2.0 cm   → maxLift at y = −13.0 cm
    //                           WARNING: knee hits ~93.7° at peak (clamped).
    //                           Reduce to 1.5 cm to stay within limits fully.
    //  groundY     = −15.0 cm
    //  homeRatio   = −2/3.5   → homeX = −2.0 cm  (near mid-point of range)
    //
    Trajectory::Params p = Trajectory::defaultParams();
    traj.setParams(p);

    // ── Print confirmed parameters ────────────────────────────────────────
    Serial.print(F("  stepLength  = ")); Serial.print(p.stepLength);  Serial.println(F(" cm"));
    Serial.print(F("  stepHeight  = ")); Serial.print(p.stepHeight);  Serial.println(F(" cm"));
    Serial.print(F("  groundY     = ")); Serial.print(p.groundY);     Serial.println(F(" cm"));
    Serial.print(F("  homeRatio   = ")); Serial.println(p.homeRatio, 4);
    Serial.print(F("  xMin        = ")); Serial.print(traj.getXMin()); Serial.println(F(" cm"));
    Serial.print(F("  xMax        = ")); Serial.print(traj.getXMax()); Serial.println(F(" cm"));
    Serial.print(F("  homeX       = ")); Serial.print(traj.getHomePosition().x); Serial.println(F(" cm"));
    Serial.print(F("  cyclePeriod = ")); Serial.print(GAIT_CYCLE_MS); Serial.println(F(" ms"));
    Serial.print(F("  updateRate  = ")); Serial.print(1000UL / UPDATE_INTERVAL_MS); Serial.println(F(" Hz"));
    Serial.println();

    // ── Move to home position ─────────────────────────────────────────────
    //
    //  Drive the leg directly to the home position.
    //  The servo moves at its hardware-limited natural speed.
    //  HOME_SETTLE_MS gives it time to physically arrive before gait starts.
    //
    FootPosition home = traj.getHomePosition();
    Serial.print(F("Moving to home  ("));
    Serial.print(home.x, 1); Serial.print(F(", "));
    Serial.print(home.y, 1); Serial.println(F(") cm  …"));

    leg.moveTo(home.x, home.y);
    delay(HOME_SETTLE_MS);

    Serial.println(F("Home reached.  Starting gait cycle."));
    Serial.println();

#ifdef SERIAL_PLOT
    // CSV header for Serial Plotter / monitor
    Serial.println(F("phi,x_cm,y_cm,hip_deg,knee_deg,ankle_deg"));
#endif
}

// =============================================================================
//  ── MAIN LOOP ───────────────────────────────────────────────────────────────
// =============================================================================
void loop() {
    // ── Non-blocking rate limiter ─────────────────────────────────────────
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();

    if (now - lastUpdate < UPDATE_INTERVAL_MS) return;
    lastUpdate = now;

    // ── Compute normalised phase φ ∈ [0, 1) ──────────────────────────────
    //
    //  millis() rolls over after ~49.7 days.
    //  The modulo operation gives clean, jitter-free phase wrapping without
    //  any explicit counter reset:
    //      φ = (millis() mod GAIT_CYCLE_MS) / GAIT_CYCLE_MS
    //
    float phi = static_cast<float>(now % GAIT_CYCLE_MS)
                / static_cast<float>(GAIT_CYCLE_MS);

    // ── Get foot target for this phase ───────────────────────────────────
    FootPosition fp = traj.getFootPosition(phi);

    // ── Drive leg  (IK + servo write happens inside moveTo) ──────────────
    leg.moveTo(fp.x, fp.y);

    // ── Telemetry output ─────────────────────────────────────────────────
#ifdef SERIAL_PLOT
    // Re-compute IK for logging (lightweight; ~0.4 ms on Uno at 16 MHz).
    // Remove this block — and the #define SERIAL_PLOT above — if Serial
    // output is not needed.
    IKResult ik = leg.calculateIK(fp.x, fp.y);

    Serial.print(phi, 3);
    Serial.print(F(","));
    Serial.print(fp.x, 2);
    Serial.print(F(","));
    Serial.print(fp.y, 2);
    Serial.print(F(","));
    Serial.print(ik.hip, 1);
    Serial.print(F(","));
    Serial.print(ik.knee, 1);
    Serial.print(F(","));
    Serial.println(ik.ankle, 1);
#endif
}
