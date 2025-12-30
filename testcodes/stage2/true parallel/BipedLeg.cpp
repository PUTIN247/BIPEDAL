#include "BipedLeg.h"

BipedLeg::BipedLeg(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh, bool isRightLeg) {
    _pwm = driver;
    _hipCh = hipCh; _kneeCh = kneeCh; _ankleCh = ankleCh;
    _isRightLeg = isRightLeg;
    
    // Default safe starting position
    _currX = 0; 
    _currY = -14.0;
    _isMoving = false;

    _hipOffset = 0; _kneeOffset = 0; _ankleOffset = 0;
}

void BipedLeg::setOffsets(float hipOff, float kneeOff, float ankleOff) {
    _hipOffset = hipOff;
    _kneeOffset = kneeOff;
    _ankleOffset = ankleOff;
}

// --- NEW: Non-Blocking Target Setter ---
void BipedLeg::setTarget(float targetX, float targetY, unsigned long duration_ms) {
    _startX = _currX;
    _startY = _currY;
    _targetX = targetX;
    _targetY = targetY;
    _moveDuration = duration_ms;
    _moveStartTime = millis();
    _isMoving = true;
}

// --- NEW: The Time-Based Mover ---
bool BipedLeg::update() {
    if (!_isMoving) return true; // Already there

    unsigned long now = millis();
    unsigned long elapsed = now - _moveStartTime;

    // 1. Calculate Progress (0.0 to 1.0)
    float progress = (float)elapsed / (float)_moveDuration;
    
    if (progress >= 1.0) {
        progress = 1.0;
        _isMoving = false; // Move complete
    }

    // 2. Interpolate (Linear Path)
    // This creates the straight line motion you wanted
    float nextX = _startX + (_targetX - _startX) * progress;
    float nextY = _startY + (_targetY - _startY) * progress;

    // 3. Solve Inverse Kinematics
    float h, k, a;
    if (calculateIK(nextX, nextY, h, k, a)) {
        // Apply Constraints & Offsets
        // "90.0" centers the servo range as discussed
        setServoRaw(_hipCh, 90.0 + constrain(h, HIP_MIN, HIP_MAX) + _hipOffset);
        setServoRaw(_kneeCh, 90.0 + constrain(k, KNEE_MIN, KNEE_MAX) + _kneeOffset);
        setServoRaw(_ankleCh, 90.0 + constrain(a, ANKLE_MIN, ANKLE_MAX) + _ankleOffset);
        
        // Update current position memory
        _currX = nextX;
        _currY = nextY;
    }

    return !_isMoving; // Return true if finished
}

// Instant move (for startup only)
void BipedLeg::setPositionSingle(float x, float y) {
    setTarget(x, y, 0); // 0ms duration = instant
    update();
}

void BipedLeg::setServoRaw(int channel, float angle) {
    int a = constrain((int)angle, 0, 180);
    int pulse = map(a, 0, 180, SERVOMIN, SERVOMAX);
    _pwm->setPWM(channel, 0, pulse);
}

// --- ZIG-ZAG INVERSE KINEMATICS (Unchanged from Stage 1) ---
bool BipedLeg::calculateIK(float x, float y, float &tHip, float &tKnee, float &tAnkle) {
    float dist = sqrt(x*x + y*y);

    // Safety: Reach Check
    if (dist > (L_FEMUR + L_TIBIA - 0.5) || dist < 4.0) return false;

    // Knee (Law of Cosines)
    float cosGamma = (L_FEMUR*L_FEMUR + L_TIBIA*L_TIBIA - dist*dist) / (2 * L_FEMUR * L_TIBIA);
    float gamma = acos(constrain(cosGamma, -1.0, 1.0));
    float kneeRad = PI - gamma; // Deviation from straight

    // Hip
    float alpha = atan2(x, y);
    float cosBeta = (L_FEMUR*L_FEMUR + dist*dist - L_TIBIA*L_TIBIA) / (2 * L_FEMUR * dist);
    float beta = acos(constrain(cosBeta, -1.0, 1.0));
    
    [cite_start]// Zig-Zag Logic: Subtract beta to force knee backward [cite: 53-55]
    float hipRad = alpha - beta; 

    // Ankle (Auto-Level)
    float ankleRad = -(hipRad + kneeRad);

    // Convert to degrees
    tHip = hipRad * 180.0 / PI;
    tKnee = kneeRad * 180.0 / PI;
    tAnkle = ankleRad * 180.0 / PI;

    return true;
}