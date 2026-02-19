#include "SimplePID.h"

SimplePID::SimplePID(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0;
    _prevError = 0;
}

float SimplePID::compute(float target, float current, float dt) {
    float error = target - current;

    // Proportional
    float P = _kp * error;
    
    // Integral
    _integral += error * dt;
    float I = _ki * _integral;

    // Derivative
    float D = _kd * ((error - _prevError) / dt);
    _prevError = error;

    // Output
    float output = P + I + D;

    // Clamp output
    if (output > 255) output = 255;
    if (output < -255) output = -255;

    return output;
}

void SimplePID::reset() {
    _integral = 0;
    _prevError = 0;
}