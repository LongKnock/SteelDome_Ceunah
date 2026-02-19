#pragma once
#include <Arduino.h>

class SimplePID {
    public:
        SimplePID(float kp, float ki, float kd);
        float compute(float target, float current, float dt);
        void reset();

    private:
        float _kp, _ki, _kd;
        float _integral, _prevError;
};