#pragma once
#include <Arduino.h>

class L298N {
    public:
        L298N(int pinEn1, int pinIn1, int pinIn2);
        void init();
        void setSpeed(int pwmVal);
        void stop();

    private:
        int _pinEn, _pinIn1, _pinIn2;
};