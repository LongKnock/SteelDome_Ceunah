#include "L298N.h"

L298N::L298N(int pinEn, int pinIn1, int pinIn2) {
    _pinEn = pinEn;
    _pinIn1 = pinIn1;
    _pinIn2 = pinIn2;
}

void L298N::init() {
    pinMode(_pinEn, OUTPUT);
    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
}

void L298N::setSpeed(int pwmVal) {
    int pwm = constrain(pwmVal, -255, 255);

    if (pwm >= 0) {
        digitalWrite(_pinIn1, HIGH);
        digitalWrite(_pinIn2, LOW);
    }
    else {
        digitalWrite(_pinIn1, LOW);
        digitalWrite(_pinIn2, HIGH);
        pwm = -pwm;
    }

    analogWrite(_pinEn, pwm);
}

void L298N::stop() {
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);
    analogWrite(_pinEn, 0);
}