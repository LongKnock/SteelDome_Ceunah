#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ESP32_Servo.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <cmath>

#define LOOP_FREQUENCY 200
#define LOOP_TIME_US (1000000 / LOOP_FREQUENCY)

#define SAVE_INTERVAL_MS 5000 // Auto-save every 5 seconds

#define LED_PIN 2
#define MAX_STRING_LEN 50

// --- PERSISTENT MEMORY ---
extern Preferences preferences;
extern unsigned long lastSaveTime;
extern double lastSavedPosL;
extern double lastSavedPosR;

extern double stepperLeftPos;
extern double stepperRightPos;

struct MotorState {
    // Position
    double currentPos;
    double lastPos;

    double targetPos;
    double lastTargetPos;

    // Velocity
    double currentVel;
    double lastVel;

    double targetVel;
    double lastTargetVel;
};

extern MotorState stateL;
extern MotorState stateR;

extern Servo myServo;

extern bool blinking;
extern int servo_angle;

#endif