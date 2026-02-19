#ifndef ENCODER_H
#define ENCODER_H

#include <AS5600.h>
#include <Wire.h>
#include "config.h"
#include "stepper.h"

// I2C Address
#define AS5600_ADDR 0x36

// LEFT ENCODER (I2C Bus 0)
#define RSDA_PIN 16
#define RSCL_PIN 17

// RIGHT ENCODER (I2C Bus 1)
#define LSDA_PIN 21
#define LSCL_PIN 22

extern TwoWire I2C_Left;
extern TwoWire I2C_Right;

extern AS5600 encLeft;
extern AS5600 encRight;

// Function Prototype
void setupEncoder();
void saveEncoderPosition();
void loadEncoderPosition();
void updateEncoderPosition(AS5600 &enc, MotorState &state);

#endif