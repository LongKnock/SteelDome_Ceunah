#ifndef STEPPER_H
#define STEPPER_H

#include <AccelStepper.h>
#include "config.h"

// Stepper Left Pin
#define SL_ENA 32
#define SL_DIR 33
#define SL_PUL 25

// Stepper Right Pin
#define SR_ENA 14
#define SR_DIR 27
#define SR_PUL 26

extern AccelStepper stepperLeft;
extern AccelStepper stepperRight;

// Stepper Parameter
const float pulsePerRev = 6400.0;
const float stepperMaxSpeed = 12800.0;
const float stepperMaxAccel = 6400.0;

// Function Prototype
void setupStepper();
void runStepper();

#endif