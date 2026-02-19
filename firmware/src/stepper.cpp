#include "stepper.h"

AccelStepper stepperLeft(1, SL_PUL, SL_DIR);
AccelStepper stepperRight(1, SR_PUL, SR_DIR);

void setupStepper() {
    stepperLeft.setMaxSpeed(stepperMaxSpeed);
    stepperRight.setMaxSpeed(stepperMaxSpeed);
    stepperLeft.setAcceleration(stepperMaxAccel);
    stepperRight.setAcceleration(stepperMaxAccel);
}

void runStepper() {
    if (stateL.targetPos != stateL.lastTargetPos) {
        long targetStepsL = (long)(pulsePerRev * (stateL.targetPos / 360.0));
        stepperLeft.moveTo(targetStepsL);
        stateL.lastTargetPos = stateL.targetPos;
    }

    if (stateR.targetPos != stateR.lastTargetPos) {
        long targetStepsR = -(long)(pulsePerRev * (stateR.targetPos / 360.0));
        stepperRight.moveTo(targetStepsR); 
        stateR.lastTargetPos = stateR.targetPos;
    }

    stepperLeft.run();
    stepperRight.run();

    stepperLeftPos = stepperLeft.currentPosition() * (360.0 / pulsePerRev);
    stepperRightPos = stepperRight.currentPosition() * (360.0 / pulsePerRev);

}