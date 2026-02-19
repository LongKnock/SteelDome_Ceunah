#include "encoder.h"

TwoWire I2C_Left = TwoWire(0);
TwoWire I2C_Right = TwoWire(1);

AS5600 encLeft(&I2C_Left);
AS5600 encRight(&I2C_Right);

void setupEncoder() {
    // Begin I2C
    I2C_Left.begin(LSDA_PIN, LSCL_PIN, 400000);
    I2C_Right.begin(RSDA_PIN, RSCL_PIN, 400000);

    // Begin Encoder
    encLeft.begin();
    encRight.begin();

    // Set Encoder Direction
    encLeft.setDirection(AS5600_COUNTERCLOCK_WISE);
    encRight.setDirection(AS5600_CLOCK_WISE);

    // Sync positions
    if (encLeft.isConnected()) stateL.lastPos = encLeft.rawAngle() * AS5600_RAW_TO_DEGREES;
    if (encRight.isConnected()) stateR.lastPos = encRight.rawAngle() * AS5600_RAW_TO_DEGREES;
}

void saveEncoderPosition() {
    if (stepperLeft.distanceToGo() != 0 || stepperRight.distanceToGo() != 0) {
        return; 
    }

    preferences.begin("motor_data", false);

    if (fabs(stateL.currentPos - lastSavedPosL) > 0.5) {
        preferences.putDouble("posL", stateL.currentPos);
        lastSavedPosL = stateL.currentPos;
    }

    if (fabs(stateR.currentPos - lastSavedPosR) > 0.5) {
        preferences.putDouble("posR", stateR.currentPos);
        lastSavedPosR = stateR.currentPos;
    }

    preferences.end();
}

void loadEncoderPosition() {
    preferences.begin("motor_data", true); // Read-only mode
    
    // Get double, default to 0.0 if key doesn't exist
    stateL.currentPos = (long)preferences.getDouble("posL", 0.0);
    stateR.currentPos = (long)preferences.getDouble("posR", 0.0);
    
    stepperLeft.setCurrentPosition(stateL.currentPos);
    stepperRight.setCurrentPosition(stateR.currentPos);

    // Sync tracking variables
    lastSavedPosL = stateL.currentPos;
    lastSavedPosR = stateR.currentPos;
    
    preferences.end();
}

void updateEncoderPosition(AS5600 &enc, MotorState &state) {
    double degAngle = enc.rawAngle() * AS5600_RAW_TO_DEGREES;
    double deltaAngle = degAngle - state.lastPos;

    if (enc.getDirection() == AS5600_CLOCK_WISE) {
        deltaAngle = -deltaAngle;
    }
    
    if (deltaAngle > 180.0) deltaAngle -= 360.0;
    if (deltaAngle < -180.0) deltaAngle += 360.0;

    if (enc.getDirection() == AS5600_CLOCK_WISE) {
        deltaAngle = -deltaAngle;
    }
    state.currentPos += deltaAngle;
    state.lastPos = degAngle;
}