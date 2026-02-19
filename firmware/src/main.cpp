#include "config.h"
#include "ros_node.h"
#include "stepper.h"
#include "encoder.h"

const int SERVO_PIN = 13;
Servo myServo;
int servo_angle = 0;

Preferences preferences;
unsigned long lastSaveTime = 0;
double lastSavedPosL = 0;
double lastSavedPosR = 0;

double stepperLeftPos = 0;
double stepperRightPos = 0;

bool blinking = false;

MotorState stateL = {
    0, 0, 0, -9999.0,
    0, 0, 0, 0,
};

MotorState stateR = {
    0, 0, 0, -9999.0,
    0, 0, 0, 0,
};

TaskHandle_t rosTaskHandle;

// Function Prototypes
void rosTask(void * pvParameters);

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    myServo.attach(SERVO_PIN);
    
    setupEncoder();
    setupStepper();
    loadEncoderPosition();
    
    // Create ROS Thread on Core 0
    xTaskCreatePinnedToCore(
        rosTask,
        "ROS_Task",
        10000,
        NULL,
        1,
        &rosTaskHandle,
        0
    );
}

// --- MAIN LOOP (Core 1) ---
void loop() {
    runStepper();

    updateEncoderPosition(encLeft, stateL);
    updateEncoderPosition(encRight, stateR);
}

// --- ROS THREAD (Core 0) ---
void rosTask(void * pvParameters) {
    // Setup Micro-ROS
    setup_ros_node();

    // Infinite Loop
    while (true) {
        spin_ros_node();
        
        // Saving Position
        if (millis() - lastSaveTime > SAVE_INTERVAL_MS) {
            saveEncoderPosition();
            lastSaveTime = millis();
        }
        // Yield to allow WiFi/System tasks to run
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}