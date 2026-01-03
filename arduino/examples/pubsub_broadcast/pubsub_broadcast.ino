/**
 * @file pubsub_broadcast.ino
 * @brief ESP32 Pub/Sub with Broadcast - No IP Configuration Needed!
 * 
 * This example uses UDP broadcast so you don't need to know the server's IP.
 * Just like ROS2, messages are broadcast to all devices on the network.
 * 
 * Hardware: ESP32
 * 
 * How it works:
 * - ESP32 broadcasts feedback to 255.255.255.255:6666
 * - Any computer listening on port 6666 will receive it
 * - Computer broadcasts commands to 255.255.255.255:6667
 * - ESP32 listens on port 6667 for commands
 * 
 * @author Chen Yu <chenyu@u.northwestern.edu>
 * @copyright 2025 Chen Yu
 * @license Apache-2.0
 */

#include <Arduino.h>
#include "capybarish_pubsub.h"
#include "motor_control_messages.hpp"

using namespace motor_control;

// =============================================================================
// Configuration
// =============================================================================

// WiFi credentials - UPDATE THESE!
const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";

// Port configuration (no IP addresses needed!)
const uint16_t FEEDBACK_PORT = 6666;  // Port to broadcast feedback on
const uint16_t COMMAND_PORT = 6667;   // Port to listen for commands on

// Control rate
const float CONTROL_RATE_HZ = 100.0f;

// =============================================================================
// Global Variables
// =============================================================================

cpy::Node node("motor_module");

// Broadcast publisher - sends to ALL devices on network!
cpy::Publisher<SensorData>* feedbackPub = nullptr;

// Subscription - listens for commands from ANY device
cpy::Subscription<MotorCommand>* commandSub = nullptr;

// State
MotorData motorState = {};
IMUData imuState = {};
MotorCommand lastCommand = {};
bool hasNewCommand = false;

uint32_t loopCount = 0;
uint64_t lastStatsPrint = 0;

// =============================================================================
// Forward Declarations
// =============================================================================

void printStats();
void controlLoop();
void onCommandReceived(const MotorCommand& cmd);

// =============================================================================
// Callbacks
// =============================================================================

void onCommandReceived(const MotorCommand& cmd) {
    lastCommand = cmd;
    hasNewCommand = true;
}

void controlLoop() {
    // Simulate motor dynamics
    if (hasNewCommand) {
        float error = lastCommand.target - motorState.pos;
        float torque = lastCommand.kp * error - lastCommand.kd * motorState.vel;
        
        float dt = 1.0f / CONTROL_RATE_HZ;
        motorState.vel += torque * dt;
        motorState.pos += motorState.vel * dt;
        motorState.torque = torque;
        
        hasNewCommand = false;
    }
    
    // Build and broadcast feedback
    SensorData feedback = {};
    feedback.motor = motorState;
    feedback.imu = imuState;
    feedback.timestamp = micros() / 1000000.0f;
    
    if (feedbackPub) {
        feedbackPub->publish(feedback);  // Broadcasts to all!
    }
    
    loopCount++;
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("Capybarish Broadcast Example");
    Serial.println("No IP configuration needed!");
    Serial.println("========================================\n");
    
    // Connect to WiFi
    if (!node.initWiFi(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("Failed to connect to WiFi!");
        while (1) delay(1000);
    }
    
    // Create BROADCAST publisher - no IP address needed!
    // This sends to 255.255.255.255 (all devices on network)
    feedbackPub = node.createBroadcastPublisher<SensorData>(
        "/motor/feedback",
        FEEDBACK_PORT      // Any device listening on this port will receive
    );
    
    // Create subscription - listens on specific port
    commandSub = node.createSubscription<MotorCommand>(
        "/motor/command",
        onCommandReceived,
        COMMAND_PORT       // Listen for commands on this port
    );
    
    // Create control timer
    node.createTimer(1.0f / CONTROL_RATE_HZ, controlLoop);
    
    cpy::printTopics();
    
    Serial.println("\n[Setup] Complete!");
    Serial.printf("[Setup] Broadcasting feedback on port %d\n", FEEDBACK_PORT);
    Serial.printf("[Setup] Listening for commands on port %d\n", COMMAND_PORT);
    Serial.println();
    
    lastStatsPrint = millis();
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
    if (commandSub) {
        commandSub->spinAll();
    }
    
    node.spinOnce();
    
    if (millis() - lastStatsPrint >= 5000) {
        printStats();
        lastStatsPrint = millis();
    }
    
    delayMicroseconds(100);
}

// =============================================================================
// Helpers
// =============================================================================

void printStats() {
    Serial.println("\n--- Statistics ---");
    Serial.printf("Loop count: %lu\n", loopCount);
    
    if (commandSub) {
        Serial.printf("Commands received: %lu\n", commandSub->getReceiveCount());
    }
    
    if (feedbackPub) {
        Serial.printf("Feedback broadcast: %lu\n", feedbackPub->getPublishCount());
    }
    
    Serial.printf("Motor pos: %.3f, vel: %.3f\n", motorState.pos, motorState.vel);
    Serial.printf("Free heap: %lu bytes\n", ESP.getFreeHeap());
    Serial.println("------------------\n");
}
