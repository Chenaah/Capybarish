/**
 * @file pubsub_example.ino
 * @brief ESP32 Pub/Sub Example - ROS2-like Communication Pattern
 * 
 * This example demonstrates using capybarish's pub/sub system on ESP32,
 * which mirrors the Python API (capybarish.pubsub).
 * 
 * Hardware: ESP32
 * 
 * Setup:
 * 1. Generate message types: capybarish gen --cpp schemas/motor_control.cpy -o arduino/examples/pubsub_example/
 * 2. Copy this sketch to your PlatformIO project or Arduino IDE
 * 3. Update WiFi credentials and server IP
 * 
 * @author Chen Yu <chenyu@u.northwestern.edu>
 * @copyright 2025 Chen Yu
 * @license Apache-2.0
 */

#include <Arduino.h>
#include "capybarish_pubsub.h"

// Include generated message types
// Run: capybarish gen --cpp schemas/motor_control.cpy
#include "motor_control_messages.hpp"

using namespace motor_control;

// =============================================================================
// Forward Declarations
// =============================================================================

void printStats();
void controlLoop();
void onCommandReceived(const ReceivedData& cmd);

// =============================================================================
// Configuration
// =============================================================================

// WiFi credentials
const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";

// Server (Python) settings
const char* SERVER_IP = "192.168.1.100";  // Your server IP
const uint16_t SERVER_PORT = 6666;         // Port server listens on
const uint16_t LOCAL_PORT = 6666;          // Port ESP32 listens on

// Control rate
const float CONTROL_RATE_HZ = 100.0f;

// =============================================================================
// Global Variables
// =============================================================================

// Create a node (similar to ROS2 node)
cpy::Node node("motor_module");

// Publishers and Subscribers (will be initialized in setup)
cpy::Publisher<SentData>* feedbackPub = nullptr;
cpy::Subscription<ReceivedData>* commandSub = nullptr;

// State
MotorData motorState = {};
IMUData imuState = {};
ReceivedData lastCommand = {};
bool hasNewCommand = false;

// Statistics
uint32_t loopCount = 0;
uint64_t lastStatsPrint = 0;

// =============================================================================
// Callback Functions
// =============================================================================

/**
 * @brief Callback when command is received from server
 */
void onCommandReceived(const ReceivedData& cmd) {
    lastCommand = cmd;
    hasNewCommand = true;
    
    // Log received command (optional, can be noisy at high rates)
    // node.logInfo("Cmd: target=%.2f, kp=%.1f", cmd.target, cmd.kp);
}

/**
 * @brief Simulated motor control loop
 */
void controlLoop() {
    // Simulate motor dynamics
    if (hasNewCommand) {
        // Simple P controller simulation
        float error = lastCommand.target - motorState.pos;
        float torque = lastCommand.kp * error - lastCommand.kd * motorState.vel;
        
        // Integrate (simple Euler)
        float dt = 1.0f / CONTROL_RATE_HZ;
        motorState.vel += torque * dt;
        motorState.pos += motorState.vel * dt;
        motorState.torque = torque;
        
        hasNewCommand = false;
    }
    
    // Simulate IMU data
    imuState.orientation.x = sin(millis() / 1000.0f) * 0.1f;
    imuState.orientation.y = cos(millis() / 1000.0f) * 0.1f;
    imuState.orientation.z = 0.0f;
    
    // Build feedback message
    SentData feedback = {};
    feedback.motor = motorState;
    feedback.imu = imuState;
    feedback.timestamp = micros() / 1000000.0f;  // Convert to seconds
    
    // Publish feedback to server
    if (feedbackPub) {
        feedbackPub->publish(feedback);
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
    Serial.println("Capybarish Pub/Sub Example");
    Serial.println("========================================\n");
    
    // Initialize WiFi through node
    if (!node.initWiFi(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("Failed to connect to WiFi!");
        while (1) delay(1000);
    }
    
    // Create publisher for motor feedback
    // Similar to Python: pub = node.create_publisher(SentData, '/motor/feedback', ...)
    feedbackPub = node.createPublisher<SentData>(
        "/motor/feedback",  // Topic name
        SERVER_IP,          // Remote IP
        SERVER_PORT         // Remote port
    );
    
    // Create subscription for commands
    // Similar to Python: sub = node.create_subscription(ReceivedData, '/motor/command', callback, ...)
    commandSub = node.createSubscription<ReceivedData>(
        "/motor/command",   // Topic name
        onCommandReceived,  // Callback function
        LOCAL_PORT          // Local port to listen on
    );
    
    // Create control loop timer at 100 Hz
    // Similar to Python: timer = node.create_timer(0.01, control_loop)
    node.createTimer(1.0f / CONTROL_RATE_HZ, controlLoop);
    
    // Print topic registry
    cpy::printTopics();
    
    Serial.println("\n[Setup] Complete! Starting main loop...\n");
    
    lastStatsPrint = millis();
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
    // Process incoming messages (callbacks will be called)
    // Similar to Python: cpy.spin_once(node)
    if (commandSub) {
        commandSub->spinAll();  // Process all pending messages
    }
    
    // Process timers (control loop will be called at 100 Hz)
    node.spinOnce();
    
    // Print statistics every 5 seconds
    if (millis() - lastStatsPrint >= 5000) {
        printStats();
        lastStatsPrint = millis();
    }
    
    // Small delay to prevent watchdog issues
    // In a real application, use cpy::Rate for precise timing
    delayMicroseconds(100);
}

// =============================================================================
// Helper Functions
// =============================================================================

void printStats() {
    Serial.println("\n--- Statistics ---");
    Serial.printf("Loop count: %lu\n", loopCount);
    
    if (commandSub) {
        Serial.printf("Commands received: %lu\n", commandSub->getReceiveCount());
        Serial.printf("Commands dropped: %lu\n", commandSub->getDropCount());
    }
    
    if (feedbackPub) {
        Serial.printf("Feedback sent: %lu\n", feedbackPub->getPublishCount());
    }
    
    Serial.printf("Motor pos: %.3f, vel: %.3f\n", motorState.pos, motorState.vel);
    Serial.printf("Free heap: %lu bytes\n", ESP.getFreeHeap());
    Serial.println("------------------\n");
}
