/**
 * ESP32 Client - Sends to known PC, PC replies back
 * 
 * This is the most practical pattern:
 * - ESP32 knows PC's IP (configured below)
 * - ESP32 sends feedback TO the PC
 * - PC doesn't need to know ESP32's IP - it replies to sender
 * 
 * Multiple ESP32s can connect to one PC without any PC configuration!
 * 
 * Configure SERVER_IP to your computer's IP address.
 */

#include <WiFi.h>
#include "capybarish_pubsub.h"
#include "motor_control_messages.hpp"

// Use the motor_control namespace for message types
using motor_control::SensorData;
using motor_control::MotorCommand;
using motor_control::MotorData;
using motor_control::IMUData;

// =============================================================================
// WiFi Configuration
// =============================================================================

const char* WIFI_SSID = "Xenobot";
const char* WIFI_PASSWORD = "your_password_here";  // Update this!

// =============================================================================
// Network Configuration - UPDATE THESE!
// =============================================================================

// Your computer's IP address (where Python script runs)
const char* SERVER_IP = "129.105.69.100";

// Ports
const uint16_t FEEDBACK_PORT = 6666;  // Port to send feedback to (Python listens)
const uint16_t COMMAND_PORT = 6667;   // Port to receive commands on

// =============================================================================
// Publishers and Subscribers
// =============================================================================

cpy::Node* node = nullptr;
cpy::Publisher<SensorData>* feedbackPub = nullptr;
cpy::Subscription<MotorCommand>* commandSub = nullptr;

// Latest command received
MotorCommand latestCommand;
bool hasNewCommand = false;

// Motor state (simulated)
float motorPos = 0.0f;
float motorVel = 0.0f;

// =============================================================================
// Forward Declarations
// =============================================================================

void onCommandReceived(const MotorCommand& cmd);
void controlLoop();
void printStats();

// =============================================================================
// Callback: Command Received
// =============================================================================

void onCommandReceived(const MotorCommand& cmd) {
    latestCommand = cmd;
    hasNewCommand = true;
}

// =============================================================================
// Control Loop
// =============================================================================

void controlLoop() {
    if (hasNewCommand) {
        // Simple P controller simulation
        float error = latestCommand.target - motorPos;
        motorVel = latestCommand.kp * error;
        motorPos += motorVel * 0.001f;  // dt = 1ms
        
        hasNewCommand = false;
    }
    
    // Send feedback
    SensorData feedback;
    feedback.motor.pos = motorPos;
    feedback.motor.vel = motorVel;
    feedback.motor.torque = 0.0f;
    feedback.imu.quaternion.w = 1.0f;
    feedback.imu.quaternion.x = 0.0f;
    feedback.imu.quaternion.y = 0.0f;
    feedback.imu.quaternion.z = 0.0f;
    feedback.timestamp = millis();
    
    feedbackPub->publish(feedback);
}

// =============================================================================
// Stats Printing
// =============================================================================

unsigned long lastStats = 0;

void printStats() {
    if (millis() - lastStats >= 1000) {
        Serial.printf("[Stats] Pos: %.3f | Vel: %.3f | Target: %.3f | Tx: %lu | Rx: %lu\n",
                      motorPos, motorVel, latestCommand.target,
                      feedbackPub->getPublishCount(),
                      commandSub->getReceiveCount());
        lastStats = millis();
    }
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println();
    Serial.println("=========================================");
    Serial.println("ESP32 Direct IP Communication");
    Serial.println("=========================================");
    
    // Connect to WiFi
    Serial.printf("[Node] Connecting to WiFi '%s'...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.printf("[Node] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[Node] Server IP: %s\n", SERVER_IP);
    Serial.println();
    
    // Create node
    node = new cpy::Node("esp32_motor");
    
    // Create publisher - sends feedback DIRECTLY to server
    feedbackPub = node->createPublisher<SensorData>(
        "/motor_feedback",
        SERVER_IP,       // Direct IP, not broadcast!
        FEEDBACK_PORT
    );
    
    // Create subscriber - listens for commands
    // Signature: createSubscription<T>(topic, callback, port)
    commandSub = node->createSubscription<MotorCommand>(
        "/motor_cmd",
        onCommandReceived,
        COMMAND_PORT
    );
    
    Serial.println("[Node] Ready!");
    Serial.printf("[Node] Sending feedback to %s:%d\n", SERVER_IP, FEEDBACK_PORT);
    Serial.printf("[Node] Listening for commands on port %d\n", COMMAND_PORT);
    Serial.println();
}

// =============================================================================
// Main Loop
// =============================================================================

// Rate limiting for publishing (100 Hz is reasonable for WiFi)
// Increase to 200-500 Hz if needed, but watch for errors
const float PUBLISH_RATE_HZ = 100.0f;
const unsigned long PUBLISH_PERIOD_US = 1000000 / PUBLISH_RATE_HZ;
unsigned long lastPublishTime = 0;

void loop() {
    unsigned long now = micros();
    
    // Process incoming messages (always, as fast as possible)
    node->spinOnce();
    commandSub->spinOnce();
    
    // Rate-limited publishing to avoid overwhelming WiFi
    if (now - lastPublishTime >= PUBLISH_PERIOD_US) {
        controlLoop();
        lastPublishTime = now;
    }
    
    // Print stats
    printStats();
    
    // Small delay to yield to WiFi task
    delay(1);
}
