/**
 * ESP32 Multicast Communication Example
 * 
 * Unlike broadcast (which stays in local subnet), multicast can work
 * across subnets if the network allows multicast routing.
 * 
 * This is similar to how ROS2 DDS works for auto-discovery!
 * 
 * Multicast group: 239.255.0.1 (default, like ROS2)
 */

#include <WiFi.h>
#include "capybarish_pubsub.h"
#include "motor_control_messages.hpp"

// Use the motor_control namespace for message types
using motor_control::SentData;
using motor_control::ReceivedData;
using motor_control::MotorData;
using motor_control::IMUData;

// =============================================================================
// WiFi Configuration
// =============================================================================

const char* WIFI_SSID = "Xenobot";
const char* WIFI_PASSWORD = "your_password_here";  // Update this!

// =============================================================================
// Multicast Configuration
// =============================================================================

// Multicast group address (same as ROS2 DDS default)
const char* MULTICAST_GROUP = "239.255.0.1";

// Ports
const uint16_t FEEDBACK_PORT = 6666;  // Port to send feedback to
const uint16_t COMMAND_PORT = 6667;   // Port to listen for commands on

// =============================================================================
// Publishers and Subscribers
// =============================================================================

cpy::Node* node = nullptr;
cpy::Publisher<SentData>* feedbackPub = nullptr;
cpy::Subscription<ReceivedData>* commandSub = nullptr;

// Latest command received
ReceivedData latestCommand;
bool hasNewCommand = false;

// Motor state (simulated)
float motorPos = 0.0f;
float motorVel = 0.0f;

// =============================================================================
// Forward Declarations
// =============================================================================

void onCommandReceived(const ReceivedData& cmd);
void controlLoop();
void printStats();

// =============================================================================
// Callback: Command Received
// =============================================================================

void onCommandReceived(const ReceivedData& cmd) {
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
    
    // Send feedback to multicast group
    SentData feedback;
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
    Serial.println("ESP32 Multicast Communication");
    Serial.println("Works across subnets (unlike broadcast)!");
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
    Serial.printf("[Node] Multicast group: %s\n", MULTICAST_GROUP);
    Serial.println();
    
    // Create node
    node = new cpy::Node("esp32_motor");
    
    // Create MULTICAST publisher - sends to multicast group!
    feedbackPub = node->createMulticastPublisher<SentData>(
        "/motor_feedback",
        FEEDBACK_PORT,
        MULTICAST_GROUP
    );
    
    // Create MULTICAST subscriber - joins multicast group to receive!
    commandSub = node->createMulticastSubscription<ReceivedData>(
        "/motor_cmd",
        onCommandReceived,
        COMMAND_PORT,
        MULTICAST_GROUP
    );
    
    Serial.println("[Node] Ready!");
    Serial.printf("[Node] Sending feedback to MULTICAST %s:%d\n", MULTICAST_GROUP, FEEDBACK_PORT);
    Serial.printf("[Node] Listening on MULTICAST %s:%d\n", MULTICAST_GROUP, COMMAND_PORT);
    Serial.println();
}

// =============================================================================
// Main Loop
// =============================================================================

// Rate limiting for publishing (100 Hz is more reasonable for WiFi)
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
