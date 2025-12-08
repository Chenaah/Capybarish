/**
 * @file basic_comm.ino
 * @brief Basic example of Capybarish UDP communication
 * 
 * This example demonstrates:
 * - Defining custom message types
 * - Setting up WiFi and UDP communication
 * - Sending and receiving data
 * 
 * @author Chen Yu <chenyu@u.northwestern.edu>
 */

#include <capybarish.h>

// ============================================================================
// Configuration - Modify these for your setup
// ============================================================================

#define WIFI_SSID     "YourNetworkSSID"
#define WIFI_PASSWORD "YourNetworkPassword"
#define SERVER_IP     "192.168.1.100"
#define SERVER_PORT   6666

// ============================================================================
// Message Definitions
// ============================================================================

// Command message received from server
// Must match the Python SentDataStruct layout
struct ReceivedData {
    float target;
    float target_vel;
    float kp;
    float kd;
    int32_t enable_filter;
    int32_t switch_;
    int32_t calibrate;
    int32_t restart;
    float timestamp;
};

// Sensor data sent to server
struct SentData {
    int32_t module_id;
    int32_t receive_dt;
    int32_t timestamp;
    int32_t switch_off;
    float last_rcv_timestamp;
    int32_t info;
    
    // Motor data
    float motor_pos;
    float motor_large_pos;
    float motor_vel;
    float motor_torque;
    float motor_voltage;
    float motor_current;
    int32_t motor_temperature;
    int32_t motor_error0;
    int32_t motor_error1;
    
    // IMU orientation
    float imu_orientation_x;
    float imu_orientation_y;
    float imu_orientation_z;
    
    // IMU quaternion
    float imu_quat_x;
    float imu_quat_y;
    float imu_quat_z;
    float imu_quat_w;
    
    // IMU angular velocity
    float imu_omega_x;
    float imu_omega_y;
    float imu_omega_z;
    
    // IMU acceleration
    float imu_acc_x;
    float imu_acc_y;
    float imu_acc_z;
    
    // Error data
    int32_t reset_reason0;
    int32_t reset_reason1;
};

// ============================================================================
// Global Variables
// ============================================================================

// Create communication instance with our message types
Capybarish::UDPComm<ReceivedData, SentData> comm;

// Module ID (unique for each device)
const int MODULE_ID = 1;

// Timing
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 10;  // 100 Hz

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("========================================");
    Serial.println("Capybarish Basic Communication Example");
    Serial.println("========================================");
    Serial.printf("Receive message size: %d bytes\n", Capybarish::UDPComm<ReceivedData, SentData>::receiveSize());
    Serial.printf("Send message size: %d bytes\n", Capybarish::UDPComm<ReceivedData, SentData>::sendSize());
    Serial.println();
    
    // Initialize communication
    if (comm.begin(WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT)) {
        Serial.println("Communication initialized successfully!");
    } else {
        Serial.println("Failed to initialize communication!");
        while (1) {
            delay(1000);
        }
    }
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    // Check for incoming commands
    ReceivedData cmd;
    if (comm.receive(cmd)) {
        // Process received command
        Serial.printf("Received: target=%.2f, kp=%.2f, kd=%.2f\n", 
                      cmd.target, cmd.kp, cmd.kd);
        
        // Here you would apply the command to your motor controller
        // e.g., motor.setTarget(cmd.target);
    }
    
    // Send sensor data at regular intervals
    if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
        lastSendTime = millis();
        
        // Prepare sensor data
        SentData data = {};  // Zero initialize
        
        data.module_id = MODULE_ID;
        data.timestamp = micros();
        
        // Fill in your sensor readings here
        // Example: simulated data
        data.motor_pos = sin(millis() / 1000.0) * 3.14159;
        data.motor_vel = cos(millis() / 1000.0);
        data.motor_torque = 0.5;
        
        data.imu_orientation_x = 0.0;
        data.imu_orientation_y = 0.0;
        data.imu_orientation_z = 0.0;
        
        data.imu_quat_x = 0.0;
        data.imu_quat_y = 0.0;
        data.imu_quat_z = 0.0;
        data.imu_quat_w = 1.0;
        
        // Send data
        if (comm.send(data)) {
            // Successfully sent
        } else {
            Serial.println("Failed to send data!");
        }
    }
    
    // Small delay to prevent watchdog issues
    delay(1);
}
