/**
 * @file capybarish.h
 * @brief Capybarish - Lightweight communication middleware for ESP32 robots
 * 
 * This library provides UDP-based real-time communication for modular robotics,
 * with support for custom message types and automatic serialization.
 * 
 * @author Chen Yu <chenyu@u.northwestern.edu>
 * @copyright 2025 Chen Yu
 * @license Apache-2.0
 * 
 * @see https://github.com/chenaah/capybarish
 */

#pragma once

#ifndef CAPYBARISH_H
#define CAPYBARISH_H

#include "Arduino.h"

#ifdef ESP32
    #include <WiFi.h>
    #include "esp_wifi.h"
#elif defined(ESP8266)
    #include <ESP8266WiFi.h>
#endif

#include <WiFiUdp.h>
#include <cstring>

namespace Capybarish {

/**
 * @brief Configuration for UDP communication
 */
struct Config {
    const char* ssid = nullptr;
    const char* password = nullptr;
    const char* serverIP = nullptr;
    uint16_t serverPort = 6666;
    uint16_t localPort = 6666;
    uint32_t connectionTimeout = 30000;  // ms
    uint32_t receiveTimeout = 100;       // ms
    bool autoReconnect = true;
};

/**
 * @brief Connection status enumeration
 */
enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    CONNECTION_LOST
};

/**
 * @brief Statistics for communication monitoring
 */
struct Stats {
    uint32_t packetsSent = 0;
    uint32_t packetsReceived = 0;
    uint32_t sendErrors = 0;
    uint32_t receiveErrors = 0;
    uint64_t lastSendTime = 0;
    uint64_t lastReceiveTime = 0;
    uint32_t roundTripTime = 0;  // microseconds
};

/**
 * @brief Template wrapper for message serialization
 * 
 * Provides helper methods for serializing/deserializing POD structs.
 * 
 * @tparam T The message struct type (must be POD)
 */
template<typename T>
class Message {
public:
    T data;
    
    Message() = default;
    Message(const T& d) : data(d) {}
    
    /**
     * @brief Get the serialized size of the message
     */
    static constexpr size_t size() {
        return sizeof(T);
    }
    
    /**
     * @brief Serialize the message to a byte buffer
     * @param buffer Output buffer (must be at least size() bytes)
     */
    void serialize(uint8_t* buffer) const {
        memcpy(buffer, &data, sizeof(T));
    }
    
    /**
     * @brief Deserialize the message from a byte buffer
     * @param buffer Input buffer
     * @param len Buffer length
     * @return true if deserialization successful
     */
    bool deserialize(const uint8_t* buffer, size_t len) {
        if (len < sizeof(T)) {
            return false;
        }
        memcpy(&data, buffer, sizeof(T));
        return true;
    }
    
    /**
     * @brief Get pointer to raw data
     */
    const uint8_t* raw() const {
        return reinterpret_cast<const uint8_t*>(&data);
    }
    
    /**
     * @brief Get mutable pointer to raw data
     */
    uint8_t* raw() {
        return reinterpret_cast<uint8_t*>(&data);
    }
};

/**
 * @brief UDP Communication class for ESP32
 * 
 * Template-based UDP communication that handles WiFi connection,
 * UDP socket management, and message serialization.
 * 
 * @tparam TReceive Type of messages to receive (must be POD struct)
 * @tparam TSend Type of messages to send (must be POD struct)
 * 
 * @example
 * @code
 * struct Command { float target; int mode; };
 * struct Status { float position; int error; };
 * 
 * Capybarish::UDPComm<Command, Status> comm;
 * comm.begin("SSID", "password", "192.168.1.100", 6666);
 * 
 * Command cmd;
 * if (comm.receive(cmd)) {
 *     // Process command
 * }
 * 
 * Status status = {1.5f, 0};
 * comm.send(status);
 * @endcode
 */
template<typename TReceive, typename TSend>
class UDPComm {
public:
    UDPComm() : _status(ConnectionStatus::DISCONNECTED) {}
    
    /**
     * @brief Initialize WiFi and UDP communication
     * 
     * @param ssid WiFi network name
     * @param password WiFi password
     * @param serverIP Server IP address
     * @param serverPort Server UDP port
     * @param localPort Local UDP port (default: same as server)
     * @return true if connection successful
     */
    bool begin(const char* ssid, const char* password, 
               const char* serverIP, uint16_t serverPort,
               uint16_t localPort = 0) {
        _config.ssid = ssid;
        _config.password = password;
        _config.serverIP = serverIP;
        _config.serverPort = serverPort;
        _config.localPort = localPort > 0 ? localPort : serverPort;
        
        return _connectWiFi() && _setupUDP();
    }
    
    /**
     * @brief Initialize with config struct
     */
    bool begin(const Config& config) {
        _config = config;
        return _connectWiFi() && _setupUDP();
    }
    
    /**
     * @brief Receive data from server
     * 
     * @param data Output struct to fill with received data
     * @return true if data was received
     */
    bool receive(TReceive& data) {
        int packetSize = _udp.parsePacket();
        if (packetSize == 0) {
            return false;
        }
        
        if (packetSize != sizeof(TReceive)) {
            _stats.receiveErrors++;
            // Flush the packet
            while (_udp.available()) {
                _udp.read();
            }
            return false;
        }
        
        uint8_t buffer[sizeof(TReceive)];
        _udp.read(buffer, sizeof(TReceive));
        memcpy(&data, buffer, sizeof(TReceive));
        
        _stats.packetsReceived++;
        _stats.lastReceiveTime = micros();
        
        return true;
    }
    
    /**
     * @brief Send data to server
     * 
     * @param data Struct to send
     * @return true if send successful
     */
    bool send(const TSend& data) {
        if (_status != ConnectionStatus::CONNECTED) {
            _stats.sendErrors++;
            return false;
        }
        
        uint64_t startTime = micros();
        
        _udp.beginPacket(_config.serverIP, _config.serverPort);
        _udp.write(reinterpret_cast<const uint8_t*>(&data), sizeof(TSend));
        
        if (_udp.endPacket()) {
            _stats.packetsSent++;
            _stats.lastSendTime = startTime;
            return true;
        } else {
            _stats.sendErrors++;
            return false;
        }
    }
    
    /**
     * @brief Check if connected to WiFi
     */
    bool isConnected() const {
        return WiFi.status() == WL_CONNECTED;
    }
    
    /**
     * @brief Get current connection status
     */
    ConnectionStatus getStatus() const {
        return _status;
    }
    
    /**
     * @brief Get local IP address
     */
    IPAddress getLocalIP() const {
        return WiFi.localIP();
    }
    
    /**
     * @brief Get MAC address
     */
    String getMacAddress() const {
        return WiFi.macAddress();
    }
    
    /**
     * @brief Get communication statistics
     */
    const Stats& getStats() const {
        return _stats;
    }
    
    /**
     * @brief Reset statistics
     */
    void resetStats() {
        _stats = Stats();
    }
    
    /**
     * @brief Update connection state (call in loop for auto-reconnect)
     */
    void update() {
        if (_config.autoReconnect && !isConnected()) {
            _status = ConnectionStatus::CONNECTION_LOST;
            _connectWiFi();
        }
    }
    
    /**
     * @brief Close connection
     */
    void end() {
        _udp.stop();
        WiFi.disconnect();
        _status = ConnectionStatus::DISCONNECTED;
    }
    
    /**
     * @brief Get size of receive message type
     */
    static constexpr size_t receiveSize() {
        return sizeof(TReceive);
    }
    
    /**
     * @brief Get size of send message type
     */
    static constexpr size_t sendSize() {
        return sizeof(TSend);
    }

private:
    Config _config;
    WiFiUDP _udp;
    ConnectionStatus _status;
    Stats _stats;
    
    bool _connectWiFi() {
        _status = ConnectionStatus::CONNECTING;
        
        Serial.println("[Capybarish] Connecting to WiFi...");
        
        #ifdef ESP32
        WiFi.setSleep(false);
        #endif
        WiFi.disconnect(true);
        WiFi.begin(_config.ssid, _config.password);
        
        if (_config.autoReconnect) {
            WiFi.setAutoReconnect(true);
        }
        
        uint32_t startTime = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - startTime > _config.connectionTimeout) {
                Serial.println("[Capybarish] WiFi connection timeout!");
                _status = ConnectionStatus::DISCONNECTED;
                return false;
            }
            delay(100);
            Serial.print(".");
        }
        
        Serial.println();
        Serial.print("[Capybarish] Connected! IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("[Capybarish] MAC: ");
        Serial.println(WiFi.macAddress());
        
        _status = ConnectionStatus::CONNECTED;
        return true;
    }
    
    bool _setupUDP() {
        if (_udp.begin(_config.localPort)) {
            Serial.print("[Capybarish] UDP listening on port ");
            Serial.println(_config.localPort);
            Serial.print("[Capybarish] Server: ");
            Serial.print(_config.serverIP);
            Serial.print(":");
            Serial.println(_config.serverPort);
            return true;
        } else {
            Serial.println("[Capybarish] Failed to start UDP!");
            return false;
        }
    }
};

/**
 * @brief Low-level UDP communication (non-template version)
 * 
 * For advanced users who need direct buffer access.
 */
class UDPCommRaw {
public:
    bool begin(const char* ssid, const char* password,
               const char* serverIP, uint16_t serverPort,
               uint16_t localPort = 0);
    
    bool send(const uint8_t* data, size_t len);
    int receive(uint8_t* buffer, size_t maxLen);
    
    bool isConnected() const;
    void end();
    
private:
    Config _config;
    WiFiUDP _udp;
    ConnectionStatus _status = ConnectionStatus::DISCONNECTED;
    
    bool _connectWiFi();
    bool _setupUDP();
};

} // namespace Capybarish

#endif // CAPYBARISH_H
