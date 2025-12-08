/**
 * @file capybarish.cpp
 * @brief Implementation of non-template Capybarish functions
 * 
 * @author Chen Yu <chenyu@u.northwestern.edu>
 * @copyright 2025 Chen Yu
 * @license Apache-2.0
 */

#include "capybarish.h"

namespace Capybarish {

bool UDPCommRaw::begin(const char* ssid, const char* password,
                       const char* serverIP, uint16_t serverPort,
                       uint16_t localPort) {
    _config.ssid = ssid;
    _config.password = password;
    _config.serverIP = serverIP;
    _config.serverPort = serverPort;
    _config.localPort = localPort > 0 ? localPort : serverPort;
    
    return _connectWiFi() && _setupUDP();
}

bool UDPCommRaw::send(const uint8_t* data, size_t len) {
    if (_status != ConnectionStatus::CONNECTED) {
        return false;
    }
    
    _udp.beginPacket(_config.serverIP, _config.serverPort);
    _udp.write(data, len);
    return _udp.endPacket();
}

int UDPCommRaw::receive(uint8_t* buffer, size_t maxLen) {
    int packetSize = _udp.parsePacket();
    if (packetSize == 0) {
        return 0;
    }
    
    size_t readLen = min((size_t)packetSize, maxLen);
    _udp.read(buffer, readLen);
    
    // Flush remaining data if packet was larger than buffer
    while (_udp.available()) {
        _udp.read();
    }
    
    return readLen;
}

bool UDPCommRaw::isConnected() const {
    return WiFi.status() == WL_CONNECTED;
}

void UDPCommRaw::end() {
    _udp.stop();
    WiFi.disconnect();
    _status = ConnectionStatus::DISCONNECTED;
}

bool UDPCommRaw::_connectWiFi() {
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
    
    _status = ConnectionStatus::CONNECTED;
    return true;
}

bool UDPCommRaw::_setupUDP() {
    if (_udp.begin(_config.localPort)) {
        Serial.print("[Capybarish] UDP listening on port ");
        Serial.println(_config.localPort);
        return true;
    } else {
        Serial.println("[Capybarish] Failed to start UDP!");
        return false;
    }
}

} // namespace Capybarish
