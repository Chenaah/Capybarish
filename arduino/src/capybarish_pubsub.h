/**
 * @file capybarish_pubsub.h
 * @brief ROS2-like Pub/Sub system for ESP32
 * 
 * This header provides a ROS2-style API for pub/sub communication
 * on ESP32, compatible with the Python capybarish.pubsub module.
 * 
 * @author Chen Yu <chenyu@u.northwestern.edu>
 * @copyright 2025 Chen Yu
 * @license Apache-2.0
 */

#pragma once

#ifndef CAPYBARISH_PUBSUB_H
#define CAPYBARISH_PUBSUB_H

#include "Arduino.h"

#ifdef ESP32
    #include <WiFi.h>
    #include "esp_wifi.h"
    #include "lwip/igmp.h"
    #include "lwip/ip_addr.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "freertos/semphr.h"
#elif defined(ESP8266)
    #include <ESP8266WiFi.h>
#endif

#include <WiFiUdp.h>
#include <functional>
#include <cstring>
#include <vector>

namespace cpy {

// Forward declarations
template<typename T> class Publisher;
template<typename T> class Subscription;
class Timer;
class Node;

// =============================================================================
// QoS Configuration
// =============================================================================

/**
 * @brief Reliability policy for message delivery
 */
enum class QoSReliability {
    RELIABLE,      ///< Guarantee delivery (not implemented over UDP)
    BEST_EFFORT    ///< No guarantee, lowest latency
};

/**
 * @brief History policy for message queue
 */
enum class QoSHistory {
    KEEP_LAST,     ///< Keep last N messages
    KEEP_ALL       ///< Keep all messages (bounded by depth)
};

/**
 * @brief Quality of Service profile
 */
struct QoSProfile {
    QoSReliability reliability = QoSReliability::BEST_EFFORT;
    QoSHistory history = QoSHistory::KEEP_LAST;
    uint8_t depth = 10;
    
    static QoSProfile sensorData() {
        return QoSProfile{QoSReliability::BEST_EFFORT, QoSHistory::KEEP_LAST, 5};
    }
    
    static QoSProfile defaultProfile() {
        return QoSProfile{QoSReliability::BEST_EFFORT, QoSHistory::KEEP_LAST, 10};
    }
};

// =============================================================================
// Topic Registry
// =============================================================================

/**
 * @brief Topic information entry
 */
struct TopicInfo {
    const char* name;
    uint16_t port;
    size_t msgSize;
    bool isPublisher;  // true = we publish, false = we subscribe
};

/**
 * @brief Global topic registry singleton
 */
class TopicRegistry {
public:
    static TopicRegistry& instance() {
        static TopicRegistry inst;
        return inst;
    }
    
    /**
     * @brief Register a topic with its port mapping
     */
    bool registerTopic(const char* name, uint16_t port, size_t msgSize, bool isPublisher) {
        if (_numTopics >= MAX_TOPICS) return false;
        
        // Check if already registered
        for (size_t i = 0; i < _numTopics; i++) {
            if (strcmp(_topics[i].name, name) == 0) {
                return true;  // Already exists
            }
        }
        
        _topics[_numTopics++] = {name, port, msgSize, isPublisher};
        return true;
    }
    
    /**
     * @brief Get port for a topic name
     */
    uint16_t getPort(const char* name) const {
        for (size_t i = 0; i < _numTopics; i++) {
            if (strcmp(_topics[i].name, name) == 0) {
                return _topics[i].port;
            }
        }
        return 0;  // Not found
    }
    
    /**
     * @brief Auto-assign port based on topic name hash
     */
    static uint16_t autoPort(const char* name, uint16_t basePort = 7000) {
        uint32_t hash = 0;
        while (*name) {
            hash = hash * 31 + *name++;
        }
        return basePort + (hash % 1000);  // Ports 7000-7999
    }
    
    /**
     * @brief Print all registered topics
     */
    void printTopics() const {
        Serial.println("[TopicRegistry] Registered topics:");
        for (size_t i = 0; i < _numTopics; i++) {
            Serial.printf("  %s -> port %d (%s, %d bytes)\n",
                _topics[i].name,
                _topics[i].port,
                _topics[i].isPublisher ? "pub" : "sub",
                _topics[i].msgSize
            );
        }
    }
    
private:
    TopicRegistry() = default;
    
    static constexpr size_t MAX_TOPICS = 32;
    TopicInfo _topics[MAX_TOPICS];
    size_t _numTopics = 0;
};

// =============================================================================
// Publisher
// =============================================================================

/**
 * @brief Publishes messages to a topic
 * 
 * @tparam T Message type (POD struct with serialize() method)
 * 
 * Supports three modes:
 * - Unicast: Send to specific IP (default)
 * - Broadcast: Send to 255.255.255.255 (all devices on subnet)
 * - Multicast: Send to multicast group (e.g., 239.255.0.1)
 * 
 * @example
 * @code
 * // Unicast mode (specific IP)
 * auto pub = node.createPublisher<MotorCommand>("/motor/command", serverIP, 6666);
 * 
 * // Broadcast mode (auto-discovery)
 * auto pub = node.createBroadcastPublisher<MotorCommand>("/motor/command", 6666);
 * 
 * MotorCommand msg = {1.5f, 0.0f, 10.0f, 0.5f};
 * pub->publish(msg);
 * @endcode
 */
template<typename T>
class Publisher {
public:
    Publisher(const char* topicName, const char* remoteIP, uint16_t remotePort,
              uint16_t localPort = 0, QoSProfile qos = QoSProfile::defaultProfile(),
              bool broadcast = false)
        : _topicName(topicName)
        , _remoteIP(remoteIP)
        , _remotePort(remotePort)
        , _localPort(localPort)
        , _qos(qos)
        , _broadcast(broadcast)
        , _pubCount(0)
        , _initialized(false)
    {
        TopicRegistry::instance().registerTopic(topicName, remotePort, sizeof(T), true);
    }
    
    /**
     * @brief Initialize the publisher (call after WiFi is connected)
     */
    bool init() {
        if (_localPort > 0) {
            _initialized = _udp.begin(_localPort);
        } else {
            _initialized = true;  // No local binding needed for sending
        }
        
        if (_initialized) {
            if (_broadcast) {
                Serial.printf("[Publisher] %s -> BROADCAST:%d\n", _topicName, _remotePort);
            } else {
                Serial.printf("[Publisher] %s -> %s:%d\n", _topicName, _remoteIP, _remotePort);
            }
        }
        return _initialized;
    }
    
    /**
     * @brief Publish a message
     */
    bool publish(const T& msg) {
        if (!_initialized) return false;
        
        // Use broadcast address if enabled
        if (_broadcast) {
            _udp.beginPacket(IPAddress(255, 255, 255, 255), _remotePort);
        } else {
            _udp.beginPacket(_remoteIP, _remotePort);
        }
        
        // Use serialize() if available, otherwise raw memory
        if constexpr (requires { msg.serialize((uint8_t*)nullptr); }) {
            uint8_t buffer[sizeof(T)];
            msg.serialize(buffer);
            _udp.write(buffer, sizeof(T));
        } else {
            _udp.write(reinterpret_cast<const uint8_t*>(&msg), sizeof(T));
        }
        
        bool success = _udp.endPacket();
        if (success) {
            _pubCount++;
            _lastPubTime = micros();
        }
        return success;
    }
    
    /**
     * @brief Publish raw bytes
     */
    bool publishRaw(const uint8_t* data, size_t len) {
        if (!_initialized) return false;
        
        _udp.beginPacket(_remoteIP, _remotePort);
        _udp.write(data, len);
        return _udp.endPacket();
    }
    
    const char* getTopicName() const { return _topicName; }
    uint32_t getPublishCount() const { return _pubCount; }
    uint64_t getLastPublishTime() const { return _lastPubTime; }
    
    static constexpr size_t msgSize() { return sizeof(T); }
    
private:
    const char* _topicName;
    const char* _remoteIP;
    uint16_t _remotePort;
    uint16_t _localPort;
    QoSProfile _qos;
    bool _broadcast;
    WiFiUDP _udp;
    uint32_t _pubCount;
    uint64_t _lastPubTime = 0;
    bool _initialized;
};

// =============================================================================
// Subscription
// =============================================================================

/**
 * @brief Callback function type for subscriptions
 */
template<typename T>
using SubscriptionCallback = std::function<void(const T&)>;

/**
 * @brief Subscribes to messages from a topic
 * 
 * @tparam T Message type (POD struct with deserialize() method)
 * 
 * @example
 * @code
 * void onCommand(const MotorCommand& msg) {
 *     Serial.printf("Target: %.2f\n", msg.target);
 * }
 * 
 * auto sub = node.createSubscription<MotorCommand>("/motor/command", onCommand, 6666);
 * @endcode
 */
template<typename T>
class Subscription {
public:
    Subscription(const char* topicName, SubscriptionCallback<T> callback,
                 uint16_t localPort, QoSProfile qos = QoSProfile::defaultProfile())
        : _topicName(topicName)
        , _callback(callback)
        , _localPort(localPort)
        , _qos(qos)
        , _recvCount(0)
        , _dropCount(0)
        , _initialized(false)
    {
        TopicRegistry::instance().registerTopic(topicName, localPort, sizeof(T), false);
    }
    
    /**
     * @brief Initialize the subscription (bind to port)
     */
    bool init() {
        _initialized = _udp.begin(_localPort);
        if (_initialized) {
            Serial.printf("[Subscription] %s <- port %d\n", _topicName, _localPort);
        } else {
            Serial.printf("[Subscription] FAILED to bind %s to port %d\n", _topicName, _localPort);
        }
        return _initialized;
    }
    
    /**
     * @brief Initialize with multicast group membership
     * @param multicastIP Multicast group IP to join (e.g., "239.255.0.1")
     */
    bool initMulticast(const char* multicastIP) {
        IPAddress mcastAddr;
        mcastAddr.fromString(multicastIP);
        
        // ESP32 WiFiUDP has beginMulticast method
        _initialized = _udp.beginMulticast(mcastAddr, _localPort);
        
        if (_initialized) {
            Serial.printf("[Subscription] %s <- MULTICAST %s:%d\n", _topicName, multicastIP, _localPort);
        } else {
            // Fallback: try regular bind and manually join multicast group
            _initialized = _udp.begin(_localPort);
            if (_initialized) {
                #ifdef ESP32
                // Join multicast group using IGMP
                ip4_addr_t mcast4;
                mcast4.addr = (uint32_t)mcastAddr;
                ip4_addr_t any4 = {0};  // 0.0.0.0 = any interface
                igmp_joingroup(&any4, &mcast4);
                #endif
                Serial.printf("[Subscription] %s <- MULTICAST %s:%d (igmp)\n", 
                              _topicName, multicastIP, _localPort);
            } else {
                Serial.printf("[Subscription] FAILED multicast %s:%d\n", multicastIP, _localPort);
            }
        }
        return _initialized;
    }
    
    /**
     * @brief Process one pending message (non-blocking)
     * @return true if a message was processed
     */
    bool spinOnce() {
        if (!_initialized) return false;
        
        int packetSize = _udp.parsePacket();
        if (packetSize == 0) return false;
        
        if ((size_t)packetSize < sizeof(T)) {
            // Flush invalid packet
            while (_udp.available()) _udp.read();
            _dropCount++;
            return false;
        }
        
        uint8_t buffer[sizeof(T)];
        _udp.read(buffer, sizeof(T));
        
        T msg;
        // Use fromBytes() if available, otherwise deserialize()
        if constexpr (requires { T::fromBytes(buffer, sizeof(T)); }) {
            msg = T::fromBytes(buffer, sizeof(T));
        } else {
            memcpy(&msg, buffer, sizeof(T));
        }
        
        _recvCount++;
        _lastRecvTime = micros();
        
        // Call the callback
        if (_callback) {
            _callback(msg);
        }
        
        return true;
    }
    
    /**
     * @brief Process all pending messages
     * @return Number of messages processed
     */
    size_t spinAll() {
        size_t count = 0;
        while (spinOnce()) {
            count++;
            if (count >= _qos.depth) break;  // Limit per spin
        }
        return count;
    }
    
    /**
     * @brief Take a message without callback (polling mode)
     */
    bool take(T& msg) {
        if (!_initialized) return false;
        
        int packetSize = _udp.parsePacket();
        if (packetSize == 0) return false;
        
        if ((size_t)packetSize < sizeof(T)) {
            while (_udp.available()) _udp.read();
            _dropCount++;
            return false;
        }
        
        uint8_t buffer[sizeof(T)];
        _udp.read(buffer, sizeof(T));
        
        if constexpr (requires { T::fromBytes(buffer, sizeof(T)); }) {
            msg = T::fromBytes(buffer, sizeof(T));
        } else {
            memcpy(&msg, buffer, sizeof(T));
        }
        
        _recvCount++;
        _lastRecvTime = micros();
        return true;
    }
    
    const char* getTopicName() const { return _topicName; }
    uint32_t getReceiveCount() const { return _recvCount; }
    uint32_t getDropCount() const { return _dropCount; }
    uint64_t getLastReceiveTime() const { return _lastRecvTime; }
    
    static constexpr size_t msgSize() { return sizeof(T); }
    
private:
    const char* _topicName;
    SubscriptionCallback<T> _callback;
    uint16_t _localPort;
    QoSProfile _qos;
    WiFiUDP _udp;
    uint32_t _recvCount;
    uint32_t _dropCount;
    uint64_t _lastRecvTime = 0;
    bool _initialized;
};

// =============================================================================
// Timer
// =============================================================================

/**
 * @brief Callback function type for timers
 */
using TimerCallback = std::function<void()>;

/**
 * @brief Periodic timer
 * 
 * @example
 * @code
 * Timer timer(0.01, []() {  // 100 Hz
 *     // Timer callback
 * });
 * 
 * void loop() {
 *     timer.spinOnce();
 * }
 * @endcode
 */
class Timer {
public:
    Timer(float periodSec, TimerCallback callback)
        : _periodUs(static_cast<uint64_t>(periodSec * 1000000))
        , _callback(callback)
        , _lastFire(0)
        , _callCount(0)
        , _active(true)
    {}
    
    /**
     * @brief Check and fire timer if ready
     * @return true if timer fired
     */
    bool spinOnce() {
        if (!_active) return false;
        
        uint64_t now = micros();
        if (now - _lastFire >= _periodUs) {
            _lastFire = now;
            _callCount++;
            if (_callback) {
                _callback();
            }
            return true;
        }
        return false;
    }
    
    void reset() { _lastFire = micros(); }
    void cancel() { _active = false; }
    void resume() { _active = true; reset(); }
    
    bool isActive() const { return _active; }
    uint32_t getCallCount() const { return _callCount; }
    float getPeriod() const { return _periodUs / 1000000.0f; }
    float getFrequency() const { return 1000000.0f / _periodUs; }
    
private:
    uint64_t _periodUs;
    TimerCallback _callback;
    uint64_t _lastFire;
    uint32_t _callCount;
    bool _active;
};

// =============================================================================
// Rate Limiter
// =============================================================================

/**
 * @brief Rate limiter for control loops
 * 
 * @example
 * @code
 * Rate rate(100);  // 100 Hz
 * 
 * void loop() {
 *     // Do work
 *     rate.sleep();
 * }
 * @endcode
 */
class Rate {
public:
    Rate(float hz) : _periodUs(static_cast<uint64_t>(1000000.0f / hz)) {
        _lastTime = micros();
    }
    
    /**
     * @brief Sleep to maintain the target rate
     */
    void sleep() {
        uint64_t now = micros();
        uint64_t elapsed = now - _lastTime;
        
        if (elapsed < _periodUs) {
            uint64_t sleepUs = _periodUs - elapsed;
            if (sleepUs > 1000) {
                // Use delay for longer sleeps
                delay(sleepUs / 1000);
            } else {
                // Busy wait for short sleeps
                delayMicroseconds(sleepUs);
            }
        }
        
        _lastTime = micros();
    }
    
    float getPeriod() const { return _periodUs / 1000000.0f; }
    float getFrequency() const { return 1000000.0f / _periodUs; }
    
private:
    uint64_t _periodUs;
    uint64_t _lastTime;
};

// =============================================================================
// Node
// =============================================================================

/**
 * @brief Maximum number of publishers/subscribers per node
 */
constexpr size_t MAX_PUBLISHERS = 8;
constexpr size_t MAX_SUBSCRIPTIONS = 8;
constexpr size_t MAX_TIMERS = 8;

/**
 * @brief A computational node with publishers, subscribers, and timers
 * 
 * This is the main entry point for the pub/sub system, similar to ROS2 nodes.
 * 
 * @example
 * @code
 * cpy::Node node("motor_module");
 * 
 * // Connect to WiFi
 * node.initWiFi("SSID", "password");
 * 
 * // Create publisher and subscriber
 * auto* cmdSub = node.createSubscription<MotorCommand>("/motor/cmd", onCommand, 6666);
 * auto* fbPub = node.createPublisher<SensorData>("/motor/feedback", serverIP, 6667);
 * 
 * // Create 100Hz timer
 * node.createTimer(0.01, controlLoop);
 * 
 * void loop() {
 *     node.spinOnce();
 * }
 * @endcode
 */
class Node {
public:
    /**
     * @brief Create a node
     * @param name Node name
     * @param ns Optional namespace
     */
    Node(const char* name, const char* ns = "")
        : _name(name)
        , _namespace(ns)
        , _numPubs(0)
        , _numSubs(0)
        , _numTimers(0)
    {
        Serial.printf("[Node] Created: %s%s%s\n", 
            strlen(ns) > 0 ? ns : "", 
            strlen(ns) > 0 ? "/" : "", 
            name);
    }
    
    /**
     * @brief Initialize WiFi connection
     */
    bool initWiFi(const char* ssid, const char* password, uint32_t timeout = 30000) {
        Serial.printf("[Node] Connecting to WiFi '%s'...\n", ssid);
        
        #ifdef ESP32
        WiFi.setSleep(false);
        #endif
        WiFi.begin(ssid, password);
        
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - start > timeout) {
                Serial.println("\n[Node] WiFi connection timeout!");
                return false;
            }
            delay(100);
            Serial.print(".");
        }
        
        Serial.printf("\n[Node] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        return true;
    }
    
    /**
     * @brief Create a publisher
     * 
     * @tparam T Message type
     * @param topic Topic name
     * @param remoteIP Remote server IP
     * @param remotePort Remote server port
     * @param qos QoS profile
     * @return Publisher pointer (owned by node)
     */
    template<typename T>
    Publisher<T>* createPublisher(const char* topic, const char* remoteIP, 
                                   uint16_t remotePort, QoSProfile qos = QoSProfile::defaultProfile()) {
        if (_numPubs >= MAX_PUBLISHERS) {
            Serial.println("[Node] Max publishers reached!");
            return nullptr;
        }
        
        auto* pub = new Publisher<T>(topic, remoteIP, remotePort, 0, qos, false);
        pub->init();
        _publishers[_numPubs++] = {pub, [](void* p) { delete static_cast<Publisher<T>*>(p); }};
        
        return pub;
    }
    
    /**
     * @brief Create a broadcast publisher (no IP needed!)
     * 
     * Uses UDP broadcast to send to all devices on the network.
     * Any device listening on the specified port will receive messages.
     * NOTE: Broadcast only works within the same subnet!
     * 
     * @tparam T Message type
     * @param topic Topic name
     * @param remotePort Port that receivers are listening on
     * @param qos QoS profile
     * @return Publisher pointer (owned by node)
     */
    template<typename T>
    Publisher<T>* createBroadcastPublisher(const char* topic, uint16_t remotePort, 
                                            QoSProfile qos = QoSProfile::defaultProfile()) {
        if (_numPubs >= MAX_PUBLISHERS) {
            Serial.println("[Node] Max publishers reached!");
            return nullptr;
        }
        
        // Use broadcast mode (IP is ignored when broadcast=true)
        auto* pub = new Publisher<T>(topic, "255.255.255.255", remotePort, 0, qos, true);
        pub->init();
        _publishers[_numPubs++] = {pub, [](void* p) { delete static_cast<Publisher<T>*>(p); }};
        
        return pub;
    }
    
    /**
     * @brief Create a multicast publisher (works across subnets!)
     * 
     * Uses UDP multicast to send to all devices that joined the multicast group.
     * Unlike broadcast, multicast CAN be routed across subnets if the network allows.
     * 
     * Default multicast group: 239.255.0.1 (like ROS2 DDS)
     * 
     * @tparam T Message type
     * @param topic Topic name
     * @param remotePort Port that receivers are listening on
     * @param multicastIP Multicast group IP (default: 239.255.0.1)
     * @param qos QoS profile
     * @return Publisher pointer (owned by node)
     */
    template<typename T>
    Publisher<T>* createMulticastPublisher(const char* topic, uint16_t remotePort,
                                            const char* multicastIP = "239.255.0.1",
                                            QoSProfile qos = QoSProfile::defaultProfile()) {
        if (_numPubs >= MAX_PUBLISHERS) {
            Serial.println("[Node] Max publishers reached!");
            return nullptr;
        }
        
        // Multicast uses the multicast IP directly
        auto* pub = new Publisher<T>(topic, multicastIP, remotePort, 0, qos, false);
        pub->init();
        _publishers[_numPubs++] = {pub, [](void* p) { delete static_cast<Publisher<T>*>(p); }};
        
        Serial.printf("[Publisher] %s -> MULTICAST %s:%d\n", topic, multicastIP, remotePort);
        return pub;
    }
    
    /**
     * @brief Create a subscription
     * 
     * @tparam T Message type
     * @param topic Topic name
     * @param callback Callback function
     * @param localPort Local port to bind
     * @param qos QoS profile
     * @return Subscription pointer (owned by node)
     */
    template<typename T>
    Subscription<T>* createSubscription(const char* topic, SubscriptionCallback<T> callback,
                                         uint16_t localPort, QoSProfile qos = QoSProfile::defaultProfile()) {
        if (_numSubs >= MAX_SUBSCRIPTIONS) {
            Serial.println("[Node] Max subscriptions reached!");
            return nullptr;
        }
        
        auto* sub = new Subscription<T>(topic, callback, localPort, qos);
        sub->init();
        _subscriptions[_numSubs++] = {sub, [](void* s) { delete static_cast<Subscription<T>*>(s); }};
        
        return sub;
    }
    
    /**
     * @brief Create a subscription without callback (polling mode)
     */
    template<typename T>
    Subscription<T>* createSubscription(const char* topic, uint16_t localPort,
                                         QoSProfile qos = QoSProfile::defaultProfile()) {
        return createSubscription<T>(topic, nullptr, localPort, qos);
    }
    
    /**
     * @brief Create a multicast subscription (works across subnets!)
     * 
     * Joins a multicast group to receive messages from any sender on the network.
     * Unlike broadcast, multicast CAN work across subnets if routing is enabled.
     * 
     * @tparam T Message type
     * @param topic Topic name
     * @param callback Callback function
     * @param localPort Local port to bind
     * @param multicastIP Multicast group IP (default: 239.255.0.1)
     * @param qos QoS profile
     * @return Subscription pointer (owned by node)
     */
    template<typename T>
    Subscription<T>* createMulticastSubscription(const char* topic, SubscriptionCallback<T> callback,
                                                  uint16_t localPort, 
                                                  const char* multicastIP = "239.255.0.1",
                                                  QoSProfile qos = QoSProfile::defaultProfile()) {
        if (_numSubs >= MAX_SUBSCRIPTIONS) {
            Serial.println("[Node] Max subscriptions reached!");
            return nullptr;
        }
        
        auto* sub = new Subscription<T>(topic, callback, localPort, qos);
        
        // Initialize with multicast group
        if (sub->initMulticast(multicastIP)) {
            _subscriptions[_numSubs++] = {sub, [](void* s) { delete static_cast<Subscription<T>*>(s); }};
            Serial.printf("[Subscription] %s <- MULTICAST %s:%d\n", topic, multicastIP, localPort);
            return sub;
        } else {
            delete sub;
            return nullptr;
        }
    }
    
    /**
     * @brief Create a periodic timer
     * 
     * @param periodSec Period in seconds
     * @param callback Callback function
     * @return Timer pointer (owned by node)
     */
    Timer* createTimer(float periodSec, TimerCallback callback) {
        if (_numTimers >= MAX_TIMERS) {
            Serial.println("[Node] Max timers reached!");
            return nullptr;
        }
        
        auto* timer = new Timer(periodSec, callback);
        _timers[_numTimers++] = timer;
        
        Serial.printf("[Node] Timer created: %.1f Hz\n", 1.0f / periodSec);
        return timer;
    }
    
    /**
     * @brief Process all pending callbacks once
     * @return Number of callbacks executed
     */
    size_t spinOnce() {
        size_t count = 0;
        
        // Process subscriptions
        for (size_t i = 0; i < _numSubs; i++) {
            // Each subscription manages its own type
            // spinOnce() is called through the stored function pointer pattern
            // For now, we rely on the user calling sub->spinOnce() or take()
        }
        
        // Process timers
        for (size_t i = 0; i < _numTimers; i++) {
            if (_timers[i]->spinOnce()) count++;
        }
        
        return count;
    }
    
    /**
     * @brief Spin a specific subscription
     */
    template<typename T>
    size_t spin(Subscription<T>* sub) {
        return sub->spinAll();
    }
    
    /**
     * @brief Get node name
     */
    const char* getName() const { return _name; }
    const char* getNamespace() const { return _namespace; }
    
    /**
     * @brief Get logger
     */
    void logInfo(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        Serial.printf("[INFO] [%s]: ", _name);
        // vprintf not available on Arduino, use workaround
        char buf[256];
        vsnprintf(buf, sizeof(buf), fmt, args);
        Serial.println(buf);
        va_end(args);
    }
    
    void logWarn(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        Serial.printf("[WARN] [%s]: ", _name);
        char buf[256];
        vsnprintf(buf, sizeof(buf), fmt, args);
        Serial.println(buf);
        va_end(args);
    }
    
    void logError(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        Serial.printf("[ERROR] [%s]: ", _name);
        char buf[256];
        vsnprintf(buf, sizeof(buf), fmt, args);
        Serial.println(buf);
        va_end(args);
    }
    
    /**
     * @brief Destructor - cleanup all resources
     */
    ~Node() {
        for (size_t i = 0; i < _numPubs; i++) {
            _publishers[i].deleter(_publishers[i].ptr);
        }
        for (size_t i = 0; i < _numSubs; i++) {
            _subscriptions[i].deleter(_subscriptions[i].ptr);
        }
        for (size_t i = 0; i < _numTimers; i++) {
            delete _timers[i];
        }
    }
    
private:
    const char* _name;
    const char* _namespace;
    
    // Type-erased storage for publishers/subscriptions
    struct TypeErased {
        void* ptr;
        void (*deleter)(void*);
    };
    
    TypeErased _publishers[MAX_PUBLISHERS];
    TypeErased _subscriptions[MAX_SUBSCRIPTIONS];
    Timer* _timers[MAX_TIMERS];
    
    size_t _numPubs;
    size_t _numSubs;
    size_t _numTimers;
};

// =============================================================================
// Global Functions (ROS2-like)
// =============================================================================

/**
 * @brief Check if system is running (placeholder)
 */
inline bool ok() {
    return WiFi.status() == WL_CONNECTED;
}

/**
 * @brief Print topic list
 */
inline void printTopics() {
    TopicRegistry::instance().printTopics();
}

} // namespace cpy

#endif // CAPYBARISH_PUBSUB_H
