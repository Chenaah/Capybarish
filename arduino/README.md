# Capybarish Arduino/ESP32 Library

Lightweight communication middleware for ESP32 robots - UDP-based real-time communication for modular robotics.

## Installation

### PlatformIO (Recommended)

Add to your `platformio.ini`:

```ini
lib_deps =
    https://github.com/chenaah/capybarish.git#main
```

Or install from local path:

```ini
lib_deps =
    /path/to/capybarish/arduino
```

### Arduino IDE

1. Download this library
2. Place it in your Arduino libraries folder (`~/Arduino/libraries/Capybarish`)
3. Restart Arduino IDE

### From Python Package

If you have the Python package installed:

```bash
capybarish arduino-install --path ~/Arduino/libraries/
```

## Usage

### Basic Example

```cpp
#include <capybarish.h>

// Define your message types (or use generated ones)
struct MyCommand {
    float target;
    float velocity;
    int32_t mode;
};

struct MySensorData {
    float position;
    float velocity;
    int32_t status;
};

// Create communication instance
Capybarish::UDPComm<MyCommand, MySensorData> comm;

void setup() {
    Serial.begin(115200);
    
    // Connect to WiFi and setup UDP
    comm.begin("YourSSID", "YourPassword", "192.168.1.100", 6666);
}

void loop() {
    // Receive commands
    MyCommand cmd;
    if (comm.receive(cmd)) {
        // Process command
        Serial.printf("Target: %.2f\n", cmd.target);
    }
    
    // Send sensor data
    MySensorData data;
    data.position = 1.23;
    data.velocity = 0.5;
    data.status = 1;
    comm.send(data);
    
    delay(10);
}
```

### Using Generated Message Types

Use the `capybarish-gen` tool to generate C++ headers from schema files:

```bash
capybarish-gen --cpp --output arduino/src/generated/ schemas/my_messages.cpy
```

Then include the generated header:

```cpp
#include <capybarish.h>
#include "generated/my_messages.hpp"
```

## API Reference

### `Capybarish::UDPComm<TReceive, TSend>`

Template class for UDP communication.

- `begin(ssid, password, serverIP, port)` - Initialize WiFi and UDP
- `receive(data)` - Receive data into struct, returns true if data received
- `send(data)` - Send data struct to server
- `isConnected()` - Check WiFi connection status
- `getLocalIP()` - Get device IP address

### `Capybarish::Message<T>`

Wrapper for message serialization (for advanced use).

- `serialize()` - Convert to byte array
- `deserialize(buffer, len)` - Parse from byte array
- `size()` - Get serialized size

## License

Apache License 2.0 - See [LICENSE](../LICENSE)
