# Capybarish Schema Files

This directory contains `.cpy` schema files that define message structures for cross-platform communication between Python and C++/ESP32.

## Schema Syntax

Capybarish uses a simple, human-readable schema format inspired by Protocol Buffers and LCM.

### Basic Structure

```
# Comment
package mypackage

message MessageName:
    type field_name           # Optional comment
    type field_name2
```

### Supported Types

| Type | Python | C++ | Size |
|------|--------|-----|------|
| `int8` | `int` | `int8_t` | 1 byte |
| `int16` | `int` | `int16_t` | 2 bytes |
| `int32` / `int` | `int` | `int32_t` | 4 bytes |
| `int64` | `int` | `int64_t` | 8 bytes |
| `uint8` / `byte` | `int` | `uint8_t` | 1 byte |
| `uint16` | `int` | `uint16_t` | 2 bytes |
| `uint32` | `int` | `uint32_t` | 4 bytes |
| `uint64` | `int` | `uint64_t` | 8 bytes |
| `float32` / `float` | `float` | `float` | 4 bytes |
| `float64` / `double` | `float` | `double` | 8 bytes |
| `bool` | `bool` | `bool` | 1 byte |

### Nested Types

You can define and use nested message types:

```
message Point3D:
    float32 x
    float32 y
    float32 z

message Pose:
    Point3D position
    Point3D orientation
```

### Arrays

Fixed-size arrays are supported:

```
message BatchData:
    float32[8] values     # Array of 8 floats
    int32[4] flags        # Array of 4 integers
```

## Example Schemas

### simple_example.cpy
A minimal example with basic command and status messages.

### motor_control.cpy
Complete schema for motor control applications, matching the original Capybarish data structures.

### multi_robot.cpy
Example schema for coordinating multiple robots with nested types and arrays.

## Generating Code

Use the `capybarish` CLI to generate Python and C++ code:

```bash
# Generate both Python and C++
capybarish gen --all schemas/motor_control.cpy --output generated/

# Generate only Python
capybarish gen --python schemas/motor_control.cpy --output generated/

# Generate only C++
capybarish gen --cpp schemas/motor_control.cpy --output generated/

# Validate a schema without generating code
capybarish validate schemas/motor_control.cpy

# Create a new schema template
capybarish init --output my_messages.cpy
```

## Using Generated Code

### Python

```python
from generated.motor_control_messages import MotorCommand, SensorData

# Create and serialize
cmd = MotorCommand()
cmd.target = 1.5
cmd.kp = 10.0
data = cmd.serialize()

# Deserialize
received = SensorData.deserialize(data)
print(f"Position: {received.motor.pos}")
print(f"Distance: {received.goal_distance}")
```

### C++ (Arduino/ESP32)

```cpp
#include "motor_control_messages.hpp"
#include <capybarish.h>

using namespace motor_control;

// Create communication
Capybarish::UDPComm<MotorCommand, SensorData> comm;

// Receive and send
MotorCommand cmd;
if (comm.receive(cmd)) {
    // Process command
}

SensorData status;
status.motor.pos = 1.5f;
status.goal_distance = 2.5f;  // Distance to goal in meters
comm.send(status);
```

## Design Philosophy

Capybarish schemas are designed to be:

1. **Simple**: Easy to read and write, minimal syntax
2. **Portable**: Same schema generates compatible code for Python and C++
3. **Efficient**: Binary serialization with no overhead
4. **Type-safe**: Strong typing with compile-time checks

The generated code uses:
- Python: `dataclasses` with `struct` for serialization
- C++: POD structs with `#pragma pack(1)` for consistent memory layout
