# Motor Control Schema for Capybarish
# This schema defines message types for robot motor control and sensor feedback
#
# Usage:
#   capybarish-gen --python --cpp schemas/motor_control.cpy
#
# Compatible with ESP32/Arduino and Python

package motor_control

# ============================================================================
# Command Messages (Server -> Robot)
# ============================================================================

# Motor command sent from server to robot module
message ReceivedData:
    float32 target           # Target position (radians)
    float32 target_vel       # Target velocity (rad/s)
    float32 kp               # Proportional gain
    float32 kd               # Derivative gain
    int32 enable_filter      # Enable low-pass filter (0 or 1)
    int32 switch_            # Motor switch state (0=off, 1=on)
    int32 calibrate          # Trigger calibration (0 or 1)
    int32 restart            # Trigger restart (0 or 1)
    float32 timestamp        # Command timestamp (seconds)

# ============================================================================
# Sensor Data Messages (Robot -> Server)
# ============================================================================

# IMU orientation (Euler angles in radians)
message IMUOrientation:
    float32 x                # Roll
    float32 y                # Pitch
    float32 z                # Yaw

# IMU quaternion representation
message IMUQuaternion:
    float32 x
    float32 y
    float32 z
    float32 w

# IMU angular velocity (rad/s)
message IMUOmega:
    float32 x
    float32 y
    float32 z

# IMU linear acceleration (m/s²)
message IMUAcceleration:
    float32 x
    float32 y
    float32 z

# Complete IMU data package
message IMUData:
    IMUOrientation orientation
    IMUQuaternion quaternion
    IMUOmega omega
    IMUAcceleration acceleration

# Motor sensor data
message MotorData:
    float32 pos              # Current position (radians)
    float32 large_pos        # Unwrapped position (radians)
    float32 vel              # Current velocity (rad/s)
    float32 torque           # Current torque (Nm)
    float32 voltage          # Motor voltage (V)
    float32 current          # Motor current (A)
    int32 temperature        # Temperature (°C)
    int32 error0             # Error code 0 (mode & error)
    int32 error1             # Error code 1

# System error data
message ErrorData:
    int32 reset_reason0      # CPU0 reset reason
    int32 reset_reason1      # CPU1 reset reason

# Complete sensor data from robot module
message SentData:
    int32 module_id          # Unique module identifier
    int32 receive_dt         # Receive processing time (µs)
    int32 timestamp          # Current timestamp (µs)
    int32 switch_off         # Switch off request flag
    float32 last_rcv_timestamp  # Last received command timestamp
    int32 info               # Info/status code
    MotorData motor          # Motor sensor data
    IMUData imu              # IMU sensor data
    ErrorData error          # Error/reset data
