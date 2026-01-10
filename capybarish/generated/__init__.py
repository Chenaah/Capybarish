"""
Generated message types for Capybarish communication.

This module re-exports generated message types from schema definitions.
The message types are binary-compatible with the ESP32/Arduino counterparts.

Usage:
    from capybarish.generated import MotorCommand, SensorData
    
    # Create command to send to ESP32
    cmd = MotorCommand()
    cmd.target = 1.5
    cmd.kp = 10.0
    data = cmd.serialize()
    
    # Parse response from ESP32
    response = SensorData.deserialize(received_bytes)
    print(response.motor.pos)
    print(response.goal_distance)  # New field for distance info

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>
"""

from .motor_control_messages import (
    MotorCommand,
    SensorData,
    MotorData,
    IMUData,
    IMUOrientation,
    IMUQuaternion,
    IMUOmega,
    IMUAcceleration,
    ErrorData,
    UWBDistances,
    MESSAGE_TYPES,
    get_message_type,
)

__all__ = [
    "MotorCommand",
    "SensorData",
    "MotorData",
    "IMUData",
    "IMUOrientation",
    "IMUQuaternion",
    "IMUOmega",
    "IMUAcceleration",
    "ErrorData",
    "UWBDistances",
    "MESSAGE_TYPES",
    "get_message_type",
]
