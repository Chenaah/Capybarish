"""
Generated message types for Capybarish communication.

This module re-exports generated message types from schema definitions.
The message types are binary-compatible with the ESP32/Arduino counterparts.

Usage:
    from capybarish.generated import ReceivedData, SentData
    
    # Create command to send to ESP32
    cmd = ReceivedData()
    cmd.target = 1.5
    cmd.kp = 10.0
    data = cmd.serialize()
    
    # Parse response from ESP32
    response = SentData.deserialize(received_bytes)
    print(response.motor.pos)

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>
"""

from .motor_control_messages import (
    ReceivedData,
    SentData,
    MotorData,
    IMUData,
    IMUOrientation,
    IMUQuaternion,
    IMUOmega,
    IMUAcceleration,
    ErrorData,
    MESSAGE_TYPES,
    get_message_type,
)

__all__ = [
    "ReceivedData",
    "SentData",
    "MotorData",
    "IMUData",
    "IMUOrientation",
    "IMUQuaternion",
    "IMUOmega",
    "IMUAcceleration",
    "ErrorData",
    "MESSAGE_TYPES",
    "get_message_type",
]
