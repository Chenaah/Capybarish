# Multi-Robot Schema
# Example for coordinating multiple robots with arrays
#
# Usage:
#   capybarish-gen --python --cpp schemas/multi_robot.cpy

package multi_robot

# Position in 3D space
message Position3D:
    float32 x
    float32 y
    float32 z

# Orientation as quaternion
message Quaternion:
    float32 x
    float32 y
    float32 z
    float32 w

# Complete pose (position + orientation)
message Pose:
    Position3D position
    Quaternion orientation

# Command for a single robot
message RobotCommand:
    int32 robot_id           # Target robot ID
    Pose target_pose         # Target pose
    float32 velocity         # Max velocity
    int32 flags              # Control flags

# Batch command for multiple robots (up to 8)
message BatchCommand:
    int32 count              # Number of active commands
    int32 sequence_id        # Sequence number for tracking
    float32[8] targets       # Target values for 8 robots
    int32[8] modes           # Modes for 8 robots

# Status from a single robot
message RobotStatus:
    int32 robot_id
    Pose current_pose
    float32 battery_level    # Battery percentage (0-100)
    int32 status_flags
    uint64 timestamp_us      # Microsecond timestamp
