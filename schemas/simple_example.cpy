# Simple Example Schema
# A minimal schema demonstrating basic message types
#
# Usage:
#   capybarish-gen --python --cpp schemas/simple_example.cpy

package simple

# Simple command message
message Command:
    float32 value            # Target value
    int32 mode               # Operation mode

# Simple status response
message Status:
    float32 position         # Current position
    float32 velocity         # Current velocity
    int32 error_code         # Error code (0 = OK)
    int32 timestamp          # Timestamp in microseconds
