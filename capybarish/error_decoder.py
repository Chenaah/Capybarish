"""
Error Decoder Protocol for Capybarish.

This module defines the interface for motor/driver error decoding,
allowing the dashboard to display human-readable error messages
for different motor types.

Users can implement custom decoders for their specific motor hardware
by implementing the ErrorDecoder protocol or subclassing BaseErrorDecoder.

Example Usage:
    ```python
    from capybarish import RLDashboard
    from capybarish.devices import CybergearErrorDecoder
    
    # Use Cybergear-specific decoder
    dashboard = RLDashboard(error_decoder=CybergearErrorDecoder())
    
    # Or create a custom decoder
    class MyMotorDecoder(BaseErrorDecoder):
        def decode_motor_error(self, error: int) -> str:
            # Custom decoding logic
            return "OK" if error == 0 else f"ERR:{error}"
    
    dashboard = RLDashboard(error_decoder=MyMotorDecoder())
    ```

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>
Licensed under the Apache License, Version 2.0
"""

from typing import Protocol, runtime_checkable


@runtime_checkable
class ErrorDecoder(Protocol):
    """Protocol for motor/driver error decoding.
    
    Implement this protocol to provide custom error decoding
    for your specific motor hardware.
    """
    
    def decode_motor_error(self, error: int) -> str:
        """Decode motor error flags into human-readable string.
        
        Args:
            error: Motor error code/flags (typically from motor controller)
            
        Returns:
            Human-readable error string, or empty string if no error
        """
        ...
    
    def decode_driver_error(self, error: int) -> str:
        """Decode driver error flags into human-readable string.
        
        Args:
            error: Driver error code/flags (typically from motor driver chip)
            
        Returns:
            Human-readable error string, or empty string if no error
        """
        ...


class BaseErrorDecoder:
    """Base class for error decoders with common utilities.
    
    Subclass this to create custom decoders with helper methods.
    """
    
    def decode_motor_error(self, error: int) -> str:
        """Default implementation shows hex code."""
        return f"M{error:02X}" if error else ""
    
    def decode_driver_error(self, error: int) -> str:
        """Default implementation shows hex code."""
        return f"D{error:X}" if error else ""
    
    @staticmethod
    def decode_bitfield(error: int, bit_definitions: dict) -> str:
        """Helper to decode a bitfield using a bit->name mapping.
        
        Args:
            error: Error code to decode
            bit_definitions: Dict mapping bit positions to error names
            
        Returns:
            Comma-separated error names, or empty string if no errors
        """
        if error == 0:
            return ""
        errors = []
        for bit, name in bit_definitions.items():
            if error & (1 << bit):
                errors.append(name)
        return ",".join(errors)


class DefaultErrorDecoder(BaseErrorDecoder):
    """Default error decoder that displays hex codes.
    
    Used when no device-specific decoder is provided.
    Shows motor errors as M{hex} and driver errors as D{hex}.
    """
    pass


# Singleton default decoder
default_decoder = DefaultErrorDecoder()

