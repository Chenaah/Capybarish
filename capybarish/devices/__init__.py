"""
Device-specific implementations for Capybarish.

This module contains error decoders and other device-specific
functionality for various motor controllers and drivers.

Available Devices:
    - CybergearErrorDecoder: Xiaomi Cybergear motor controller

Example:
    ```python
    from capybarish.devices import CybergearErrorDecoder
    from capybarish import RLDashboard
    
    dashboard = RLDashboard(error_decoder=CybergearErrorDecoder())
    ```
"""

from .cybergear import CybergearErrorDecoder

__all__ = [
    "CybergearErrorDecoder",
]

