"""
Xiaomi Cybergear Motor Error Decoder.

This module provides error decoding specific to the Xiaomi Cybergear
motor controller, translating error codes into human-readable messages.

Error Code Reference:
    Motor Error (st.error_state & 0x3F):
        - Bit 0: Undervoltage
        - Bit 1: Overcurrent  
        - Bit 2: Over temperature
        - Bit 3: Encoder error
        - Bit 4: Driver fault
        - Bit 5: Calibration error
    
    Driver Error (motor driver chip fault state):
        - Bit 0: Motor over temperature
        - Bit 1: Driver chip fault
        - Bit 2: Under voltage
        - Bit 3: Over voltage
        - Bit 4: Phase B overcurrent
        - Bit 5: Phase C overcurrent
        - Bit 7: Encoder not calibrated
        - Bits 8-15: Overload fault (8-bit value)
        - Bit 16: Phase A overcurrent
        - Bits 17-20: Fault flag (4-bit value)
        - Bit 24: Temperature warning

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>
Licensed under the Apache License, Version 2.0
"""

from ..error_decoder import BaseErrorDecoder


class CybergearErrorDecoder(BaseErrorDecoder):
    """Error decoder for Xiaomi Cybergear motor controller.
    
    Decodes motor and driver error codes into human-readable strings
    based on the Cybergear's error bit definitions.
    
    Example:
        ```python
        decoder = CybergearErrorDecoder()
        
        # Decode motor error
        motor_err = decoder.decode_motor_error(0x01)  # Returns "UV"
        
        # Decode driver error (temperature warning)
        driver_err = decoder.decode_driver_error(0x1000000)  # Returns "TmpWrn"
        ```
    """
    
    # Motor error bit definitions (from st.error_state & 0x3F)
    MOTOR_ERROR_BITS = {
        0: "UV",      # Undervoltage
        1: "OC",      # Overcurrent
        2: "OT",      # Over temperature
        3: "ENC",     # Encoder error
        4: "DRV",     # Driver fault
        5: "CAL",     # Calibration error
    }
    
    # Driver error bit definitions (from motor driver chip fault state)
    DRIVER_ERROR_BITS = {
        0: "MotOT",   # Motor over temp
        1: "DrvFlt",  # Driver chip fault
        2: "UV",      # Under voltage
        3: "OV",      # Over voltage
        4: "PhB_OC",  # Phase B overcurrent
        5: "PhC_OC",  # Phase C overcurrent
        7: "NoCAL",   # Encoder not calibrated
        16: "PhA_OC", # Phase A overcurrent
        24: "TmpWrn", # Temperature warning
    }
    
    def decode_motor_error(self, error: int) -> str:
        """Decode Cybergear motor error bits.
        
        Args:
            error: Motor error flags (6-bit value from st.error_state & 0x3F)
            
        Returns:
            Comma-separated error names (e.g., "UV,OC") or empty if no errors
        """
        if error == 0:
            return ""
        
        decoded = self.decode_bitfield(error, self.MOTOR_ERROR_BITS)
        return decoded if decoded else f"M{error:02X}"
    
    def decode_driver_error(self, error: int) -> str:
        """Decode Cybergear driver chip error bits.
        
        Args:
            error: Driver error flags (packed fault state from driver chip)
            
        Returns:
            Comma-separated error names (e.g., "TmpWrn,NoCAL") or empty if no errors
        """
        if error == 0:
            return ""
        
        errors = []
        
        # Decode individual bits
        for bit, name in self.DRIVER_ERROR_BITS.items():
            if error & (1 << bit):
                errors.append(name)
        
        # Check for overload fault (bits 8-15)
        overload = (error >> 8) & 0xFF
        if overload:
            errors.append(f"OL{overload}")
        
        # Check for fault flag (bits 17-20)
        fault_flag = (error >> 17) & 0x0F
        if fault_flag:
            errors.append(f"FF{fault_flag}")
        
        return ",".join(errors) if errors else f"D{error:X}"
    
    @classmethod
    def get_error_descriptions(cls) -> dict:
        """Get detailed error descriptions for documentation/UI.
        
        Returns:
            Dict mapping error codes to full descriptions
        """
        return {
            "motor": {
                "UV": "Undervoltage - Supply voltage too low",
                "OC": "Overcurrent - Motor current exceeded limit",
                "OT": "Over Temperature - Motor temperature too high",
                "ENC": "Encoder Error - Position sensor fault",
                "DRV": "Driver Fault - Motor driver chip error",
                "CAL": "Calibration Error - Encoder not calibrated",
            },
            "driver": {
                "MotOT": "Motor Over Temperature",
                "DrvFlt": "Driver Chip Fault",
                "UV": "Under Voltage",
                "OV": "Over Voltage",
                "PhA_OC": "Phase A Overcurrent",
                "PhB_OC": "Phase B Overcurrent",
                "PhC_OC": "Phase C Overcurrent",
                "NoCAL": "Encoder Not Calibrated",
                "TmpWrn": "Temperature Warning (approaching limit)",
                "OL": "Overload Fault (value indicates severity)",
                "FF": "Fault Flag (internal driver state)",
            }
        }

