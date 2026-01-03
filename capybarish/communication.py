"""
Modern communication layer for robot middleware.

This module provides a clean separation between communication protocols
and business logic, making the system more scalable and maintainable.

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import copy
import select
import socket
import struct
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

from .data_struct import RobotData, SentDataStruct  # Legacy support
from .generated import MotorCommand, SensorData  # New generated message types
from .utils import cache_pings, get_ping_time

# Type alias for command data (supports both legacy and generated types)
CommandData = Union[SentDataStruct, MotorCommand]


class ConnectionStatus(Enum):
    """Status of module connection."""

    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    PENDING = "pending"
    LOST = "lost"


@dataclass
class ModuleInfo:
    """Information about a connected module."""

    module_id: int
    address: Tuple[str, int]
    status: ConnectionStatus
    last_seen: float
    ping_time: Optional[float] = None
    pending_count: int = 0


class CommunicationProtocol(ABC):
    """Abstract base class for communication protocols."""

    @abstractmethod
    def setup(self, **kwargs) -> None:
        """Setup the communication protocol."""
        pass

    @abstractmethod
    def send_data(self, data: bytes, address: Tuple[str, int]) -> bool:
        """Send data to a specific address."""
        pass

    @abstractmethod
    def receive_data(self, timeout: float = 0.0) -> Optional[Tuple[bytes, Tuple[str, int]]]:
        """Receive data with optional timeout."""
        pass

    @abstractmethod
    def close(self) -> None:
        """Close the communication channel."""
        pass


class UDPProtocol(CommunicationProtocol):
    """UDP communication protocol implementation."""

    def __init__(self):
        self.socket: Optional[socket.socket] = None
        self.is_setup = False

    def setup(self, port: int = 6666, buffer_size: int = 4096, **kwargs) -> None:
        """Setup UDP socket."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buffer_size)

        server_address = ("0.0.0.0", port)
        self.socket.bind(server_address)
        self.is_setup = True

    def send_data(self, data: bytes, address: Tuple[str, int]) -> bool:
        """Send data via UDP."""
        if not self.is_setup or not self.socket:
            return False

        try:
            self.socket.sendto(data, address)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to send data to {address}: {e}")
            return False

    def receive_data(self, timeout: float = 0.0) -> Optional[Tuple[bytes, Tuple[str, int]]]:
        """Receive data via UDP with timeout."""
        if not self.is_setup or not self.socket:
            return None

        try:
            if timeout > 0:
                ready = select.select([self.socket], [], [], timeout)
                if not ready[0]:
                    return None

            data, address = self.socket.recvfrom(1024)
            return data, address
        except BlockingIOError:
            return None
        except KeyboardInterrupt:
            # Re-raise keyboard interrupt to allow proper signal handling
            raise
        except InterruptedError:
            # System call was interrupted (e.g., by signal), re-raise as KeyboardInterrupt
            raise KeyboardInterrupt("Interrupted by signal")
        except Exception as e:
            print(f"[ERROR] Failed to receive data: {e}")
            return None

    def set_nonblocking(self, nonblocking: bool = True) -> None:
        """Set socket to non-blocking mode."""
        if self.socket:
            self.socket.setblocking(not nonblocking)

    def close(self) -> None:
        """Close UDP socket."""
        if self.socket:
            self.socket.close()
            self.socket = None
        self.is_setup = False


class CommunicationManager:
    """
    Modern communication manager for robot middleware.

    This class provides a clean interface for managing communication with
    multiple robot modules, handling connection management, data serialization,
    and protocol abstraction.
    """

    def __init__(
        self,
        protocol: CommunicationProtocol,
        struct_format: str,
        expected_modules: List[int],
        connection_timeout: float = 5.0,
        max_pending_count: int = 5,
    ):
        """
        Initialize communication manager.

        Args:
            protocol: Communication protocol to use
            struct_format: Format string for data unpacking
            expected_modules: List of expected module IDs
            connection_timeout: Timeout for waiting for connections
            max_pending_count: Maximum pending count before marking module as lost
        """
        self.protocol = protocol
        self.struct_format = struct_format
        self.expected_modules = set(expected_modules)
        self.connection_timeout = connection_timeout
        self.max_pending_count = max_pending_count

        # Module management
        self.modules: Dict[int, ModuleInfo] = {}
        self.cached_pings: Dict[int, float] = {}

        # Callbacks
        self.on_module_connected: Optional[Callable[[int, ModuleInfo], None]] = None
        self.on_module_disconnected: Optional[Callable[[int, ModuleInfo], None]] = None
        self.on_data_received: Optional[Callable[[int, Dict[str, Any]], None]] = None

        # Statistics
        self.stats = {
            "messages_sent": 0,
            "messages_received": 0,
            "connection_errors": 0,
            "data_errors": 0,
            "buffer_overflows": 0,
            "messages_dropped": 0,
            "avg_batch_size": 0.0,
            "max_batch_size": 0,
        }

        # Buffer management
        self.buffer_stats = {"total_batches": 0, "total_messages_in_batches": 0}

        # Timestamp tracking for each module
        self.last_timestamps = {}  # module_id -> last_timestamp
        self.out_of_order_count = 0
        self.stale_message_count = 0

    def setup(self, check_initial_connections: bool = True, **protocol_kwargs) -> bool:
        """
        Setup communication manager.

        Args:
            check_initial_connections: Whether to wait for initial connections
            **protocol_kwargs: Additional arguments for protocol setup

        Returns:
            True if setup successful, False otherwise
        """
        try:
            print("[CommunicationManager] Setting up communication...")
            self.protocol.setup(**protocol_kwargs)

            if check_initial_connections:
                self._wait_for_initial_connections()

            # Set to non-blocking after initial setup
            if hasattr(self.protocol, "set_nonblocking"):
                self.protocol.set_nonblocking(True)

            print("[CommunicationManager] Communication setup complete")
            return True

        except Exception as e:
            print(f"[ERROR][CommunicationManager] Setup failed: {e}")
            return False

    def _wait_for_initial_connections(self) -> None:
        """Wait for all expected modules to connect initially."""
        to_be_connected = copy.deepcopy(self.expected_modules)

        print(f"[CommunicationManager] Waiting for modules {list(to_be_connected)} to connect...")
        print("[CommunicationManager] Press Ctrl+C to abort...")

        while to_be_connected:
            try:
                data_result = self.protocol.receive_data(timeout=self.connection_timeout)

                if data_result is None:
                    print(
                        f"[ERROR][CommunicationManager] Timeout waiting for modules {list(to_be_connected)}"
                    )
                    continue

                data, address = data_result

                try:
                    unpacked_data = struct.unpack(self.struct_format, data)
                    module_id = unpacked_data[0]

                    # Always print debug info for received data
                    print(f"[DEBUG][CommunicationManager] Received data from module {module_id} at {address}")

                    if module_id in to_be_connected:
                        self._register_module(module_id, address)
                        to_be_connected.remove(module_id)
                        print(f"[CommunicationManager] Module {module_id} connected from {address}")
                    else:
                        print(f"[DEBUG][CommunicationManager] Module {module_id} is not in expected list {list(to_be_connected)}")

                except Exception as e:
                    print(f"[ERROR][CommunicationManager] Failed to process initial connection: {e}")
                    self.stats["data_errors"] += 1

            except KeyboardInterrupt:
                print("\n[CommunicationManager] Connection wait interrupted by user (Ctrl+C)")
                raise

    def _register_module(self, module_id: int, address: Tuple[str, int]) -> None:
        """Register a new module or update existing one."""
        current_time = time.time()

        if module_id not in self.modules:
            # Get ping time if not cached
            if module_id not in self.cached_pings:
                # self.cached_pings[module_id] = get_ping_time(address[0])
                cache_pings(self.cached_pings)

            # Create new module info
            self.modules[module_id] = ModuleInfo(
                module_id=module_id,
                address=address,
                status=ConnectionStatus.CONNECTED,
                last_seen=current_time,
                ping_time=self.cached_pings.get(module_id),
                pending_count=0,
            )

            if self.on_module_connected:
                self.on_module_connected(module_id, self.modules[module_id])
        else:
            # Update existing module
            module_info = self.modules[module_id]
            module_info.address = address  # Allow IP changes
            module_info.status = ConnectionStatus.CONNECTED
            module_info.last_seen = current_time
            module_info.pending_count = 0

    def send_command(self, module_id: int, command_data) -> bool:
        """
        Send command to a specific module.

        Args:
            module_id: Target module ID
            command_data: Command data to send (SentDataStruct or generated MotorCommand)

        Returns:
            True if sent successfully, False otherwise
        """
        if module_id not in self.modules:
            print(f"[ERROR][CommunicationManager] Module {module_id} not connected")
            return False

        module_info = self.modules[module_id]
        
        # Support both old and new message types
        if hasattr(command_data, 'serialize'):
            serialized_data = command_data.serialize()
        else:
            # Legacy support - assume it's the old SentDataStruct
            serialized_data = command_data.serialize()

        success = self.protocol.send_data(serialized_data, module_info.address)

        if success:
            self.stats["messages_sent"] += 1
        else:
            self.stats["connection_errors"] += 1

        return success

    def receive_data_batch(
        self,
        max_messages: int = 100,
        timeout_ms: float = 0.0,
        keep_latest_only: bool = True,
        priority_modules: Optional[List[int]] = None,
        check_timestamps: bool = False,
        max_age_ms: float = 100.0,
    ) -> Dict[int, Dict[str, Any]]:
        """
        Improved batch data reception with better buffer management.

        Args:
            max_messages: Maximum number of messages to process per batch
            timeout_ms: Timeout in milliseconds for the entire batch operation
            keep_latest_only: If True, keep only the latest message per module
            priority_modules: List of module IDs to process first
            check_timestamps: If True, validate message timestamps and detect out-of-order
            max_age_ms: Maximum age of messages to accept (in milliseconds)

        Returns:
            Dictionary mapping module_id to processed data
        """
        import time

        start_time = time.time()
        timeout_seconds = timeout_ms / 1000.0

        received_data = {}
        expected_modules = copy.deepcopy(self.expected_modules)
        messages_processed = 0
        buffer_overflow_detected = False

        # Temporary storage for multiple messages per module
        all_module_messages = {} if not keep_latest_only else None

        # Phase 1: Drain the UDP buffer efficiently
        while messages_processed < max_messages:
            # Check timeout
            if timeout_seconds > 0 and (time.time() - start_time) > timeout_seconds:
                break

            data_result = self.protocol.receive_data(timeout=0.0)

            if data_result is None:
                break  # No more data available

            data, address = data_result
            messages_processed += 1

            try:
                # Unpack and process data
                robot_data = RobotData.unpack(data, self.struct_format)
                module_id = robot_data.module_id
                data_dict = robot_data.get_data_dict()
                print(f"[DEBUG][CommunicationManager] Received data from module {module_id}")
                # print(f"[DEBUG][CommunicationManager] Data: {data_dict}")

                # Update module info
                self._register_module(module_id, address)

                # Timestamp validation if requested
                should_process_message = True
                if check_timestamps:
                    should_process_message = self._validate_message_timestamp(
                        module_id, data_dict, max_age_ms
                    )

                if should_process_message:
                    if keep_latest_only:
                        # Keep only the latest message per module
                        received_data[module_id] = data_dict
                    else:
                        # Store all messages per module
                        if module_id not in all_module_messages:
                            all_module_messages[module_id] = []
                        all_module_messages[module_id].append(data_dict)
                else:
                    # Message was rejected due to timestamp issues
                    continue

                # Remove from expected list
                expected_modules.discard(module_id)

                # Call callback if set
                if self.on_data_received:
                    self.on_data_received(module_id, data_dict)

                self.stats["messages_received"] += 1

            except Exception as e:
                print(f"[ERROR][CommunicationManager] Failed to process data from {address}: {e}")
                self.stats["data_errors"] += 1

        print(f"[DEBUG][CommunicationManager] Processed {messages_processed} messages in batch")
        # Check for buffer overflow
        if messages_processed >= max_messages:
            # Check if there are still more messages
            overflow_check = self.protocol.receive_data(timeout=0.0)
            if overflow_check is not None:
                buffer_overflow_detected = True
                self.stats["buffer_overflows"] += 1

                # Count how many messages we're dropping
                dropped_count = 1  # We already found one
                while True:
                    extra_data = self.protocol.receive_data(timeout=0.0)
                    if extra_data is None:
                        break
                    dropped_count += 1
                    if dropped_count > 1000:  # Safety limit
                        break

                self.stats["messages_dropped"] += dropped_count
                print(
                    f"[WARNING][CommunicationManager] Buffer overflow detected! Dropped {dropped_count} messages"
                )

        # Phase 2: Process priority modules if specified
        if priority_modules and not keep_latest_only:
            priority_data = {}
            for module_id in priority_modules:
                if module_id in all_module_messages:
                    # For priority modules, use the latest message
                    priority_data[module_id] = all_module_messages[module_id][-1]
            received_data = priority_data
        elif not keep_latest_only:
            # Convert all messages to latest messages
            for module_id, messages in all_module_messages.items():
                received_data[module_id] = messages[-1]  # Keep latest

        # Update statistics
        self.buffer_stats["total_batches"] += 1
        self.buffer_stats["total_messages_in_batches"] += messages_processed

        if messages_processed > self.stats["max_batch_size"]:
            self.stats["max_batch_size"] = messages_processed

        # Update average batch size
        self.stats["avg_batch_size"] = (
            self.buffer_stats["total_messages_in_batches"] / self.buffer_stats["total_batches"]
        )

        # Update pending counts for modules we didn't hear from
        self._update_pending_modules(expected_modules)

        # Log performance info if there are issues
        if buffer_overflow_detected:
            print(
                f"[PERFORMANCE] Batch stats - Processed: {messages_processed}, "
                f"Expected modules: {len(expected_modules)}, "
                f"Received modules: {len(received_data)}"
            )

        return received_data

    def receive_data_batch_legacy(self, max_messages: int = 100) -> Dict[int, Dict[str, Any]]:
        """
        Legacy version of receive_data_batch for backward compatibility.

        This maintains the original behavior for existing code.
        """
        return self.receive_data_batch(
            max_messages=max_messages, timeout_ms=0.0, keep_latest_only=True, priority_modules=None
        )

    def _validate_message_timestamp(
        self, module_id: int, data_dict: Dict[str, Any], max_age_ms: float
    ) -> bool:
        """
        Validate message timestamp for freshness and ordering.

        Args:
            module_id: ID of the module
            data_dict: Message data containing timestamp
            max_age_ms: Maximum acceptable message age in milliseconds

        Returns:
            True if message should be processed, False if it should be rejected
        """
        import time

        # Extract timestamp from message
        message_timestamp = data_dict.get("timestamp")
        if message_timestamp is None:
            # No timestamp available, accept the message
            return True

        current_time_ms = time.time() * 1000

        # Check message age (freshness)
        # Note: Your timestamps appear to be in ESP32 ticks, not Unix time
        # We'll focus on relative ordering rather than absolute age

        # Check for out-of-order delivery
        if module_id in self.last_timestamps:
            last_timestamp = self.last_timestamps[module_id]

            if message_timestamp < last_timestamp:
                # Out of order message detected
                self.out_of_order_count += 1
                print(
                    f"[WARNING][CommunicationManager] Out-of-order message from module {module_id}: "
                    f"current={message_timestamp}, last={last_timestamp}"
                )

                # For real-time control, reject old messages
                return False
            elif message_timestamp == last_timestamp:
                # Duplicate message
                print(
                    f"[WARNING][CommunicationManager] Duplicate message from module {module_id}: "
                    f"timestamp={message_timestamp}"
                )
                return False

        # Update last timestamp for this module
        self.last_timestamps[module_id] = message_timestamp

        # Message passed all checks
        return True

    def get_timestamp_statistics(self) -> Dict[str, Any]:
        """Get timestamp-related statistics."""
        return {
            "out_of_order_messages": self.out_of_order_count,
            "stale_messages": self.stale_message_count,
            "modules_tracked": len(self.last_timestamps),
            "last_timestamps": self.last_timestamps.copy(),
        }

    def _update_pending_modules(self, missing_modules: set) -> None:
        """Update pending status for modules we didn't hear from."""
        for module_id in missing_modules:
            if module_id in self.modules:
                module_info = self.modules[module_id]
                module_info.pending_count += 1

                if module_info.pending_count > self.max_pending_count:
                    if module_info.status != ConnectionStatus.LOST:
                        module_info.status = ConnectionStatus.LOST
                        print(f"[ERROR][CommunicationManager] Module {module_id} lost connection")

                        if self.on_module_disconnected:
                            self.on_module_disconnected(module_id, module_info)
                else:
                    module_info.status = ConnectionStatus.PENDING

    def get_module_status(self, module_id: int) -> Optional[ConnectionStatus]:
        """Get status of a specific module."""
        if module_id in self.modules:
            return self.modules[module_id].status
        return None

    def get_connected_modules(self) -> List[int]:
        """Get list of currently connected module IDs."""
        return [
            module_id
            for module_id, info in self.modules.items()
            if info.status == ConnectionStatus.CONNECTED
        ]

    def get_module_info(self, module_id: int) -> Optional[ModuleInfo]:
        """Get detailed info about a module."""
        return self.modules.get(module_id)

    def get_all_modules_info(self) -> Dict[int, ModuleInfo]:
        """Get info about all modules."""
        return copy.deepcopy(self.modules)

    def get_buffer_health(self) -> Dict[str, Any]:
        """Get buffer health information."""
        total_batches = self.buffer_stats["total_batches"]
        if total_batches == 0:
            return {
                "status": "no_data",
                "avg_batch_size": 0.0,
                "max_batch_size": 0,
                "overflow_rate": 0.0,
                "drop_rate": 0.0,
                "recommendation": "No data processed yet",
            }

        overflow_rate = self.stats["buffer_overflows"] / total_batches
        drop_rate = self.stats["messages_dropped"] / max(self.stats["messages_received"], 1)

        # Determine buffer health status
        if overflow_rate > 0.1:  # More than 10% overflow rate
            status = "unhealthy"
            recommendation = "Increase max_messages or reduce data rate"
        elif overflow_rate > 0.05:  # More than 5% overflow rate
            status = "degraded"
            recommendation = "Consider increasing max_messages"
        elif self.stats["avg_batch_size"] > 80:  # Near max capacity
            status = "near_capacity"
            recommendation = "Monitor closely, consider optimization"
        else:
            status = "healthy"
            recommendation = "Buffer performing well"

        return {
            "status": status,
            "avg_batch_size": self.stats["avg_batch_size"],
            "max_batch_size": self.stats["max_batch_size"],
            "overflow_rate": overflow_rate,
            "drop_rate": drop_rate,
            "total_overflows": self.stats["buffer_overflows"],
            "total_dropped": self.stats["messages_dropped"],
            "recommendation": recommendation,
        }

    def optimize_batch_size(self) -> int:
        """Suggest optimal batch size based on current performance."""
        health = self.get_buffer_health()

        if health["status"] == "unhealthy":
            # Increase batch size significantly
            return min(int(self.stats["max_batch_size"] * 1.5), 500)
        elif health["status"] == "degraded":
            # Increase batch size moderately
            return min(int(self.stats["max_batch_size"] * 1.2), 300)
        elif health["avg_batch_size"] < 10:
            # Decrease batch size to reduce overhead
            return max(int(self.stats["avg_batch_size"] * 2), 20)
        else:
            # Current size seems good
            return 100  # Default

    def clear_buffer_stats(self) -> None:
        """Clear buffer statistics (useful for testing or reset)."""
        self.stats.update(
            {
                "buffer_overflows": 0,
                "messages_dropped": 0,
                "avg_batch_size": 0.0,
                "max_batch_size": 0,
            }
        )
        self.buffer_stats.update({"total_batches": 0, "total_messages_in_batches": 0})

    def get_statistics(self) -> Dict[str, Any]:
        """Get communication statistics."""
        return {
            **self.stats,
            "connected_modules": len(self.get_connected_modules()),
            "total_modules": len(self.modules),
            "expected_modules": len(self.expected_modules),
            "buffer_health": self.get_buffer_health(),
            "timestamp_stats": self.get_timestamp_statistics(),
        }

    def close(self) -> None:
        """Close communication manager and cleanup resources."""
        print("[CommunicationManager] Closing communication...")
        self.protocol.close()
        self.modules.clear()
