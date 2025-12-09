"""
ROS2-like Publisher/Subscriber/Topic/Node system for Capybarish.

This module provides a familiar ROS2-style API for inter-process and 
intra-process communication using topics, publishers, and subscribers.

Example Usage:
    ```python
    import capybarish as cpy
    from capybarish.generated import ReceivedData, SentData
    
    # Create a node
    node = cpy.Node('motor_controller')
    
    # Create publisher and subscriber
    pub = node.create_publisher(ReceivedData, '/motor/command', qos_depth=10)
    sub = node.create_subscription(SentData, '/motor/feedback', callback, qos_depth=10)
    
    # Publish messages
    msg = ReceivedData(target=1.5, target_vel=0.0, kp=10.0, kd=0.5)
    pub.publish(msg)
    
    # Spin to process callbacks
    cpy.spin(node)
    ```

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>
Licensed under the Apache License, Version 2.0
"""

import queue
import socket
import struct
import threading
import time
import weakref
from abc import ABC, abstractmethod
from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Set,
    Tuple,
    Type,
    TypeVar,
    Union,
)

# Type variable for message types
MsgT = TypeVar('MsgT')


# =============================================================================
# QoS (Quality of Service) Settings
# =============================================================================

class QoSReliabilityPolicy(Enum):
    """Reliability policy for message delivery."""
    RELIABLE = auto()      # Guarantee delivery (with retries)
    BEST_EFFORT = auto()   # No guarantee, lowest latency


class QoSHistoryPolicy(Enum):
    """History policy for message queue."""
    KEEP_LAST = auto()     # Keep last N messages
    KEEP_ALL = auto()      # Keep all messages (unbounded)


class QoSDurabilityPolicy(Enum):
    """Durability policy for late subscribers."""
    VOLATILE = auto()      # No persistence
    TRANSIENT_LOCAL = auto()  # Keep last message for late joiners


@dataclass
class QoSProfile:
    """Quality of Service profile for publishers and subscribers.
    
    Similar to ROS2 QoS profiles, this controls message delivery behavior.
    """
    reliability: QoSReliabilityPolicy = QoSReliabilityPolicy.RELIABLE
    history: QoSHistoryPolicy = QoSHistoryPolicy.KEEP_LAST
    depth: int = 10  # Queue depth for KEEP_LAST
    durability: QoSDurabilityPolicy = QoSDurabilityPolicy.VOLATILE
    
    @classmethod
    def sensor_data(cls) -> 'QoSProfile':
        """QoS profile optimized for sensor data (best effort, small queue)."""
        return cls(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
    
    @classmethod
    def default(cls) -> 'QoSProfile':
        """Default QoS profile (reliable, depth 10)."""
        return cls()
    
    @classmethod
    def services(cls) -> 'QoSProfile':
        """QoS profile for service calls (reliable)."""
        return cls(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
    
    @classmethod
    def parameters(cls) -> 'QoSProfile':
        """QoS profile for parameters (reliable, transient local)."""
        return cls(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )


# =============================================================================
# Topic Manager (Global Registry)
# =============================================================================

class TopicManager:
    """Global topic registry and message router.
    
    This singleton manages all topics and routes messages between
    publishers and subscribers. Supports both intra-process (direct)
    and inter-process (UDP) communication.
    """
    
    _instance: Optional['TopicManager'] = None
    _lock = threading.Lock()
    
    def __new__(cls) -> 'TopicManager':
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        
        self._topics: Dict[str, 'Topic'] = {}
        self._nodes: Dict[str, 'Node'] = {}
        self._topic_lock = threading.Lock()
        self._node_lock = threading.Lock()
        
        # Network settings for inter-process communication
        self._multicast_group = '239.255.0.1'
        self._base_port = 7000
        self._port_map: Dict[str, int] = {}  # topic_name -> port
        self._next_port = self._base_port
        
        self._initialized = True
    
    def get_or_create_topic(
        self, 
        name: str, 
        msg_type: Type[MsgT],
        qos: Optional[QoSProfile] = None
    ) -> 'Topic[MsgT]':
        """Get existing topic or create new one."""
        with self._topic_lock:
            if name not in self._topics:
                qos = qos or QoSProfile.default()
                port = self._allocate_port(name)
                self._topics[name] = Topic(name, msg_type, qos, port)
            return self._topics[name]
    
    def _allocate_port(self, topic_name: str) -> int:
        """Allocate a unique port for a topic."""
        if topic_name not in self._port_map:
            self._port_map[topic_name] = self._next_port
            self._next_port += 1
        return self._port_map[topic_name]
    
    def register_node(self, node: 'Node') -> None:
        """Register a node with the manager."""
        with self._node_lock:
            if node.full_name in self._nodes:
                raise ValueError(f"Node '{node.full_name}' already exists")
            self._nodes[node.full_name] = node
    
    def unregister_node(self, node: 'Node') -> None:
        """Unregister a node from the manager."""
        with self._node_lock:
            self._nodes.pop(node.full_name, None)
    
    def get_topic_names(self) -> List[str]:
        """Get list of all registered topic names."""
        with self._topic_lock:
            return list(self._topics.keys())
    
    def get_node_names(self) -> List[str]:
        """Get list of all registered node names."""
        with self._node_lock:
            return list(self._nodes.keys())
    
    def get_topic_info(self, topic_name: str) -> Optional[Dict[str, Any]]:
        """Get information about a topic."""
        with self._topic_lock:
            if topic_name in self._topics:
                topic = self._topics[topic_name]
                return {
                    'name': topic.name,
                    'type': topic.msg_type.__name__,
                    'publishers': len(topic._publishers),
                    'subscribers': len(topic._subscribers),
                    'port': topic._port,
                }
        return None
    
    @classmethod
    def reset(cls) -> None:
        """Reset the singleton (for testing)."""
        with cls._lock:
            if cls._instance is not None:
                cls._instance._topics.clear()
                cls._instance._nodes.clear()
            cls._instance = None


# =============================================================================
# Topic
# =============================================================================

class Topic(Generic[MsgT]):
    """A named channel for message passing.
    
    Topics connect publishers to subscribers. Messages published to a topic
    are delivered to all subscribers of that topic.
    """
    
    def __init__(
        self,
        name: str,
        msg_type: Type[MsgT],
        qos: QoSProfile,
        port: int,
    ):
        self.name = name
        self.msg_type = msg_type
        self.qos = qos
        self._port = port
        
        self._publishers: List[weakref.ref] = []
        self._subscribers: List[weakref.ref] = []
        self._lock = threading.Lock()
        
        # Last message for transient local durability
        self._last_message: Optional[MsgT] = None
        
        # Statistics
        self._msg_count = 0
        self._last_pub_time: Optional[float] = None
    
    def add_publisher(self, pub: 'Publisher[MsgT]') -> None:
        """Register a publisher to this topic."""
        with self._lock:
            self._publishers.append(weakref.ref(pub))
    
    def remove_publisher(self, pub: 'Publisher[MsgT]') -> None:
        """Unregister a publisher from this topic."""
        with self._lock:
            self._publishers = [
                ref for ref in self._publishers 
                if ref() is not None and ref() is not pub
            ]
    
    def add_subscriber(self, sub: 'Subscription[MsgT]') -> None:
        """Register a subscriber to this topic."""
        with self._lock:
            self._subscribers.append(weakref.ref(sub))
            
            # Send last message if durability is transient local
            if (self.qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL 
                and self._last_message is not None):
                sub._enqueue(self._last_message)
    
    def remove_subscriber(self, sub: 'Subscription[MsgT]') -> None:
        """Unregister a subscriber from this topic."""
        with self._lock:
            self._subscribers = [
                ref for ref in self._subscribers
                if ref() is not None and ref() is not sub
            ]
    
    def publish(self, msg: MsgT) -> int:
        """Publish a message to all subscribers.
        
        Returns:
            Number of subscribers that received the message.
        """
        delivered = 0
        with self._lock:
            self._msg_count += 1
            self._last_pub_time = time.time()
            
            if self.qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL:
                self._last_message = msg
            
            # Clean up dead references and deliver
            live_subs = []
            for ref in self._subscribers:
                sub = ref()
                if sub is not None:
                    live_subs.append(ref)
                    sub._enqueue(msg)
                    delivered += 1
            self._subscribers = live_subs
        
        return delivered
    
    @property
    def publisher_count(self) -> int:
        """Get number of active publishers."""
        with self._lock:
            self._publishers = [ref for ref in self._publishers if ref() is not None]
            return len(self._publishers)
    
    @property
    def subscriber_count(self) -> int:
        """Get number of active subscribers."""
        with self._lock:
            self._subscribers = [ref for ref in self._subscribers if ref() is not None]
            return len(self._subscribers)


# =============================================================================
# Publisher
# =============================================================================

class Publisher(Generic[MsgT]):
    """Publishes messages to a topic.
    
    Example:
        ```python
        pub = node.create_publisher(ReceivedData, '/motor/command', qos_depth=10)
        msg = ReceivedData(target=1.5, target_vel=0.0)
        pub.publish(msg)
        ```
    """
    
    def __init__(
        self,
        node: 'Node',
        msg_type: Type[MsgT],
        topic_name: str,
        qos: QoSProfile,
    ):
        self._node = node
        self._msg_type = msg_type
        self._topic_name = topic_name
        self._qos = qos
        
        # Get or create the topic
        self._topic = TopicManager().get_or_create_topic(topic_name, msg_type, qos)
        self._topic.add_publisher(self)
        
        # Statistics
        self._pub_count = 0
        self._last_pub_time: Optional[float] = None
        
        # Network publisher for inter-process (optional)
        self._udp_socket: Optional[socket.socket] = None
        self._remote_endpoints: List[Tuple[str, int]] = []
    
    @property
    def topic_name(self) -> str:
        """Get the topic name."""
        return self._topic_name
    
    @property
    def msg_type(self) -> Type[MsgT]:
        """Get the message type."""
        return self._msg_type
    
    def publish(self, msg: MsgT) -> None:
        """Publish a message to the topic.
        
        Args:
            msg: Message to publish (must match the publisher's message type)
        """
        if not isinstance(msg, self._msg_type):
            raise TypeError(
                f"Expected message type {self._msg_type.__name__}, "
                f"got {type(msg).__name__}"
            )
        
        self._pub_count += 1
        self._last_pub_time = time.time()
        
        # Local delivery
        self._topic.publish(msg)
        
        # Network delivery (if configured)
        if self._udp_socket and self._remote_endpoints:
            self._publish_network(msg)
    
    def _publish_network(self, msg: MsgT) -> None:
        """Publish message over network."""
        if hasattr(msg, 'serialize'):
            data = msg.serialize()
            for endpoint in self._remote_endpoints:
                try:
                    self._udp_socket.sendto(data, endpoint)
                except OSError:
                    pass  # Ignore network errors in best-effort mode
    
    def add_remote_endpoint(self, host: str, port: int) -> None:
        """Add a remote endpoint for network publishing."""
        if self._udp_socket is None:
            self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._remote_endpoints.append((host, port))
    
    def get_subscription_count(self) -> int:
        """Get number of subscribers to this topic."""
        return self._topic.subscriber_count
    
    def destroy(self) -> None:
        """Clean up the publisher."""
        self._topic.remove_publisher(self)
        if self._udp_socket:
            self._udp_socket.close()
            self._udp_socket = None


# =============================================================================
# Subscription
# =============================================================================

class Subscription(Generic[MsgT]):
    """Subscribes to messages from a topic.
    
    Example:
        ```python
        def callback(msg: SentData):
            print(f"Received: pos={msg.motor.position}")
        
        sub = node.create_subscription(SentData, '/motor/feedback', callback)
        ```
    """
    
    def __init__(
        self,
        node: 'Node',
        msg_type: Type[MsgT],
        topic_name: str,
        callback: Callable[[MsgT], None],
        qos: QoSProfile,
    ):
        self._node = node
        self._msg_type = msg_type
        self._topic_name = topic_name
        self._callback = callback
        self._qos = qos
        
        # Message queue
        if qos.history == QoSHistoryPolicy.KEEP_ALL:
            self._queue: queue.Queue[MsgT] = queue.Queue()
        else:
            self._queue = queue.Queue(maxsize=qos.depth)
        
        # Get or create the topic
        self._topic = TopicManager().get_or_create_topic(topic_name, msg_type, qos)
        self._topic.add_subscriber(self)
        
        # Statistics
        self._recv_count = 0
        self._drop_count = 0
        self._last_recv_time: Optional[float] = None
        
        # Network subscriber (optional)
        self._udp_socket: Optional[socket.socket] = None
        self._network_thread: Optional[threading.Thread] = None
        self._running = False
    
    @property
    def topic_name(self) -> str:
        """Get the topic name."""
        return self._topic_name
    
    @property
    def msg_type(self) -> Type[MsgT]:
        """Get the message type."""
        return self._msg_type
    
    def _enqueue(self, msg: MsgT) -> None:
        """Add message to the queue (called by Topic)."""
        try:
            if self._qos.history == QoSHistoryPolicy.KEEP_LAST:
                # Non-blocking put, drop oldest if full
                if self._queue.full():
                    try:
                        self._queue.get_nowait()
                        self._drop_count += 1
                    except queue.Empty:
                        pass
            self._queue.put_nowait(msg)
            self._recv_count += 1
            self._last_recv_time = time.time()
        except queue.Full:
            self._drop_count += 1
    
    def take(self, timeout: Optional[float] = None) -> Optional[MsgT]:
        """Take a message from the queue.
        
        Args:
            timeout: Max time to wait (None for non-blocking)
            
        Returns:
            Message or None if queue is empty
        """
        try:
            return self._queue.get(timeout=timeout) if timeout else self._queue.get_nowait()
        except queue.Empty:
            return None
    
    def take_all(self) -> List[MsgT]:
        """Take all messages from the queue."""
        messages = []
        while True:
            try:
                messages.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return messages
    
    def process_one(self) -> bool:
        """Process one message from the queue.
        
        Returns:
            True if a message was processed, False if queue was empty
        """
        msg = self.take()
        if msg is not None:
            self._callback(msg)
            return True
        return False
    
    def process_all(self) -> int:
        """Process all pending messages.
        
        Returns:
            Number of messages processed
        """
        count = 0
        for msg in self.take_all():
            self._callback(msg)
            count += 1
        return count
    
    def bind_network(self, host: str = '0.0.0.0', port: Optional[int] = None) -> None:
        """Bind to network for receiving messages from remote publishers.
        
        Args:
            host: Host to bind to
            port: Port to bind to (uses topic's port if not specified)
        """
        if self._udp_socket is not None:
            return
        
        port = port or self._topic._port
        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_socket.bind((host, port))
        self._udp_socket.settimeout(0.1)
        
        self._running = True
        self._network_thread = threading.Thread(target=self._network_loop, daemon=True)
        self._network_thread.start()
    
    def _network_loop(self) -> None:
        """Background thread for receiving network messages."""
        while self._running:
            try:
                data, addr = self._udp_socket.recvfrom(4096)
                if hasattr(self._msg_type, 'deserialize'):
                    msg = self._msg_type.deserialize(data)
                    self._enqueue(msg)
            except socket.timeout:
                continue
            except Exception:
                continue
    
    def get_publisher_count(self) -> int:
        """Get number of publishers to this topic."""
        return self._topic.publisher_count
    
    @property
    def pending_count(self) -> int:
        """Get number of pending messages in queue."""
        return self._queue.qsize()
    
    def destroy(self) -> None:
        """Clean up the subscription."""
        self._running = False
        if self._network_thread:
            self._network_thread.join(timeout=1.0)
        if self._udp_socket:
            self._udp_socket.close()
            self._udp_socket = None
        self._topic.remove_subscriber(self)


# =============================================================================
# Timer
# =============================================================================

class Timer:
    """Periodic timer that triggers a callback.
    
    Example:
        ```python
        def timer_callback():
            print("Timer fired!")
        
        timer = node.create_timer(0.1, timer_callback)  # 10 Hz
        ```
    """
    
    def __init__(
        self,
        node: 'Node',
        period_sec: float,
        callback: Callable[[], None],
    ):
        self._node = node
        self._period = period_sec
        self._callback = callback
        
        self._last_call = time.time()
        self._call_count = 0
        self._active = True
    
    @property
    def period(self) -> float:
        """Get timer period in seconds."""
        return self._period
    
    @property
    def is_ready(self) -> bool:
        """Check if timer is ready to fire."""
        return self._active and (time.time() - self._last_call) >= self._period
    
    def fire(self) -> None:
        """Fire the timer callback."""
        if self._active:
            self._last_call = time.time()
            self._call_count += 1
            self._callback()
    
    def reset(self) -> None:
        """Reset the timer."""
        self._last_call = time.time()
    
    def cancel(self) -> None:
        """Cancel the timer."""
        self._active = False
    
    def destroy(self) -> None:
        """Destroy the timer."""
        self.cancel()


# =============================================================================
# Node
# =============================================================================

class Node:
    """A computational unit that can publish/subscribe to topics.
    
    Nodes are the primary entity in the pub/sub system. Each node can have
    multiple publishers, subscribers, and timers.
    
    Example:
        ```python
        node = Node('motor_controller')
        
        pub = node.create_publisher(ReceivedData, '/motor/command')
        sub = node.create_subscription(SentData, '/motor/feedback', my_callback)
        timer = node.create_timer(0.01, control_loop)  # 100 Hz
        ```
    """
    
    def __init__(self, name: str, *, namespace: str = ''):
        """Create a new node.
        
        Args:
            name: Node name (must be unique)
            namespace: Optional namespace prefix for topics
        """
        self._name = name
        self._namespace = namespace
        self._full_name = f"{namespace}/{name}" if namespace else name
        
        self._publishers: List[Publisher] = []
        self._subscriptions: List[Subscription] = []
        self._timers: List[Timer] = []
        
        self._lock = threading.Lock()
        self._context = TopicManager()
        self._context.register_node(self)
        
        self._logger = NodeLogger(self._full_name)
    
    @property
    def name(self) -> str:
        """Get the node name."""
        return self._name
    
    @property
    def namespace(self) -> str:
        """Get the node namespace."""
        return self._namespace
    
    @property
    def full_name(self) -> str:
        """Get the fully qualified node name."""
        return self._full_name
    
    def get_logger(self) -> 'NodeLogger':
        """Get the node's logger."""
        return self._logger
    
    def create_publisher(
        self,
        msg_type: Type[MsgT],
        topic: str,
        qos_depth: int = 10,
        qos_profile: Optional[QoSProfile] = None,
    ) -> Publisher[MsgT]:
        """Create a publisher for a topic.
        
        Args:
            msg_type: Message type class
            topic: Topic name
            qos_depth: Queue depth (shorthand for qos_profile)
            qos_profile: Full QoS profile (overrides qos_depth)
            
        Returns:
            Publisher instance
        """
        if qos_profile is None:
            qos_profile = QoSProfile(depth=qos_depth)
        
        # Apply namespace
        full_topic = self._resolve_topic_name(topic)
        
        pub = Publisher(self, msg_type, full_topic, qos_profile)
        with self._lock:
            self._publishers.append(pub)
        
        self._logger.debug(f"Created publisher: {full_topic} [{msg_type.__name__}]")
        return pub
    
    def create_subscription(
        self,
        msg_type: Type[MsgT],
        topic: str,
        callback: Callable[[MsgT], None],
        qos_depth: int = 10,
        qos_profile: Optional[QoSProfile] = None,
    ) -> Subscription[MsgT]:
        """Create a subscription to a topic.
        
        Args:
            msg_type: Message type class
            topic: Topic name
            callback: Function to call when message is received
            qos_depth: Queue depth (shorthand for qos_profile)
            qos_profile: Full QoS profile (overrides qos_depth)
            
        Returns:
            Subscription instance
        """
        if qos_profile is None:
            qos_profile = QoSProfile(depth=qos_depth)
        
        # Apply namespace
        full_topic = self._resolve_topic_name(topic)
        
        sub = Subscription(self, msg_type, full_topic, callback, qos_profile)
        with self._lock:
            self._subscriptions.append(sub)
        
        self._logger.debug(f"Created subscription: {full_topic} [{msg_type.__name__}]")
        return sub
    
    def create_timer(
        self,
        period_sec: float,
        callback: Callable[[], None],
    ) -> Timer:
        """Create a periodic timer.
        
        Args:
            period_sec: Timer period in seconds
            callback: Function to call when timer fires
            
        Returns:
            Timer instance
        """
        timer = Timer(self, period_sec, callback)
        with self._lock:
            self._timers.append(timer)
        
        self._logger.debug(f"Created timer: period={period_sec}s")
        return timer
    
    def _resolve_topic_name(self, topic: str) -> str:
        """Resolve topic name with namespace."""
        if topic.startswith('/'):
            return topic  # Absolute topic name
        elif self._namespace:
            return f"/{self._namespace}/{topic}"
        else:
            return f"/{topic}"
    
    def spin_once(self, timeout_sec: float = 0.0) -> int:
        """Process pending callbacks once.
        
        Args:
            timeout_sec: Max time to wait for messages
            
        Returns:
            Number of callbacks executed
        """
        count = 0
        
        # Process subscriptions
        with self._lock:
            subs = list(self._subscriptions)
        
        for sub in subs:
            count += sub.process_all()
        
        # Process timers
        with self._lock:
            timers = list(self._timers)
        
        for timer in timers:
            if timer.is_ready:
                timer.fire()
                count += 1
        
        return count
    
    def destroy(self) -> None:
        """Destroy the node and clean up resources."""
        with self._lock:
            for pub in self._publishers:
                pub.destroy()
            for sub in self._subscriptions:
                sub.destroy()
            for timer in self._timers:
                timer.destroy()
            
            self._publishers.clear()
            self._subscriptions.clear()
            self._timers.clear()
        
        self._context.unregister_node(self)
        self._logger.info("Node destroyed")
    
    def __enter__(self) -> 'Node':
        return self
    
    def __exit__(self, *args) -> None:
        self.destroy()


# =============================================================================
# Logger
# =============================================================================

class LogLevel(Enum):
    """Log levels."""
    DEBUG = 10
    INFO = 20
    WARN = 30
    ERROR = 40
    FATAL = 50


class NodeLogger:
    """Logger for a node (similar to ROS2 logging)."""
    
    _level = LogLevel.INFO
    
    def __init__(self, node_name: str):
        self._node_name = node_name
    
    @classmethod
    def set_level(cls, level: LogLevel) -> None:
        """Set global log level."""
        cls._level = level
    
    def _log(self, level: LogLevel, msg: str) -> None:
        if level.value >= self._level.value:
            timestamp = time.strftime('%H:%M:%S')
            print(f"[{timestamp}] [{level.name}] [{self._node_name}]: {msg}")
    
    def debug(self, msg: str) -> None:
        self._log(LogLevel.DEBUG, msg)
    
    def info(self, msg: str) -> None:
        self._log(LogLevel.INFO, msg)
    
    def warn(self, msg: str) -> None:
        self._log(LogLevel.WARN, msg)
    
    def error(self, msg: str) -> None:
        self._log(LogLevel.ERROR, msg)
    
    def fatal(self, msg: str) -> None:
        self._log(LogLevel.FATAL, msg)


# =============================================================================
# Executor (Spin Functions)
# =============================================================================

class SingleThreadedExecutor:
    """Single-threaded executor for processing node callbacks."""
    
    def __init__(self):
        self._nodes: List[Node] = []
        self._running = False
    
    def add_node(self, node: Node) -> None:
        """Add a node to the executor."""
        self._nodes.append(node)
    
    def remove_node(self, node: Node) -> None:
        """Remove a node from the executor."""
        if node in self._nodes:
            self._nodes.remove(node)
    
    def spin_once(self, timeout_sec: float = 0.0) -> int:
        """Process pending callbacks once across all nodes."""
        count = 0
        for node in self._nodes:
            count += node.spin_once(timeout_sec)
        return count
    
    def spin(self) -> None:
        """Spin until shutdown."""
        self._running = True
        while self._running:
            self.spin_once()
            time.sleep(0.001)  # Prevent busy loop
    
    def shutdown(self) -> None:
        """Shutdown the executor."""
        self._running = False


class MultiThreadedExecutor:
    """Multi-threaded executor for parallel callback processing."""
    
    def __init__(self, num_threads: int = 4):
        self._nodes: List[Node] = []
        self._running = False
        self._num_threads = num_threads
        self._threads: List[threading.Thread] = []
    
    def add_node(self, node: Node) -> None:
        """Add a node to the executor."""
        self._nodes.append(node)
    
    def remove_node(self, node: Node) -> None:
        """Remove a node from the executor."""
        if node in self._nodes:
            self._nodes.remove(node)
    
    def _worker(self, node: Node) -> None:
        """Worker thread for a node."""
        while self._running:
            node.spin_once()
            time.sleep(0.001)
    
    def spin(self) -> None:
        """Spin until shutdown with multiple threads."""
        self._running = True
        
        # Create one thread per node
        for node in self._nodes:
            thread = threading.Thread(target=self._worker, args=(node,), daemon=True)
            thread.start()
            self._threads.append(thread)
        
        # Wait for shutdown
        try:
            while self._running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.shutdown()
    
    def shutdown(self) -> None:
        """Shutdown the executor."""
        self._running = False
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()


# =============================================================================
# Global Functions (ROS2-like API)
# =============================================================================

_default_executor: Optional[SingleThreadedExecutor] = None
_shutdown_flag = False


def init() -> None:
    """Initialize the pub/sub system."""
    global _default_executor, _shutdown_flag
    _default_executor = SingleThreadedExecutor()
    _shutdown_flag = False


def shutdown() -> None:
    """Shutdown the pub/sub system."""
    global _shutdown_flag
    _shutdown_flag = True
    if _default_executor:
        _default_executor.shutdown()
    TopicManager.reset()


def ok() -> bool:
    """Check if the system is still running."""
    return not _shutdown_flag


def spin(node: Node) -> None:
    """Spin a node until shutdown.
    
    Args:
        node: Node to spin
    """
    global _default_executor
    if _default_executor is None:
        init()
    
    _default_executor.add_node(node)
    try:
        while ok():
            _default_executor.spin_once()
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        _default_executor.remove_node(node)


def spin_once(node: Node, timeout_sec: float = 0.0) -> int:
    """Spin a node once.
    
    Args:
        node: Node to spin
        timeout_sec: Max time to wait for messages
        
    Returns:
        Number of callbacks executed
    """
    return node.spin_once(timeout_sec)


def spin_until_future_complete(node: Node, future: Any) -> None:
    """Spin until a future completes (placeholder for async support)."""
    while ok() and not getattr(future, 'done', lambda: True)():
        spin_once(node)
        time.sleep(0.001)


def get_topic_names_and_types() -> List[Tuple[str, str]]:
    """Get all topic names and their types."""
    manager = TopicManager()
    result = []
    for name in manager.get_topic_names():
        info = manager.get_topic_info(name)
        if info:
            result.append((name, info['type']))
    return result


def get_node_names() -> List[str]:
    """Get all node names."""
    return TopicManager().get_node_names()


# =============================================================================
# Rate Limiter (ROS2-like)
# =============================================================================

class Rate:
    """Rate limiter for controlling loop frequency.
    
    Example:
        ```python
        rate = Rate(100)  # 100 Hz
        while ok():
            # Do work
            rate.sleep()
        ```
    """
    
    def __init__(self, hz: float):
        """Create a rate limiter.
        
        Args:
            hz: Target frequency in Hz
        """
        self._period = 1.0 / hz
        self._last_time = time.time()
    
    def sleep(self) -> None:
        """Sleep to maintain the target rate."""
        elapsed = time.time() - self._last_time
        sleep_time = self._period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        self._last_time = time.time()
    
    @property
    def period(self) -> float:
        """Get the period in seconds."""
        return self._period


# =============================================================================
# Convenience Aliases
# =============================================================================

# Commonly used QoS profiles
qos_profile_sensor_data = QoSProfile.sensor_data()
qos_profile_default = QoSProfile.default()
qos_profile_services = QoSProfile.services()
qos_profile_parameters = QoSProfile.parameters()


# =============================================================================
# Network Server (Reply-to-Sender Pattern)
# =============================================================================

@dataclass
class RemoteDevice:
    """Information about a discovered remote device."""
    address: str
    port: int
    last_seen: float
    recv_count: int = 0
    send_count: int = 0
    last_message: Optional[Any] = None


class NetworkServer(Generic[MsgT]):
    """Server that auto-discovers clients and replies to senders.
    
    This implements the pattern where:
    - Server doesn't need to know client IPs
    - Clients send to server's known IP
    - Server replies back to each client's address
    
    Perfect for ESP32 -> PC communication where ESP32 knows PC's IP,
    but PC auto-discovers ESP32s.
    
    Example:
        ```python
        from capybarish.pubsub import NetworkServer
        from capybarish.generated import ReceivedData, SentData
        
        def on_feedback(msg: SentData, addr: str):
            print(f"Feedback from {addr}: pos={msg.motor.pos}")
        
        server = NetworkServer(
            recv_type=SentData,
            send_type=ReceivedData,
            recv_port=6666,
            send_port=6667,
            callback=on_feedback,
        )
        
        # Send to all discovered clients
        cmd = ReceivedData(target=1.0, kp=10.0)
        server.send_to_all(cmd)
        
        # Or send to specific client
        server.send_to("192.168.1.100", cmd)
        
        # Spin in main loop
        while True:
            server.spin_once()
        ```
    """
    
    def __init__(
        self,
        recv_type: Type[MsgT],
        send_type: Type,
        recv_port: int,
        send_port: int,
        callback: Optional[Callable[[MsgT, str], None]] = None,
        timeout_sec: float = 2.0,
    ):
        """Create a network server.
        
        Args:
            recv_type: Message type to receive
            send_type: Message type to send
            recv_port: Port to listen on for incoming messages
            send_port: Port to send replies to
            callback: Callback(msg, sender_ip) when message received
            timeout_sec: Time after which a client is considered inactive
        """
        self._recv_type = recv_type
        self._send_type = send_type
        self._recv_port = recv_port
        self._send_port = send_port
        self._callback = callback
        self._timeout_sec = timeout_sec
        
        # Socket for receiving and sending
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", recv_port))
        self._socket.setblocking(False)
        
        # Discovered devices
        self._devices: Dict[str, RemoteDevice] = {}
        self._devices_lock = threading.Lock()
        
        # Statistics
        self._total_recv = 0
        self._total_send = 0
    
    @property
    def devices(self) -> Dict[str, RemoteDevice]:
        """Get all discovered devices."""
        with self._devices_lock:
            return dict(self._devices)
    
    @property
    def active_devices(self) -> Dict[str, RemoteDevice]:
        """Get only active devices (seen within timeout)."""
        now = time.time()
        with self._devices_lock:
            return {
                addr: dev for addr, dev in self._devices.items()
                if now - dev.last_seen < self._timeout_sec
            }
    
    def spin_once(self) -> int:
        """Process all pending incoming messages.
        
        Returns:
            Number of messages processed
        """
        count = 0
        while True:
            try:
                data, addr = self._socket.recvfrom(4096)
                sender_ip = addr[0]
                
                # Check message size
                if hasattr(self._recv_type, '_SIZE'):
                    if len(data) < self._recv_type._SIZE:
                        continue
                
                # Deserialize
                if hasattr(self._recv_type, 'deserialize'):
                    msg = self._recv_type.deserialize(data)
                else:
                    continue
                
                # Update device info
                now = time.time()
                with self._devices_lock:
                    if sender_ip not in self._devices:
                        self._devices[sender_ip] = RemoteDevice(
                            address=sender_ip,
                            port=addr[1],
                            last_seen=now,
                        )
                    dev = self._devices[sender_ip]
                    dev.last_seen = now
                    dev.recv_count += 1
                    dev.last_message = msg
                
                self._total_recv += 1
                count += 1
                
                # Call user callback
                if self._callback:
                    self._callback(msg, sender_ip)
                    
            except BlockingIOError:
                break  # No more data
            except Exception:
                continue
        
        return count
    
    def send_to(self, address: str, msg) -> bool:
        """Send a message to a specific device.
        
        Args:
            address: IP address of the device
            msg: Message to send (must have serialize() method)
            
        Returns:
            True if sent successfully
        """
        try:
            if hasattr(msg, 'serialize'):
                data = msg.serialize()
                self._socket.sendto(data, (address, self._send_port))
                
                with self._devices_lock:
                    if address in self._devices:
                        self._devices[address].send_count += 1
                
                self._total_send += 1
                return True
        except Exception:
            pass
        return False
    
    def send_to_all(self, msg, active_only: bool = True) -> int:
        """Send a message to all discovered devices.
        
        Args:
            msg: Message to send
            active_only: Only send to active devices (default True)
            
        Returns:
            Number of devices sent to
        """
        devices = self.active_devices if active_only else self.devices
        count = 0
        for addr in devices:
            if self.send_to(addr, msg):
                count += 1
        return count
    
    def close(self) -> None:
        """Close the server socket."""
        self._socket.close()
    
    def __enter__(self):
        return self
    
    def __exit__(self, *args):
        self.close()

