"""
Capybarish: Lightweight communication middleware for cloud-based robotics controllers.

Capybarish is an MVP designed to support research on reconfigurable legged metamachines
- modular robots that can dynamically reconfigure their morphology using autonomous
modular legs. This middleware enables real-time communication between cloud-based
controllers and distributed modular robot components.

Research Context:
This package supports the research presented in "Reconfigurable legged metamachines
that run on autonomous modular legs" (https://arxiv.org/abs/2505.00784) by Chen Yu et al.

Key features:
- Real-time UDP communication for distributed robot systems
- Modular plugin architecture for autonomous robot components
- Dashboard visualization for real-time monitoring

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

__version__ = "0.1.0"
__author__ = "Chen Yu"
__email__ = "chenyu@u.northwestern.edu"
__license__ = "Apache-2.0"

from .dashboard_server import DashboardServer
from .data_struct import RobotData, RobotDataLite, SentDataStruct

# Import main classes for convenience
from .interface import Interface
from .utils import load_cfg

# Code generation module
from . import codegen

# Pub/Sub system (ROS2-like API)
from .pubsub import (
    # Core classes
    Node,
    Publisher,
    Subscription,
    Timer,
    Topic,
    TopicManager,
    # QoS
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    # Executors
    SingleThreadedExecutor,
    MultiThreadedExecutor,
    # Rate control
    Rate,
    # Global functions (ROS2-like)
    init,
    shutdown,
    ok,
    spin,
    spin_once,
    spin_until_future_complete,
    get_topic_names_and_types,
    get_node_names,
    # Pre-defined QoS profiles
    qos_profile_sensor_data,
    qos_profile_default,
    qos_profile_services,
    qos_profile_parameters,
    # Logging
    NodeLogger,
    LogLevel,
)

__all__ = [
    # Legacy API
    "Interface",
    "DashboardServer",
    "load_cfg",
    "SentDataStruct",
    "RobotData",
    "RobotDataLite",
    "codegen",
    # Pub/Sub API (ROS2-like)
    "Node",
    "Publisher",
    "Subscription",
    "Timer",
    "Topic",
    "TopicManager",
    "QoSProfile",
    "QoSReliabilityPolicy",
    "QoSHistoryPolicy",
    "QoSDurabilityPolicy",
    "SingleThreadedExecutor",
    "MultiThreadedExecutor",
    "Rate",
    "init",
    "shutdown",
    "ok",
    "spin",
    "spin_once",
    "spin_until_future_complete",
    "get_topic_names_and_types",
    "get_node_names",
    "qos_profile_sensor_data",
    "qos_profile_default",
    "qos_profile_services",
    "qos_profile_parameters",
    "NodeLogger",
    "LogLevel",
    "__version__",
]
