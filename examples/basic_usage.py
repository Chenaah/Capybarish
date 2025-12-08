#!/usr/bin/env python3
"""
Basic usage example for the Capybarish motion capture system.

This example demonstrates how to:
- Initialize and configure the Capybarish interface
- Connect to robot modules
- Send sinusoidal control commands to motors
- Handle real-time keyboard input for motor control
- Receive and process sensor data

The example runs a continuous control loop that sends sinusoidal position
commands to all configured robot modules while allowing real-time control
via keyboard input.

Keyboard Controls:
    'e': Enable motors (switch_on = 1)
    'd': Disable motors (switch_on = 0)
    Ctrl+C: Exit the program

Usage:
    python basic_usage.py                    # Use default configuration
    python basic_usage.py --cfg my_config   # Use custom configuration
    python basic_usage.py --help            # Show help message

Requirements:
    - Properly configured robot modules
    - Valid configuration file in config/ directory
    - Network connectivity to robot modules

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

import argparse
import signal
import sys
import time
from collections import deque
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from omegaconf import DictConfig

from capybarish.interface import Interface
from capybarish.kbhit import KBHit
from capybarish.utils import load_cfg

# Constants
DEFAULT_CONFIG_NAME = "default"
INITIAL_DATA_COLLECTION_CYCLES = 10
CONTROL_LOOP_SLEEP_TIME = 0.02  # 50 Hz control loop
SINUSOIDAL_AMPLITUDE = 0.6
SINUSOIDAL_FREQUENCY_DIVIDER = 3
DEFAULT_KP_GAIN = 8.0
DEFAULT_KD_GAIN = 0.2
PLOT_HISTORY_LENGTH = 200  # Number of data points to display in plots

# Keyboard command mappings
KEY_ENABLE_MOTORS = "e"
KEY_DISABLE_MOTORS = "d"

# Global variables for cleanup
interface: Optional[Interface] = None
kb: Optional[KBHit] = None

# Global variables for plotting
fig = None
ax_action = None
ax_dof = None
action_history = None
dof_history = None
time_history = None


def signal_handler(signum: int, frame) -> None:
    """Handle interrupt signals for graceful shutdown.

    Args:
        signum: Signal number
        frame: Current stack frame
    """
    print(f"\nReceived signal {signum}. Shutting down gracefully...")
    cleanup_and_exit()


def cleanup_and_exit() -> None:
    """Perform cleanup operations and exit the program."""
    global interface, kb, fig

    try:
        # Disable motors first
        if interface is not None:
            print("Disabling motors...")
            interface.switch_on = 0
            # Send a final command to ensure motors are disabled
            try:
                if hasattr(interface, "send_action"):
                    zero_action = np.zeros(len(interface.cfg.interface.module_ids))
                    interface.send_action(zero_action)
                print("Motors disabled.")
            except Exception as e:
                print(f"Warning: Could not send final motor command: {e}")

        # Close plots before other cleanup to avoid matplotlib blocking
        if fig is not None:
            print("Closing plots...")
            try:
                plt.close('all')  # Close all figures
                plt.ioff()  # Turn off interactive mode
            except Exception as e:
                print(f"Warning: Error closing plots: {e}")

        # Cleanup keyboard handler
        if kb is not None:
            print("Cleaning up keyboard handler...")
            try:
                if hasattr(kb, "set_normal_term"):
                    kb.set_normal_term()
            except Exception as e:
                print(f"Warning: Error cleaning keyboard handler: {e}")

        # Explicitly cleanup interface display (Rich Live)
        if interface is not None:
            print("Cleaning up interface display...")
            try:
                if hasattr(interface, "_cleanup_display"):
                    interface._cleanup_display()
            except Exception as e:
                print(f"Warning: Error cleaning interface display: {e}")

    except Exception as e:
        print(f"Error during cleanup: {e}")
    finally:
        print("Cleanup complete. Exiting...")
        # Restore cursor explicitly as final safety measure
        try:
            sys.stdout.write('\033[?25h')
            sys.stdout.flush()
        except:
            pass
        # Force exit without calling sys.exit() to avoid deadlock
        import os
        os._exit(0)


def initialize_system(cfg: DictConfig) -> Interface:
    """Initialize the Capybarish interface system.

    Args:
        cfg: Configuration object containing system parameters

    Returns:
        Initialized Interface instance

    Raises:
        RuntimeError: If initialization fails
    """
    try:
        print("Initializing Capybarish interface...")
        interface = Interface(cfg)

        print(f"Collecting initial data for {INITIAL_DATA_COLLECTION_CYCLES} cycles...")
        for i in range(INITIAL_DATA_COLLECTION_CYCLES):
            interface.receive_module_data()
            print(f"  Initial data collection: {i+1}/{INITIAL_DATA_COLLECTION_CYCLES}")

        print("System initialization complete!")
        return interface

    except Exception as e:
        raise RuntimeError(f"Failed to initialize system: {e}") from e


def initialize_plots(num_modules: int) -> None:
    """Initialize real-time plotting for action and position data.

    Args:
        num_modules: Number of robot modules to plot
    """
    global fig, ax_action, ax_dof, action_history, dof_history, time_history

    # Enable interactive mode
    plt.ion()

    # Create figure with single subplot
    fig, ax_action = plt.subplots(1, 1, figsize=(12, 6))
    ax_dof = ax_action  # Use the same axis for both
    fig.suptitle('Real-time Motor Control Data', fontsize=14, fontweight='bold')

    # Initialize data buffers (deques for efficient append/pop operations)
    action_history = [deque(maxlen=PLOT_HISTORY_LENGTH) for _ in range(num_modules)]
    dof_history = [deque(maxlen=PLOT_HISTORY_LENGTH) for _ in range(num_modules)]
    time_history = deque(maxlen=PLOT_HISTORY_LENGTH)

    # Configure subplot
    ax_action.set_xlabel('Time Step')
    ax_action.set_ylabel('Value')
    ax_action.set_title('Action Commands & Joint Positions')
    ax_action.grid(True, alpha=0.3)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    plt.show(block=False)

    print("Real-time plotting initialized!")


def update_plots(time_step: int, action_array: np.ndarray, dof_pos: np.ndarray) -> None:
    """Update real-time plots with new data.

    Args:
        time_step: Current time step
        action_array: Array of action commands
        dof_pos: Array of DOF positions
    """
    global fig, ax_action, ax_dof, action_history, dof_history, time_history

    if fig is None or action_history is None:
        return

    # Add new data to history buffers
    time_history.append(time_step)
    for i in range(len(action_array)):
        action_history[i].append(action_array[i])
        dof_history[i].append(dof_pos[i])

    # Clear previous plot
    ax_action.clear()

    # Reapply subplot settings
    ax_action.set_xlabel('Time Step')
    ax_action.set_ylabel('Value')
    ax_action.set_title('Action Commands & Joint Positions')
    ax_action.grid(True, alpha=0.3)

    # Plot data for each module
    time_array = list(time_history)
    for i in range(len(action_array)):
        action_data = list(action_history[i])
        dof_data = list(dof_history[i])
        
        # Plot action with solid line
        ax_action.plot(time_array, action_data, 
                      label=f'Action {i}', 
                      linewidth=2, 
                      linestyle='-')
        # Plot DOF position with dashed line
        ax_action.plot(time_array, dof_data, 
                      label=f'Position {i}', 
                      linewidth=1.5, 
                      linestyle='--',
                      alpha=0.8)

    # Add legend
    ax_action.legend(loc='upper right', fontsize=8, ncol=2)

    # Refresh the plot
    plt.tight_layout()
    plt.pause(0.001)  # Small pause to update the plot


def run_control_loop(cfg: DictConfig) -> None:
    """Run the main control loop with sinusoidal commands and keyboard input.

    This function implements the main control loop that:
    1. Receives sensor data from robot modules
    2. Generates sinusoidal position commands
    3. Sends commands to all configured modules
    4. Handles keyboard input for motor control

    Args:
        cfg: Configuration object containing system parameters

    Raises:
        KeyboardInterrupt: When user requests shutdown
        RuntimeError: If control loop encounters critical errors
    """
    global interface, kb

    try:
        # Initialize system components
        interface = initialize_system(cfg)
        kb = KBHit()

        # Initialize real-time plotting
        num_modules = len(cfg.interface.module_ids)
        initialize_plots(num_modules)

        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        print("\\nStarting control loop...")
        print("Controls: 'e' = enable motors, 'd' = disable motors, Ctrl+C = exit")
        print("=" * 60)

        time_step = 0
        num_modules = len(cfg.interface.module_ids)

        while True:
            try:
                # Record start time of iteration
                iteration_start_time = time.perf_counter()
                
                # Receive sensor data from all modules
                interface.receive_module_data()

                # Get observable data (for logging/monitoring)
                observable_data = interface.get_observable_data()

                # print(observable_data)
                # import pdb; pdb.set_trace()
                # observable_data.keys()
                dof_pos = observable_data["dof_pos"]
                print(f"Current DOF Positions: {dof_pos}")

                # Generate sinusoidal control command
                sinusoidal_command = SINUSOIDAL_AMPLITUDE * np.sin(
                    time_step / SINUSOIDAL_FREQUENCY_DIVIDER
                )
                # sinusoidal_command *= 0 
                action_array = np.ones(num_modules) * sinusoidal_command

                # Update real-time plots
                update_plots(time_step, action_array, dof_pos)

                # Send control commands to all modules
                interface.send_action(
                    action_array, kps=np.array([DEFAULT_KP_GAIN]*num_modules), kds=np.array([DEFAULT_KD_GAIN]*num_modules)
                )

                # Handle keyboard input
                if kb.kbhit():
                    input_key = kb.getch()
                    handle_keyboard_input(input_key, interface)

                # Control loop timing - sleep for remaining time to maintain target loop rate
                iteration_elapsed_time = time.perf_counter() - iteration_start_time
                sleep_time = max(0, CONTROL_LOOP_SLEEP_TIME - iteration_elapsed_time)
                time.sleep(sleep_time)
                
                # Calculate actual loop time and frequency
                actual_loop_time = time.perf_counter() - iteration_start_time
                actual_frequency = 1.0 / actual_loop_time if actual_loop_time > 0 else 0.0
                
                time_step += 1

                # Optional: Print status every N iterations
                if time_step % 100 == 0:
                    status = "ENABLED" if interface.switch_on else "DISABLED"
                    target_frequency = 1.0 / CONTROL_LOOP_SLEEP_TIME
                    print(f"Step {time_step}: Motors {status}, Command: {sinusoidal_command:.3f}, "
                          f"Freq: {actual_frequency:.1f} Hz (target: {target_frequency:.1f} Hz)")

            except KeyboardInterrupt:
                print("\\nKeyboard interrupt received...")
                break
            except Exception as e:
                print(f"Error in control loop: {e}")
                # Continue loop for non-critical errors
                continue

    except Exception as e:
        print(f"Critical error in control loop: {e}")
        raise RuntimeError(f"Control loop failed: {e}") from e
    finally:
        cleanup_and_exit()


def handle_keyboard_input(key: str, interface: Interface) -> None:
    """Handle keyboard input commands.

    Args:
        key: Pressed key character
        interface: Interface instance to control
    """
    if key == KEY_ENABLE_MOTORS:
        interface.switch_on = 1
        print("Motors ENABLED")
    elif key == KEY_DISABLE_MOTORS:
        interface.switch_on = 0
        print("Motors DISABLED")
    else:
        # Ignore unknown keys silently
        pass


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments.

    Returns:
        Parsed arguments namespace
    """
    parser = argparse.ArgumentParser(
        description="Basic usage example for Capybarish motion capture system",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "--cfg",
        type=str,
        default=DEFAULT_CONFIG_NAME,
        help="Configuration file name (without .yaml extension)",
    )

    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose output")

    return parser.parse_args()


def main() -> None:
    """Main entry point for the basic usage example."""
    try:
        # Parse command line arguments
        args = parse_arguments()

        if args.verbose:
            print(f"Loading configuration: {args.cfg}")

        # Load configuration
        try:
            config = load_cfg(args.cfg)
        except FileNotFoundError as e:
            print(f"Error: Configuration file not found: {e}")
            print("Make sure the configuration file exists in the config/ directory")
            sys.exit(1)
        except Exception as e:
            print(f"Error loading configuration: {e}")
            sys.exit(1)

        if args.verbose:
            print(f"Configuration loaded successfully")
            print(f"Number of modules: {len(config.interface.module_ids)}")

        # Run the main control loop
        run_control_loop(config)

    except KeyboardInterrupt:
        print("\\nProgram interrupted by user")
        cleanup_and_exit()
    except Exception as e:
        print(f"Unexpected error: {e}")
        cleanup_and_exit()


if __name__ == "__main__":
    main()
