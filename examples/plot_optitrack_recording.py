#!/usr/bin/env python3
"""
Plot OptiTrack Recording Data.

This script loads and visualizes data recorded by optitrack_robot_client.py,
including position trajectories, distance to goal, and speed information.

Usage:
    python plot_optitrack_recording.py recording.npz
    python plot_optitrack_recording.py recording.npz --output trajectory.png
    python plot_optitrack_recording.py recording.npz --speed-window 5

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>
"""

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def load_recording(filepath: str) -> dict:
    """Load recording data from .npz file."""
    data = np.load(filepath, allow_pickle=True)
    return {key: data[key] for key in data.files}


def compute_speed(
    timestamps: np.ndarray,
    positions_x: np.ndarray,
    positions_y: np.ndarray,
    positions_z: np.ndarray,
    window: int = 5,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute speed from position data using finite differences.
    
    Args:
        timestamps: Time stamps in seconds
        positions_x, positions_y, positions_z: Position coordinates
        window: Number of samples for smoothing (moving average)
    
    Returns:
        speed_2d: 2D speed (XY plane)
        speed_3d: 3D speed
        velocity_x: X velocity component
        velocity_y: Y velocity component
    """
    n = len(timestamps)
    if n < 2:
        return np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    
    # Compute time differences
    dt = np.diff(timestamps)
    dt = np.where(dt > 0, dt, 1e-6)  # Avoid division by zero
    
    # Compute position differences
    dx = np.diff(positions_x)
    dy = np.diff(positions_y)
    dz = np.diff(positions_z)
    
    # Compute velocities
    vx = dx / dt
    vy = dy / dt
    vz = dz / dt
    
    # Compute speeds
    speed_2d = np.sqrt(vx**2 + vy**2)
    speed_3d = np.sqrt(vx**2 + vy**2 + vz**2)
    
    # Apply moving average smoothing
    if window > 1 and len(speed_2d) >= window:
        kernel = np.ones(window) / window
        speed_2d_smooth = np.convolve(speed_2d, kernel, mode='same')
        speed_3d_smooth = np.convolve(speed_3d, kernel, mode='same')
        vx_smooth = np.convolve(vx, kernel, mode='same')
        vy_smooth = np.convolve(vy, kernel, mode='same')
    else:
        speed_2d_smooth = speed_2d
        speed_3d_smooth = speed_3d
        vx_smooth = vx
        vy_smooth = vy
    
    # Pad to match original array length (add 0 at the start)
    speed_2d_final = np.concatenate([[0], speed_2d_smooth])
    speed_3d_final = np.concatenate([[0], speed_3d_smooth])
    vx_final = np.concatenate([[0], vx_smooth])
    vy_final = np.concatenate([[0], vy_smooth])
    
    return speed_2d_final, speed_3d_final, vx_final, vy_final


def quaternion_to_yaw(rotations: np.ndarray) -> np.ndarray:
    """
    Convert quaternions to yaw angles (rotation around Z axis).
    
    Args:
        rotations: Array of quaternions [x, y, z, w]
    
    Returns:
        yaw: Array of yaw angles in radians
    """
    yaw = np.zeros(len(rotations))
    for i, quat in enumerate(rotations):
        x, y, z, w = quat
        # Yaw (rotation around Z axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw[i] = np.arctan2(siny_cosp, cosy_cosp)
    return yaw


def plot_recording(
    data: dict,
    output_file: Optional[str] = None,
    speed_window: int = 5,
    show_plot: bool = True,
    figsize: Tuple[int, int] = (16, 12),
) -> None:
    """
    Create comprehensive visualization of recording data.
    
    Args:
        data: Dictionary of recorded data
        output_file: Optional path to save the figure
        speed_window: Window size for speed smoothing
        show_plot: Whether to display the plot
        figsize: Figure size
    """
    timestamps = data['timestamps']
    positions_x = data['positions_x']
    positions_y = data['positions_y']
    positions_z = data['positions_z']
    goal_x = data['goal_x']
    goal_y = data['goal_y']
    distances = data['distances']
    rotations = data['rotations']
    
    # Metadata
    rigid_body_id = data.get('rigid_body_id', 'Unknown')
    record_interval = data.get('record_interval', 0.1)
    total_runtime = data.get('total_runtime', timestamps[-1] if len(timestamps) > 0 else 0)
    
    # Compute speed
    speed_2d, speed_3d, vx, vy = compute_speed(
        timestamps, positions_x, positions_y, positions_z, window=speed_window
    )
    
    # Compute yaw from quaternion
    yaw = quaternion_to_yaw(rotations)
    
    # Create figure with GridSpec
    fig = plt.figure(figsize=figsize)
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # Title
    fig.suptitle(
        f"OptiTrack Recording Analysis\n"
        f"RB ID: {rigid_body_id} | Duration: {total_runtime:.1f}s | "
        f"Samples: {len(timestamps)} | Interval: {record_interval}s",
        fontsize=14, fontweight='bold'
    )
    
    # ==========================================================================
    # Plot 1: 2D Trajectory (XY plane)
    # ==========================================================================
    ax1 = fig.add_subplot(gs[0, 0])
    
    # Color trajectory by time
    scatter = ax1.scatter(positions_x, positions_y, c=timestamps, cmap='viridis', 
                          s=10, alpha=0.7, label='Trajectory')
    
    # Start and end markers
    ax1.plot(positions_x[0], positions_y[0], 'go', markersize=12, label='Start', zorder=5)
    ax1.plot(positions_x[-1], positions_y[-1], 'rs', markersize=12, label='End', zorder=5)
    
    # Goal position (use the last goal, assuming it might change)
    ax1.plot(goal_x[-1], goal_y[-1], 'k*', markersize=15, label='Goal', zorder=5)
    
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('2D Trajectory (XY Plane)')
    ax1.legend(loc='best', fontsize=8)
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Time (s)')
    
    # ==========================================================================
    # Plot 2: Position vs Time
    # ==========================================================================
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(timestamps, positions_x, 'r-', label='X', alpha=0.8)
    ax2.plot(timestamps, positions_y, 'g-', label='Y', alpha=0.8)
    ax2.plot(timestamps, positions_z, 'b-', label='Z', alpha=0.8)
    
    # Plot goal positions
    ax2.axhline(y=goal_x[-1], color='r', linestyle='--', alpha=0.5, label=f'Goal X={goal_x[-1]:.2f}')
    ax2.axhline(y=goal_y[-1], color='g', linestyle='--', alpha=0.5, label=f'Goal Y={goal_y[-1]:.2f}')
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time')
    ax2.legend(loc='best', fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # ==========================================================================
    # Plot 3: Distance to Goal vs Time
    # ==========================================================================
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(timestamps, distances, 'b-', linewidth=2)
    ax3.fill_between(timestamps, 0, distances, alpha=0.3)
    
    ax3.axhline(y=0.1, color='g', linestyle='--', alpha=0.5, label='Close (0.1m)')
    ax3.axhline(y=0.5, color='orange', linestyle='--', alpha=0.5, label='Near (0.5m)')
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Distance (m)')
    ax3.set_title('Distance to Goal vs Time')
    ax3.legend(loc='best', fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(bottom=0)
    
    # ==========================================================================
    # Plot 4: 2D Speed vs Time
    # ==========================================================================
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(timestamps, speed_2d, 'purple', linewidth=1.5, label=f'2D Speed (window={speed_window})')
    ax4.fill_between(timestamps, 0, speed_2d, alpha=0.3, color='purple')
    
    # Statistics
    mean_speed = np.mean(speed_2d)
    max_speed = np.max(speed_2d)
    ax4.axhline(y=mean_speed, color='red', linestyle='--', alpha=0.7, 
                label=f'Mean: {mean_speed:.3f} m/s')
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Speed (m/s)')
    ax4.set_title(f'2D Speed (XY Plane) - Max: {max_speed:.3f} m/s')
    ax4.legend(loc='best', fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.set_ylim(bottom=0)
    
    # ==========================================================================
    # Plot 5: 3D Speed vs Time
    # ==========================================================================
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(timestamps, speed_3d, 'teal', linewidth=1.5, label=f'3D Speed (window={speed_window})')
    ax5.fill_between(timestamps, 0, speed_3d, alpha=0.3, color='teal')
    
    mean_speed_3d = np.mean(speed_3d)
    max_speed_3d = np.max(speed_3d)
    ax5.axhline(y=mean_speed_3d, color='red', linestyle='--', alpha=0.7,
                label=f'Mean: {mean_speed_3d:.3f} m/s')
    
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Speed (m/s)')
    ax5.set_title(f'3D Speed - Max: {max_speed_3d:.3f} m/s')
    ax5.legend(loc='best', fontsize=8)
    ax5.grid(True, alpha=0.3)
    ax5.set_ylim(bottom=0)
    
    # ==========================================================================
    # Plot 6: Velocity Components
    # ==========================================================================
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.plot(timestamps, vx, 'r-', label='Vx', alpha=0.8)
    ax6.plot(timestamps, vy, 'g-', label='Vy', alpha=0.8)
    ax6.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Velocity (m/s)')
    ax6.set_title('Velocity Components (XY)')
    ax6.legend(loc='best', fontsize=8)
    ax6.grid(True, alpha=0.3)
    
    # ==========================================================================
    # Plot 7: Heading/Yaw vs Time
    # ==========================================================================
    ax7 = fig.add_subplot(gs[2, 0])
    ax7.plot(timestamps, np.degrees(yaw), 'orange', linewidth=1.5)
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Yaw (degrees)')
    ax7.set_title('Heading (Yaw) vs Time')
    ax7.grid(True, alpha=0.3)
    
    # ==========================================================================
    # Plot 8: Trajectory colored by speed
    # ==========================================================================
    ax8 = fig.add_subplot(gs[2, 1])
    scatter2 = ax8.scatter(positions_x, positions_y, c=speed_2d, cmap='hot', 
                           s=15, alpha=0.8)
    ax8.plot(positions_x[0], positions_y[0], 'go', markersize=10, label='Start')
    ax8.plot(positions_x[-1], positions_y[-1], 'bs', markersize=10, label='End')
    ax8.plot(goal_x[-1], goal_y[-1], 'k*', markersize=12, label='Goal')
    
    ax8.set_xlabel('X Position (m)')
    ax8.set_ylabel('Y Position (m)')
    ax8.set_title('Trajectory Colored by Speed')
    ax8.legend(loc='best', fontsize=8)
    ax8.axis('equal')
    ax8.grid(True, alpha=0.3)
    
    cbar2 = plt.colorbar(scatter2, ax=ax8)
    cbar2.set_label('Speed (m/s)')
    
    # ==========================================================================
    # Plot 9: Speed Histogram
    # ==========================================================================
    ax9 = fig.add_subplot(gs[2, 2])
    ax9.hist(speed_2d, bins=30, color='purple', alpha=0.7, edgecolor='black')
    ax9.axvline(x=mean_speed, color='red', linestyle='--', linewidth=2,
                label=f'Mean: {mean_speed:.3f} m/s')
    ax9.axvline(x=np.median(speed_2d), color='green', linestyle='--', linewidth=2,
                label=f'Median: {np.median(speed_2d):.3f} m/s')
    
    ax9.set_xlabel('Speed (m/s)')
    ax9.set_ylabel('Count')
    ax9.set_title('Speed Distribution')
    ax9.legend(loc='best', fontsize=8)
    ax9.grid(True, alpha=0.3)
    
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    
    # Save figure
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {output_file}")
    
    # Show plot
    if show_plot:
        plt.show()
    else:
        plt.close()


def print_statistics(data: dict, speed_window: int = 5) -> None:
    """Print summary statistics of the recording."""
    timestamps = data['timestamps']
    positions_x = data['positions_x']
    positions_y = data['positions_y']
    positions_z = data['positions_z']
    distances = data['distances']
    
    speed_2d, speed_3d, _, _ = compute_speed(
        timestamps, positions_x, positions_y, positions_z, window=speed_window
    )
    
    print("\n" + "=" * 60)
    print("RECORDING STATISTICS")
    print("=" * 60)
    
    print(f"\n📊 General:")
    print(f"  Total samples:     {len(timestamps)}")
    print(f"  Duration:          {timestamps[-1] - timestamps[0]:.2f} s")
    print(f"  Record interval:   {data.get('record_interval', 'N/A')} s")
    print(f"  Rigid body ID:     {data.get('rigid_body_id', 'N/A')}")
    
    print(f"\n📍 Position Range:")
    print(f"  X: [{positions_x.min():.3f}, {positions_x.max():.3f}] m")
    print(f"  Y: [{positions_y.min():.3f}, {positions_y.max():.3f}] m")
    print(f"  Z: [{positions_z.min():.3f}, {positions_z.max():.3f}] m")
    
    print(f"\n🎯 Distance to Goal:")
    print(f"  Start:   {distances[0]:.3f} m")
    print(f"  End:     {distances[-1]:.3f} m")
    print(f"  Min:     {distances.min():.3f} m")
    print(f"  Max:     {distances.max():.3f} m")
    print(f"  Mean:    {distances.mean():.3f} m")
    
    print(f"\n🚀 Speed (2D, XY Plane):")
    print(f"  Mean:    {speed_2d.mean():.4f} m/s")
    print(f"  Max:     {speed_2d.max():.4f} m/s")
    print(f"  Min:     {speed_2d.min():.4f} m/s")
    print(f"  Std:     {speed_2d.std():.4f} m/s")
    
    print(f"\n🚀 Speed (3D):")
    print(f"  Mean:    {speed_3d.mean():.4f} m/s")
    print(f"  Max:     {speed_3d.max():.4f} m/s")
    
    # Total distance traveled
    dx = np.diff(positions_x)
    dy = np.diff(positions_y)
    total_dist_2d = np.sum(np.sqrt(dx**2 + dy**2))
    print(f"\n📏 Total Distance Traveled (2D): {total_dist_2d:.3f} m")
    
    print("=" * 60)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot OptiTrack recording data",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    
    parser.add_argument("recording", type=str, help="Path to recording .npz file")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Output file path for saving the figure")
    parser.add_argument("--speed-window", "-w", type=int, default=5,
                        help="Window size for speed smoothing (moving average)")
    parser.add_argument("--no-show", action="store_true",
                        help="Don't display the plot (only save)")
    parser.add_argument("--stats-only", action="store_true",
                        help="Only print statistics, don't plot")
    parser.add_argument("--figsize", type=int, nargs=2, default=[16, 12],
                        help="Figure size (width height)")
    
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    
    # Check if file exists
    if not Path(args.recording).exists():
        print(f"Error: Recording file not found: {args.recording}")
        sys.exit(1)
    
    # Load data
    print(f"Loading recording: {args.recording}")
    try:
        data = load_recording(args.recording)
    except Exception as e:
        print(f"Error loading recording: {e}")
        sys.exit(1)
    
    # Print statistics
    print_statistics(data, speed_window=args.speed_window)
    
    # Plot if not stats-only
    if not args.stats_only:
        plot_recording(
            data,
            output_file=args.output,
            speed_window=args.speed_window,
            show_plot=not args.no_show,
            figsize=tuple(args.figsize),
        )


if __name__ == "__main__":
    main()
