#!/usr/bin/env python3
import sys, math, re
from pathlib import Path

import rosbag
import pandas as pd
import numpy as np
import tf
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# Constants
SIZE_FACTOR = 7.33
X_MAP_SHIFT = 48
Y_MAP_SHIFT = 50

COLORS = {
    'real':   '#2B83BA',
    'mapped': '#FDAE61',
    'sim':    '#ABDDA4'
}

SIM_TOPIC   = '/sim/euler'
REAL_TOPIC  = '/donkey/pose'

def rotate(points: np.ndarray, degrees: float = 0) -> np.ndarray:
    theta = np.deg2rad(degrees)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    return points.dot(R.T)

def extract_messages(bag: rosbag.Bag, topic: str):
    return [(t.to_sec(), msg) for _, msg, t in bag.read_messages(topics=[topic])]

from scipy.signal import savgol_filter
import numpy as np

def compute_speed(positions: np.ndarray, times: np.ndarray, window: int = 30, poly: int = 3):
    """
    Compute smoothed speed from position and time arrays using double Savitzky-Golay filtering.

    Parameters:
    - positions: Nx2 array of x, y positions
    - times: 1D array of timestamps
    - window: Savitzky-Golay window size (must be odd and >= poly+2)
    - poly: polynomial order

    Returns:
    - speed_time: timestamps for speed values (1-index shift)
    - smoothed_speed: 1D array of smoothed speed in m/s
    """
    if len(times) < window or len(positions) < window:
        return np.array([]), np.array([])

    # Step 1: Smooth positions
    x = savgol_filter(positions[:, 0], window, poly)
    y = savgol_filter(positions[:, 1], window, poly)

    # Step 2: Compute velocities
    dt = np.diff(times)
    vx = np.diff(x) / dt
    vy = np.diff(y) / dt

    # Step 3: Compute raw speed
    speed = np.sqrt(vx**2 + vy**2)

    # Step 4: Final smoothing of speed
    smoothed_speed = savgol_filter(speed, window, poly)

    return times[1:], smoothed_speed  # Speed aligns with 1st diff

from scipy.signal import savgol_filter
import numpy as np

def compute_smoothed_acceleration(positions: np.ndarray, times: np.ndarray, window: int = 30, poly: int = 3):
    """
    Compute smoothed acceleration from position and time arrays using two-stage smoothing.

    Parameters:
    - positions: Nx2 array of x, y positions
    - times: 1D array of timestamps
    - window: window size for Savitzky-Golay filter (must be odd)
    - poly: polynomial order for Savitzky-Golay filter

    Returns:
    - accel_time: timestamps for acceleration points
    - smoothed_accel: acceleration in m/s² (scaled by 1/7.33)
    """
    if len(times) < window or len(positions) < window:
        return np.array([]), np.array([])

    # First smooth the positions
    x = savgol_filter(positions[:, 0], window, poly)
    y = savgol_filter(positions[:, 1], window, poly)

    dt = np.diff(times)
    vx = np.diff(x) / dt
    vy = np.diff(y) / dt
    speed = np.sqrt(vx**2 + vy**2)

    # Smooth speed before differentiation again
    smoothed_speed = savgol_filter(speed, window, poly)

    dv = np.diff(smoothed_speed)
    dt2 = dt[1:]  # Adjust for second diff
    acceleration = dv / dt2

    # Final smoothing
    smoothed_accel = savgol_filter(acceleration, window, poly)

    # Scale to match unit convention
    smoothed_accel /= 7.33

    return times[2:], smoothed_accel

def create_combined_plot_layout(figsize=(12, 3.2)):
    """
    Returns: fig, legend_ax, trajectory_ax, acceleration_ax
    """
    fig = plt.figure(figsize=figsize)
    from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec

    # Outer layout: 2 rows (legend/title + content), 2 cols (trajectory + acceleration)
    outer_gs = GridSpec(2, 2, figure=fig, width_ratios=[9, 3], height_ratios=[0.00001, 200])

    legend_ax = fig.add_subplot(outer_gs[0, 0])  # Top-left: title + legend
    traj_ax   = fig.add_subplot(outer_gs[1, 0])  # Bottom-left: trajectory

    # Right side with top/bottom spacers for visual alignment
    right_gs = GridSpecFromSubplotSpec(3, 1, subplot_spec=outer_gs[:, 1], height_ratios=[1, 100, 1])
    fig.add_subplot(right_gs[0]).axis("off")  # Top spacer
    fig.add_subplot(right_gs[2]).axis("off")  # Bottom spacer
    accel_ax = fig.add_subplot(right_gs[1])   # Middle: acceleration plot

    return fig, legend_ax, traj_ax, accel_ax

def process_pair(name: str, sim_path: Path, real_path: Path, out_dir: Path):
    sim_bag  = rosbag.Bag(str(sim_path))
    real_bag = rosbag.Bag(str(real_path))

    def get_waypoint_time(bag):
        for _, _, t in bag.read_messages(topics=["/waypoints"]):
            return t.to_sec()
        return None

    t0_real = get_waypoint_time(real_bag) or extract_messages(real_bag, REAL_TOPIC)[0][0]
    t0_sim  = get_waypoint_time(sim_bag)  or extract_messages(sim_bag, SIM_TOPIC)[0][0]

    def to_xy(data, is_sim, t0):
        result = []
        for t, msg in data:
            if t < t0: continue
            if is_sim:
                x = (msg.x - X_MAP_SHIFT) / SIZE_FACTOR
                y = (msg.z - Y_MAP_SHIFT) / SIZE_FACTOR
            else:
                x = msg.pose.position.x
                y = msg.pose.position.y
            result.append((t - t0, x, y))
        return result

    sim_data    = extract_messages(sim_bag, SIM_TOPIC)
    mapped_data = extract_messages(real_bag, SIM_TOPIC)
    real_data   = extract_messages(real_bag, REAL_TOPIC)

    sim_pts    = to_xy(sim_data,    True,  t0_sim)
    mapped_pts = to_xy(mapped_data, True,  t0_real)
    real_pts   = to_xy(real_data,   False, t0_real)

    def to_arr(pts):
        if not pts:
            return np.zeros((0, 2))
        return np.array([[x, y] for _, x, y in pts]) - np.array([[x, y] for _, x, y in pts])[0]

    arr_sim    = to_arr(sim_pts)
    arr_mapped = to_arr(mapped_pts)
    arr_real   = to_arr(real_pts)

    wp = np.array([[0, 2.05]])
    q = real_data[0][1].pose.orientation
    yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    deg = math.degrees(yaw) + 90
    arr_sim    = rotate(arr_sim, -deg)
    arr_mapped = rotate(arr_mapped, -deg)
    arr_real   = rotate(arr_real, -deg)
    wp = rotate(wp - arr_real[0], -deg - 90)

    times_sim    = np.array([t for t, _, _ in sim_pts])
    times_mapped = np.array([t for t, _, _ in mapped_pts])
    times_real   = np.array([t for t, _, _ in real_pts])

    ts_sim_spd,  sim_spd  = compute_speed(arr_sim,    times_sim)
    ts_map_spd,  map_spd  = compute_speed(arr_mapped, times_mapped)
    ts_real_spd, real_spd = compute_speed(arr_real,   times_real)

    ts_sim_acc,  sim_acc  = compute_smoothed_acceleration(arr_sim,    times_sim)
    ts_map_acc,  map_acc  = compute_smoothed_acceleration(arr_mapped, times_mapped)
    ts_real_acc, real_acc = compute_smoothed_acceleration(arr_real,   times_real)

    common_t = np.unique(np.concatenate([ts_sim_spd, ts_map_spd, ts_real_spd]))
    sim_spd_common  = np.interp(common_t, ts_sim_spd,  sim_spd,  left=np.nan, right=np.nan)
    map_spd_common  = np.interp(common_t, ts_map_spd,  map_spd,  left=np.nan, right=np.nan)
    real_spd_common = np.interp(common_t, ts_real_spd, real_spd, left=np.nan, right=np.nan)

    common_a = np.unique(np.concatenate([ts_sim_acc, ts_map_acc, ts_real_acc]))
    sim_acc_common  = np.interp(common_a, ts_sim_acc,  sim_acc,  left=np.nan, right=np.nan)
    map_acc_common  = np.interp(common_a, ts_map_acc,  map_acc,  left=np.nan, right=np.nan)
    real_acc_common = np.interp(common_a, ts_real_acc, real_acc, left=np.nan, right=np.nan)

    out_dir.mkdir(exist_ok=True)

    # pd.DataFrame({
    #     'time': common_t,
    #     'speed_real': real_spd_common,
    #     'speed_mapped': map_spd_common,
    #     'speed_sim': sim_spd_common
    # }).to_csv(out_dir / f"{name}_speeds.csv", index=False)

    # pd.DataFrame({
    #     'time': common_a,
    #     'accel_real': real_acc_common,
    #     'accel_mapped': map_acc_common,
    #     'accel_sim': sim_acc_common
    # }).to_csv(out_dir / f"{name}_accels.csv", index=False)

    fig, legend_ax, ax1, ax2 = create_combined_plot_layout()

    # Left plot: Speed
    ax1.plot(common_t, map_spd_common,  COLORS['mapped'], linewidth=4, label='mapped')
    ax1.plot(common_t, real_spd_common, COLORS['real'],   linewidth=2, label='real')
    ax1.plot(common_t, sim_spd_common,  COLORS['sim'],    linewidth=2, label='sim')
    ax1.set_xlabel("Time (s)", fontsize=12)
    ax1.set_ylabel("Speed (m/s)", fontsize=12)
    ax1.grid(True)

    # Right plot: Acceleration
    ax2.plot(common_a, map_acc_common,  COLORS['mapped'], linewidth=4, label='mapped')
    ax2.plot(common_a, real_acc_common, COLORS['real'],   linewidth=2, label='real')
    ax2.plot(common_a, sim_acc_common,  COLORS['sim'],    linewidth=2, label='sim')
    ax2.set_xlabel("Time (s)", fontsize=12)
    ax2.set_ylabel("Acceleration (m/s²)", fontsize=12)
    ax2.grid(True)

    # Legend & Title
    legend_ax.axis("off")
    legend_ax.text(0.0, 0.5, f"Throttle: {name} brake", fontsize=14, fontweight='bold', va='center')
    handles, labels = ax1.get_legend_handles_labels()
    legend_ax.legend(handles, labels, loc='center left', bbox_to_anchor=(0.5, 0.5),
                    ncol=3, frameon=False, fontsize=10)

    # Save
    plt.savefig(out_dir / f"{name}_combined.pdf", bbox_inches='tight')
    plt.close()

def main():
    base = Path(__file__).resolve().parent / 'ACTUATION_OUTPUTS' / 'BRAKING'
    sim_dir = base / 'sim'
    real_dir = base / 'real'
    results = Path("PROCESSED RESULTS/RQ2/Braking/plots")
    for sim_path in sorted(sim_dir.glob('*.bag')):
        name = sim_path.stem
        real_path = real_dir / f"{name}.bag"
        if real_path.exists():
            process_pair(name, sim_path, real_path, results)

if __name__ == '__main__':
    main()