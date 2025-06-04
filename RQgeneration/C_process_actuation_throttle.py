#!/usr/bin/env python3
import sys, math
from pathlib import Path

import rosbag
import pandas as pd
import numpy as np
import tf
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# Conversion constants
SIZE_FACTOR = 7.33
X_MAP_SHIFT = 48
Y_MAP_SHIFT = 50

# Plot colors
COLORS = {
    'real':   '#2B83BA',
    'mapped': '#FDAE61',
    'sim':    '#ABDDA4'
}

# Topics
SIM_TOPIC   = '/sim/euler'
REAL_TOPIC  = '/donkey/pose'
GOING_TOPIC = '/going'


def rotate(points: np.ndarray, degrees: float = 0) -> np.ndarray:
    theta = np.deg2rad(degrees)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    return points.dot(R.T)


def extract_messages(bag: rosbag.Bag, topic: str):
    return [(t.to_sec(), msg) for _, msg, t in bag.read_messages(topics=[topic])]


def remove_speed_outliers(speed, z_thresh=2.5):
    if len(speed) == 0:
        return speed
    z = (speed - np.nanmean(speed)) / np.nanstd(speed)
    clipped = np.copy(speed)
    clipped[np.abs(z) > z_thresh] = np.nan
    return pd.Series(clipped).interpolate().fillna(method='bfill').fillna(method='ffill').values


def compute_speed(positions: np.ndarray, times: np.ndarray, window: int = 30, poly: int = 3):
    if len(times) < window or len(positions) < window:
        return np.array([]), np.array([])

    x = savgol_filter(positions[:, 0], window, poly)
    y = savgol_filter(positions[:, 1], window, poly)

    dt = np.diff(times)
    vx = np.diff(x) / dt
    vy = np.diff(y) / dt
    speed = np.sqrt(vx**2 + vy**2)
    smoothed_speed = savgol_filter(speed, window, poly)

    return times[1:], smoothed_speed


def process_pair(name: str, sim_path: Path, real_path: Path, out_dir: Path):
    print(f"Processing {name}…")
    sim_bag  = rosbag.Bag(str(sim_path))
    real_bag = rosbag.Bag(str(real_path))

    sim_data   = extract_messages(sim_bag,   SIM_TOPIC)
    real_data  = extract_messages(real_bag,  REAL_TOPIC)
    going_real = extract_messages(real_bag,  GOING_TOPIC)
    going_sim  = extract_messages(sim_bag,   GOING_TOPIC)

    t0_real = next((t for t,m in going_real if getattr(m,'data',False)), real_data[0][0])
    t0_sim  = next((t for t,m in going_sim  if getattr(m,'data',False)), sim_data[0][0])

    def to_xy(stream, is_sim: bool, t0: float):
        out = []
        for t,msg in stream:
            if t < t0: continue
            if is_sim:
                x = (msg.x - X_MAP_SHIFT)/SIZE_FACTOR
                y = (msg.z - Y_MAP_SHIFT)/SIZE_FACTOR
            else:
                x = msg.pose.position.x
                y = msg.pose.position.y
            out.append((t - t0, x, y))
        return out

    sim_pts    = to_xy(sim_data,    True,  t0_sim)
    mapped_pts = to_xy(extract_messages(real_bag, SIM_TOPIC), True,  t0_real)
    real_pts   = to_xy(real_data,   False, t0_real)

    def to_arr(pts):
        if not pts: return np.zeros((0,2))
        arr = np.array([[x,y] for _,x,y in pts])
        return arr - arr[0]

    arr_sim    = to_arr(sim_pts)
    arr_mapped = to_arr(mapped_pts)
    arr_real   = to_arr(real_pts)

    first_real_msg = next((m for t,m in real_data if t>=t0_real), None)
    if first_real_msg:
        q = [
            first_real_msg.pose.orientation.x,
            first_real_msg.pose.orientation.y,
            first_real_msg.pose.orientation.z,
            first_real_msg.pose.orientation.w
        ]
        _,_,yaw = tf.transformations.euler_from_quaternion(q)
        deg     = math.degrees(yaw)
        arr_sim    = rotate(arr_sim,    -deg)
        arr_mapped = rotate(arr_mapped, -deg)
        arr_real   = rotate(arr_real,   -deg)

    times_sim    = np.array([t for t,_,_ in sim_pts])
    times_mapped = np.array([t for t,_,_ in mapped_pts])
    times_real   = np.array([t for t,_,_ in real_pts])

    ts_sim_spd,    sim_spd    = compute_speed(arr_sim,    times_sim)
    ts_map_spd,    map_spd    = compute_speed(arr_mapped, times_mapped)
    ts_real_spd,   real_spd   = compute_speed(arr_real,   times_real)

    common_t = np.unique(np.concatenate([ts_sim_spd, ts_map_spd, ts_real_spd]))

    sim_spd_common  = np.interp(common_t, ts_sim_spd,   sim_spd,   left=np.nan, right=np.nan)
    map_spd_common  = np.interp(common_t, ts_map_spd,   map_spd,   left=np.nan, right=np.nan)
    real_spd_common = np.interp(common_t, ts_real_spd,  real_spd,  left=np.nan, right=np.nan)

    out_dir.mkdir(exist_ok=True)

    # pd.DataFrame(arr_real,   columns=['x','y']).to_csv(out_dir/f"{name}_real.csv",   index=False)
    # pd.DataFrame(arr_mapped, columns=['x','y']).to_csv(out_dir/f"{name}_mapped.csv", index=False)
    # pd.DataFrame(arr_sim,    columns=['x','y']).to_csv(out_dir/f"{name}_sim.csv",    index=False)

    # pd.DataFrame({
    #     'time':         common_t,
    #     'speed_real':   real_spd_common,
    #     'speed_mapped': map_spd_common,
    #     'speed_sim':    sim_spd_common
    # }).to_csv(out_dir/f"{name}_speeds.csv", index=False)

    from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec

    fig = plt.figure(figsize=(12, 2.6))
    fig.subplots_adjust(left=0, right=1, top=0.95, bottom=0, wspace=0.17, hspace=0.0)
    outer_gs = GridSpec(2, 2, figure=fig, width_ratios=[6, 1], height_ratios=[0.0000001, 200])

    # Left top: legend and title
    legend_ax = fig.add_subplot(outer_gs[0, 0])
    # Left bottom: trajectory
    ax1 = fig.add_subplot(outer_gs[1, 0])
    # Right: speed plot (spans both rows)

    right_gs = GridSpecFromSubplotSpec(3, 1, subplot_spec=outer_gs[:, 1], height_ratios=[0, 10, 0.78])

    # Optional: keep these invisible to adjust space visually
    fig.add_subplot(right_gs[0]).axis("off")  # Top spacer
    fig.add_subplot(right_gs[2]).axis("off")  # Bottom spacer

    # Actual speed plot in the middle
    ax2 = fig.add_subplot(right_gs[1])

    # Trajectory plot
    ax1.set_aspect('equal')
    ax1.plot(arr_mapped[:,0], arr_mapped[:,1], COLORS['mapped'], label='mapped', linewidth=4)
    ax1.plot(arr_real[:,0],   arr_real[:,1],   COLORS['real'],   label='real',   linewidth=2)
    ax1.plot(arr_sim[:,0],    arr_sim[:,1],    COLORS['sim'],    label='sim',    linewidth=2)

    all_x = np.concatenate([arr_sim[:,0], arr_mapped[:,0], arr_real[:,0]])
    all_y = np.concatenate([arr_sim[:,1], arr_mapped[:,1], arr_real[:,1]])
    ax1.set_xlim([all_x.min() - 0.5, all_x.max() + 0.5])
    ax1.set_ylim([- 0.5,+ 0.5])
    ax1.set_xlabel('x (m)',fontsize=18)
    ax1.set_ylabel('y (m)',fontsize=18)


    # Speed plot
    ax2.plot(common_t, map_spd_common,  COLORS['mapped'], label='mapped',  linewidth=4)
    ax2.plot(common_t, real_spd_common, COLORS['real'],   label='real',    linewidth=2)
    ax2.plot(common_t, sim_spd_common,  COLORS['sim'],    label='sim',     linewidth=2)
    ax2.set_xlabel('Time (s)',fontsize=18)
    ax2.set_ylabel('Speed (m/s)',fontsize=18)
    ax2.grid(True)
    all_speeds = np.concatenate([real_spd_common, map_spd_common, sim_spd_common])
    ax2.set_ylim(-0.1, np.nanmax(all_speeds) + 0.1)

    ax1.tick_params(axis='both', labelsize=18) 
    ax2.tick_params(axis='both', labelsize=18)

    # Legend & title
    legend_ax.axis("off")
    legend_ax.text(0.0, 0.5, f"Throttle: {name} for 3 seconds",
                   fontsize=20, fontweight='bold', va='center')
    handles, labels = ax1.get_legend_handles_labels()
    legend_ax.legend(handles, labels, loc='center left',
                     bbox_to_anchor=(0.5, 0.5), ncol=3,
                     fontsize=18, frameon=False)
    
    

    plt.savefig(out_dir/f"{name}_combined.pdf", bbox_inches='tight')
    plt.close()


def main():
    base     = Path(__file__).resolve().parent / 'ACTUATION_OUTPUTS' / 'FORWARD'
    sim_dir  = base / 'sim'
    real_dir = base / 'real'
    results  = Path("PROCESSED RESULTS/RQ2/Throttle/plots")

    if not sim_dir.is_dir() or not real_dir.is_dir():
        print("Error: need both 'sim/' and 'real/'!")
        sys.exit(1)

    for sim_path in sorted(sim_dir.glob('*.bag')):
        name      = sim_path.stem
        real_path = real_dir / f"{name}.bag"
        if real_path.exists():
            process_pair(name, sim_path, real_path, results)

if __name__ == '__main__':
    main()