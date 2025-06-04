#!/usr/bin/env python3
"""
process_sim_real.py

Batch-process real/ and sim/ ROS bag files under ACTUATION_OUTPUTS/FORWARD.
Calculates turning radii and arc lengths from trajectories via least-squares circle fit.
For each <name>.bag pair:
  - Reads topics with rosbag and extracts /going, /sim/euler, /donkey/pose
  - Filters to messages after first /going==True
  - Converts sim/euler to real frame
  - Shifts all trajectories to start at (0,0)
  - Estimates radius and arc length of real, mapped, and sim paths
  - Saves CSVs of shifted positions
  - Plots trajectories with annotated radii and lengths
  - Summarizes mean/std radii and lengths into results/radius_stats.csv
"""
import math
from pathlib import Path
import rosbag
import pandas as pd
import numpy as np
import tf
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

# Conversion constants
SIZE_FACTOR = 7.33
X_MAP_SHIFT = 48
Y_MAP_SHIFT = 50
# Plot colors
COLORS = {'real':'#2B83BA','mapped':'#FDAE61','sim':'#ABDDA4'}

# Topics
SIM_TOPIC = '/sim/euler'
REAL_TOPIC = '/donkey/pose'
GOING_TOPIC = '/going'


def calc_radius(center, points):
    cx, cy = center
    d = np.sqrt((points[:,0]-cx)**2 + (points[:,1]-cy)**2)
    return d - d.mean()

def rotate(points, degrees=0):
    """
    Rotate 2D points by a given angle (degrees).
    """
    theta = np.deg2rad(degrees)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    pts = np.atleast_2d(points)
    return pts.dot(R.T)

def estimate_radius(points):
    init = points.mean(axis=0)
    res = least_squares(calc_radius, init, args=(points,))
    cx, cy = res.x
    radii = np.sqrt((points[:,0]-cx)**2 + (points[:,1]-cy)**2)
    return radii.mean()


def extract_messages(bag, topic):
    return [(t.to_sec(), msg) for _, msg, t in bag.read_messages(topics=[topic])]


def process_pair(name, sim_path, real_path, out_dir):
    print(f"Processing {name}")
    # Read bags
    sim_bag = rosbag.Bag(str(sim_path))
    real_bag = rosbag.Bag(str(real_path))

    sim_data = extract_messages(sim_bag, SIM_TOPIC)
    real_data = extract_messages(real_bag, REAL_TOPIC)
    going = extract_messages(real_bag, GOING_TOPIC)

    # Find start time
    go_times = [t for t, m in going if getattr(m, 'data', False)]
    t0 = go_times[0] if go_times else real_data[0][0]

    # Filter & convert to XY
    def to_xy(traj, is_sim=False):
        pts = []
        for t, msg in traj:
            if t < t0: continue
            if is_sim:
                x = (msg.x - X_MAP_SHIFT)/SIZE_FACTOR
                y = (msg.z - Y_MAP_SHIFT)/SIZE_FACTOR
            else:
                x = msg.pose.position.x
                y = msg.pose.position.y
            pts.append((t - t0, x, y))
        return pts

    sim_pts    = to_xy(sim_data,  True)
    # Extract sim/euler from REAL bag for mapped trajectory
    real_sim_data = extract_messages(real_bag, SIM_TOPIC)
    mapped_pts = to_xy(real_sim_data, True)
    # Extract real /donkey/pose for real trajectory
    real_pts = to_xy(real_data, False)

    # Shift to origin
    def pts_arr(pts):
        if not pts: return np.zeros((0,2))
        arr = np.array([[x,y] for _,x,y in pts])
        return arr - arr[0]

    arr_sim    = pts_arr(sim_pts)
    arr_mapped = pts_arr(mapped_pts)
    arr_real   = pts_arr(real_pts)
    # Rotate all trajectories to align starting orientation to zero
    # Use first real pose orientation to compute initial yaw
    first_orientation_msg = None
    for t, msg in real_data:
        if t >= t0:
            first_orientation_msg = msg
            break
    if first_orientation_msg:
        # Extract quaternion
        q = [first_orientation_msg.pose.orientation.x,
             first_orientation_msg.pose.orientation.y,
             first_orientation_msg.pose.orientation.z,
             first_orientation_msg.pose.orientation.w]
        # Convert to Euler and take yaw (radians)
        _, _, yaw = tf.transformations.euler_from_quaternion(q)
        yaw_deg = math.degrees(yaw)
        # Rotate arrays
        arr_sim    = rotate(arr_sim, -yaw_deg)
        arr_mapped = rotate(arr_mapped, -yaw_deg)
        arr_real   = rotate(arr_real, -yaw_deg)

    # Estimate radii
    r_sim    = estimate_radius(arr_sim)
    r_mapped = estimate_radius(arr_mapped)
    r_real   = estimate_radius(arr_real)

    # Compute arc lengths
    def arc_length(arr):
        if arr.shape[0] < 2: return 0.0
        diffs = np.diff(arr, axis=0)
        return np.sum(np.sqrt((diffs**2).sum(axis=1)))

    l_sim    = arc_length(arr_sim)
    l_mapped = arc_length(arr_mapped)
    l_real   = arc_length(arr_real)

    # Print estimates
    print(f"  Radii (real,mapped,sim): {r_real:.2f}, {r_mapped:.2f}, {r_sim:.2f}")
    print(f"  Lengths (real,mapped,sim): {l_real:.2f}, {l_mapped:.2f}, {l_sim:.2f}")

    # Save CSVs
    # out_dir.mkdir(exist_ok=True)
    # pd.DataFrame(arr_real,   columns=['x','y']).to_csv(out_dir/f"{name}_real.csv",   index=False)
    # pd.DataFrame(arr_mapped, columns=['x','y']).to_csv(out_dir/f"{name}_mapped.csv", index=False)
    # pd.DataFrame(arr_sim,    columns=['x','y']).to_csv(out_dir/f"{name}_sim.csv",    index=False)

    # Plot
    from matplotlib.gridspec import GridSpec

    fig = plt.figure(figsize=(3,3.5))
    gs = GridSpec(3, 1, height_ratios=[0.1,0.2, 1], figure=fig)

    # Legend + title on top
    legend_ax = fig.add_subplot(gs[0])
    legend_ax.axis("off")
    legend_ax2 = fig.add_subplot(gs[1])
    legend_ax2.axis("off")
    legend_ax.text(0.0, 0.5, f"Steering: {name}", fontsize=14, fontweight='bold', va='center')

    # Trajectory plot
    ax = fig.add_subplot(gs[2])
    ax.set_aspect('equal')
    ax.plot(arr_mapped[:,0], arr_mapped[:,1], color=COLORS['mapped'], linewidth=5, label=f"mapped r={r_mapped:.1f}, l={l_mapped:.1f}")
    ax.plot(arr_real[:,0],   arr_real[:,1],   color=COLORS['real'],   linewidth=2, label=f"real r={r_real:.1f}, l={l_real:.1f}")
    ax.plot(arr_sim[:,0],    arr_sim[:,1],    color=COLORS['sim'],    linewidth=2, label=f"sim r={r_sim:.1f}, l={l_sim:.1f}")
    ax.set_xlabel('x (m)', fontsize=12)
    ax.set_ylabel('y (m)', fontsize=12)

    # Legend
    handles, labels = ax.get_legend_handles_labels()
    legend_ax2.legend(handles, labels, loc='center left', bbox_to_anchor=(0., 0.5),
                    ncol=1, frameon=False, fontsize=10)

    plt.savefig(out_dir / f"{name}_traj.pdf", bbox_inches='tight')
    plt.close()

    return r_real, r_mapped, r_sim, l_real, l_mapped, l_sim


def main():
    base    = Path(__file__).resolve().parent / 'ACTUATION_OUTPUTS' / 'STEERING'
    sim_dir = base/'sim'
    real_dir= base/'real'
    out_dir = Path("PROCESSED RESULTS/RQ2/Steering/plots")
    results = {}

    for sim_path in sorted(sim_dir.glob('*.bag')):
        name      = sim_path.stem
        real_path = real_dir/f"{name}.bag"
        if real_path.exists():
            results[name] = dict(zip(
                ['r_real','r_map','r_sim','l_real','l_map','l_sim'],
                process_pair(name, sim_path, real_path, out_dir)
            ))

    if not results:
        print("No pairs to process.")
        return

    df = pd.DataFrame(results).T
    print(df)
    # df.describe().to_csv(out_dir/'radius_stats.csv')

if __name__=='__main__':
    main()
