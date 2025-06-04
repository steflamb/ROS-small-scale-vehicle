#!/usr/bin/env python3
import math
import json
from pathlib import Path
import rosbag
import pandas as pd
import numpy as np
import tf
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

SIZE_FACTOR = 7.33
X_MAP_SHIFT = 48
Y_MAP_SHIFT = 50
COLORS = {'real': '#2B83BA', 'mapped': '#FDAE61', 'sim': '#ABDDA4'}
SIM_TOPIC = '/sim/euler'
REAL_TOPIC = '/donkey/pose'
GOING_TOPIC = '/going'

def calc_radius(center, points):
    cx, cy = center
    d = np.sqrt((points[:,0]-cx)**2 + (points[:,1]-cy)**2)
    return d - d.mean()

def rotate(points, degrees=0):
    theta = np.deg2rad(degrees)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    return np.atleast_2d(points).dot(R.T)

def estimate_radius(points):
    init = points.mean(axis=0)
    res = least_squares(calc_radius, init, args=(points,))
    cx, cy = res.x
    radii = np.sqrt((points[:,0]-cx)**2 + (points[:,1]-cy)**2)
    return radii.mean()

def extract_messages(bag, topic):
    return [(t.to_sec(), msg) for _, msg, t in bag.read_messages(topics=[topic])]

def arc_length(arr):
    if arr.shape[0] < 2: return 0.0
    diffs = np.diff(arr, axis=0)
    return np.sum(np.linalg.norm(diffs, axis=1))

def compute_trajectory_diff(ref, cmp):
    if len(ref) == 0 or len(cmp) == 0:
        return None, None
    tree_ref = cKDTree(ref)
    tree_cmp = cKDTree(cmp)
    d1, _ = tree_cmp.query(ref)
    d2, _ = tree_ref.query(cmp)
    dists = np.concatenate([d1, d2])
    return float(np.mean(dists)), float(np.std(dists))

from scipy.stats import ttest_ind

def compute_pvalue_and_cohen(real: np.ndarray, other: np.ndarray):
    mask = ~np.isnan(real) & ~np.isnan(other)
    real = real[mask]
    other = other[mask]
    if len(real) < 2 or len(other) < 2:
        return np.nan, np.nan
    _, p = ttest_ind(real, other, equal_var=False)
    pooled_std = np.sqrt((np.var(real, ddof=1) + np.var(other, ddof=1)) / 2)
    d = (np.mean(real) - np.mean(other)) / pooled_std if pooled_std > 0 else 0
    return p, d

def process_pair(name, sim_path, real_path, out_dir):
    sim_bag = rosbag.Bag(str(sim_path))
    real_bag = rosbag.Bag(str(real_path))

    sim_data = extract_messages(sim_bag, SIM_TOPIC)
    real_data = extract_messages(real_bag, REAL_TOPIC)
    going = extract_messages(real_bag, GOING_TOPIC)
    t0 = next((t for t, m in going if getattr(m, 'data', False)), real_data[0][0])

    def to_xy(traj, is_sim=False):
        pts = []
        for t, msg in traj:
            if t < t0: continue
            x = (msg.x - X_MAP_SHIFT)/SIZE_FACTOR if is_sim else msg.pose.position.x
            y = (msg.z - Y_MAP_SHIFT)/SIZE_FACTOR if is_sim else msg.pose.position.y
            pts.append((t - t0, x, y))
        return pts

    sim_pts = to_xy(sim_data, True)
    mapped_pts = to_xy(extract_messages(real_bag, SIM_TOPIC), True)
    real_pts = to_xy(real_data, False)

    def pts_arr(pts):
        if not pts: return np.zeros((0, 2))
        arr = np.array([[x, y] for _, x, y in pts])
        return arr - arr[0]

    arr_sim = pts_arr(sim_pts)
    arr_mapped = pts_arr(mapped_pts)
    arr_real = pts_arr(real_pts)

    # Rotate to align with real yaw
    for t, msg in real_data:
        if t >= t0:
            q = [msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w]
            _, _, yaw = tf.transformations.euler_from_quaternion(q)
            deg = math.degrees(yaw)
            arr_sim = rotate(arr_sim, -deg)
            arr_mapped = rotate(arr_mapped, -deg)
            arr_real = rotate(arr_real, -deg)
            break

    r_real   = estimate_radius(arr_real)
    r_mapped = estimate_radius(arr_mapped)
    r_sim    = estimate_radius(arr_sim)

    l_real   = arc_length(arr_real)
    l_mapped = arc_length(arr_mapped)
    l_sim    = arc_length(arr_sim)

    diff_sim, std_sim = compute_trajectory_diff(arr_real, arr_sim)
    diff_mapped, std_mapped = compute_trajectory_diff(arr_real, arr_mapped)

    # Resample for Wasserstein
    resampled_real = resample_trajectory(arr_real).flatten()
    resampled_sim = resample_trajectory(arr_sim).flatten()
    resampled_mapped = resample_trajectory(arr_mapped).flatten()

    wass_sim = wasserstein_distance(resampled_real, resampled_sim)
    wass_mapped = wasserstein_distance(resampled_real, resampled_mapped)

    pval_traj_sim, cohen_traj_sim = compute_pvalue_and_cohen(resampled_real, resampled_sim)
    pval_traj_map, cohen_traj_map = compute_pvalue_and_cohen(resampled_real, resampled_mapped)

    return {
        "radius_real": r_real,
        "radius_mapped": r_mapped,
        "radius_sim": r_sim,
        "length_real": l_real,
        "length_mapped": l_mapped,
        "length_sim": l_sim,
        "trajectory_diff_sim_vs_real": diff_sim,
        "trajectory_diff_mapped_vs_real": diff_mapped,
        "wass_sim_vs_real": wass_sim,
        "wass_mapped_vs_real": wass_mapped,
        "p_value_traj_sim_vs_real":pval_traj_sim,
        "d_value_traj_sim_vs_real":cohen_traj_sim,
        "p_value_traj_mapped_vs_real":pval_traj_map,
        "d_value_traj_mapped_vs_real":cohen_traj_map,
        # RAW DATA
        "real_traj": resampled_real.tolist(),
        "sim_traj": resampled_sim.tolist(),
        "map_traj": resampled_mapped.tolist()
    }

from scipy.stats import wasserstein_distance

def resample_trajectory(traj: np.ndarray, num_points: int = 100) -> np.ndarray:
    if len(traj) < 2:
        return np.zeros((num_points, 2))
    cumulative_dist = np.cumsum(np.linalg.norm(np.diff(traj, axis=0), axis=1))
    cumulative_dist = np.insert(cumulative_dist, 0, 0)
    total_dist = cumulative_dist[-1]
    desired = np.linspace(0, total_dist, num_points)
    resampled = np.zeros((num_points, 2))
    for dim in range(2):
        resampled[:, dim] = np.interp(desired, cumulative_dist, traj[:, dim])
    return resampled

def main():
    base = Path(__file__).resolve().parent / 'ACTUATION_OUTPUTS' / 'STEERING'
    sim_dir = base / 'sim'
    real_dir = base / 'real'
    out_dir = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/Steering")
    out_dir.mkdir(exist_ok=True)

    all_results = {}
    excluded_keys = {"real_traj", "sim_traj", "map_traj"}

    all_real_traj = []
    all_sim_traj = []
    all_map_traj = []

    for sim_path in sorted(sim_dir.glob('*.bag')):
        name = sim_path.stem
        prefix = name.split('_')[0]
        real_path = real_dir / f"{name}.bag"
        if real_path.exists():
            result = process_pair(name, sim_path, real_path, out_dir)
            all_results.setdefault(prefix, []).append(result)

            all_real_traj.extend(result.get("real_traj", []))
            all_sim_traj.extend(result.get("sim_traj", []))
            all_map_traj.extend(result.get("map_traj", []))

    # Aggregate per prefix
    summary = {}
    for prefix, runs in all_results.items():
        agg = {}
        for key in runs[0]:
            if key in excluded_keys:
                continue
            values = [r[key] for r in runs if r[key] is not None]
            if values:
                agg[f"mean_{key}"] = float(np.mean(values))
                agg[f"std_{key}"] = float(np.std(values))
        summary[prefix] = agg

    # Global statistics on all resampled trajectories
    p_traj_sim, d_traj_sim = compute_pvalue_and_cohen(np.array(all_real_traj), np.array(all_sim_traj))
    p_traj_map, d_traj_map = compute_pvalue_and_cohen(np.array(all_real_traj), np.array(all_map_traj))

    summary["global_stats"] = {
        "p_value_traj_sim_vs_real": p_traj_sim,
        "d_value_traj_sim_vs_real": d_traj_sim,
        "p_value_traj_mapped_vs_real": p_traj_map,
        "d_value_traj_mapped_vs_real": d_traj_map
    }

    # Save final JSON
    json_path = out_dir / "steering_summary.json"
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"Saved: {json_path}")

if __name__ == "__main__":
    main()