#!/usr/bin/env python3
import sys, math, json
from pathlib import Path
import re

import rosbag
import pandas as pd
import numpy as np
import tf
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.stats import wasserstein_distance
from scipy.spatial import cKDTree

# --- Constants ---
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
GOING_TOPIC = '/going'

# --- Helpers ---
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

def compute_distance_traveled(pts: np.ndarray):
    if len(pts) < 2:
        return 0.0
    diffs = np.diff(pts, axis=0)
    return np.sum(np.linalg.norm(diffs, axis=1))

def compute_trajectory_diff(ref: np.ndarray, cmp: np.ndarray):
    if len(ref) == 0 or len(cmp) == 0:
        return None, None
    tree_ref = cKDTree(ref)
    tree_cmp = cKDTree(cmp)
    dists1, _ = tree_cmp.query(ref)
    dists2, _ = tree_ref.query(cmp)
    all_dists = np.concatenate([dists1, dists2])
    return float(np.mean(all_dists)), float(np.std(all_dists))

def resample_trajectory(traj: np.ndarray, num_points: int = 100) -> np.ndarray:
    if len(traj) < 2:
        return np.zeros((num_points, 2))
    cumulative_dist = np.cumsum(np.linalg.norm(np.diff(traj, axis=0), axis=1))
    cumulative_dist = np.insert(cumulative_dist, 0, 0)
    total_dist = cumulative_dist[-1]
    desired = np.linspace(0, total_dist, num_points)
    new_traj = np.zeros((num_points, 2))
    for dim in range(2):
        new_traj[:, dim] = np.interp(desired, cumulative_dist, traj[:, dim])
    return new_traj

def compute_wasserstein_trajectory(P: np.ndarray, Q: np.ndarray) -> float:
    if len(P) < 2 or len(Q) < 2:
        return np.nan
    P_resampled = resample_trajectory(P)
    Q_resampled = resample_trajectory(Q)
    return wasserstein_distance(P_resampled.flatten(), Q_resampled.flatten())

def base_run_name(full_name: str):
    match = re.match(r'^(\d+)', full_name)
    return match.group(1) if match else full_name

# --- Core Processing ---
def process_pair(name: str, sim_path: Path, real_path: Path):
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

    # Wasserstein distances
    wass_sim_vs_real = compute_wasserstein_trajectory(arr_sim, arr_real)
    wass_map_vs_real = compute_wasserstein_trajectory(arr_mapped, arr_real)

    resampled_real   = resample_trajectory(arr_real).flatten()
    resampled_sim    = resample_trajectory(arr_sim).flatten()
    resampled_mapped = resample_trajectory(arr_mapped).flatten()

    # Statistical tests
    pval_traj_sim, cohen_traj_sim = compute_pvalue_and_cohen(resampled_real, resampled_sim)
    pval_traj_map, cohen_traj_map = compute_pvalue_and_cohen(resampled_real, resampled_mapped)

    pval_speed_sim, cohen_speed_sim = compute_pvalue_and_cohen(real_spd_common, sim_spd_common)
    pval_speed_map, cohen_speed_map = compute_pvalue_and_cohen(real_spd_common, map_spd_common)

    return {
        "dist_real": compute_distance_traveled(arr_real),
        "dist_mapped": compute_distance_traveled(arr_mapped),
        "dist_sim": compute_distance_traveled(arr_sim),
        "avg_speed_real": np.nanmean(real_spd_common),
        "avg_speed_mapped": np.nanmean(map_spd_common),
        "avg_speed_sim": np.nanmean(sim_spd_common),
        "traj_diff_sim_vs_real": compute_trajectory_diff(arr_real, arr_sim)[0],
        "traj_diff_mapped_vs_real": compute_trajectory_diff(arr_real, arr_mapped)[0],
        "wass_sim_vs_real": wass_sim_vs_real,
        "wass_mapped_vs_real": wass_map_vs_real,
        "p_value_traj_sim_vs_real":pval_traj_sim,
        "d_value_traj_sim_vs_real":cohen_traj_sim,
        "p_value_traj_mapped_vs_real":pval_traj_map,
        "d_value_traj_mapped_vs_real":cohen_traj_map,
        "p_value_speed_sim_vs_real":pval_speed_sim,
        "d_value_speed_sim_vs_real":cohen_speed_sim,
        "p_value_speed_mapped_vs_real":pval_speed_map,
        "d_value_speed_mapped_vs_real":cohen_speed_map,
        # RAW
        "real_spd": real_spd_common.tolist(),
        "sim_spd": sim_spd_common.tolist(),
        "map_spd": map_spd_common.tolist(),
        "real_traj": resampled_real.tolist(),
        "sim_traj": resampled_sim.tolist(),
        "map_traj": resampled_mapped.tolist()
    }

from scipy.stats import ttest_ind
def compute_pvalue_and_cohen(real: np.ndarray, other: np.ndarray):
    """
    Computes two-sample t-test p-value and Cohen's d effect size between two arrays.
    """
    mask = ~np.isnan(real) & ~np.isnan(other)
    real = real[mask]
    other = other[mask]
    
    if len(real) < 2 or len(other) < 2:
        return np.nan, np.nan
    
    # P-value using Welch's t-test
    _, p_value = ttest_ind(real, other, equal_var=False)

    # Cohen's d
    pooled_std = np.sqrt((np.var(real, ddof=1) + np.var(other, ddof=1)) / 2)
    cohen_d = (np.mean(real) - np.mean(other)) / pooled_std if pooled_std != 0 else 0.0

    return p_value, cohen_d

# --- Main ---
def main():
    base     = Path(__file__).resolve().parent / 'ACTUATION_OUTPUTS' / 'FORWARD'
    sim_dir  = base / 'sim'
    real_dir = base / 'real'
    results  = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/Throttle")
    results.mkdir(parents=True, exist_ok=True)

    grouped_metrics = {}

    for sim_path in sorted(sim_dir.glob("*.bag")):
        name = sim_path.stem
        group = base_run_name(name)
        real_path = real_dir / f"{name}.bag"
        if not real_path.exists():
            print(f"[!] Missing real bag for {name}")
            continue

        metrics = process_pair(name, sim_path, real_path)
        grouped_metrics.setdefault(group, []).append(metrics)

    aggregated = {}

    excluded_keys = {"real_spd", "sim_spd", "map_spd", "real_traj", "sim_traj", "map_traj"}

    for group, runs in grouped_metrics.items():
        agg = {}
        for key in runs[0]:
            if key in excluded_keys:
                continue
            values = [r[key] for r in runs if r[key] is not None]
            agg[key] = {
                "mean": float(np.mean(values)) if values else None,
                "std": float(np.std(values)) if values else None
            }
        aggregated[group] = agg

    all_real_spd, all_sim_spd, all_map_spd = [], [], []
    all_real_traj, all_sim_traj, all_map_traj = [], [], []

    for group_runs in grouped_metrics.values():
        for metrics in group_runs:
            all_real_spd.extend(metrics.get("real_spd", []))
            all_sim_spd.extend(metrics.get("sim_spd", []))
            all_map_spd.extend(metrics.get("map_spd", []))
            all_real_traj.extend(metrics.get("real_traj", []))
            all_sim_traj.extend(metrics.get("sim_traj", []))
            all_map_traj.extend(metrics.get("map_traj", []))

    p_speed_sim, d_speed_sim = compute_pvalue_and_cohen(np.array(all_real_spd), np.array(all_sim_spd))
    p_speed_map, d_speed_map = compute_pvalue_and_cohen(np.array(all_real_spd), np.array(all_map_spd))
    p_traj_sim, d_traj_sim = compute_pvalue_and_cohen(np.array(all_real_traj), np.array(all_sim_traj))
    p_traj_map, d_traj_map = compute_pvalue_and_cohen(np.array(all_real_traj), np.array(all_map_traj))

    aggregated["global_stats"] = {
        "p_value_speed_sim_vs_real": p_speed_sim,
        "d_value_speed_sim_vs_real": d_speed_sim,
        "p_value_speed_mapped_vs_real": p_speed_map,
        "d_value_speed_mapped_vs_real": d_speed_map,
        "p_value_traj_sim_vs_real": p_traj_sim,
        "d_value_traj_sim_vs_real": d_traj_sim,
        "p_value_traj_mapped_vs_real": p_traj_map,
        "d_value_traj_mapped_vs_real": d_traj_map,
    }

    with open(results / "aggregated_summary.json", "w") as f:
        json.dump(aggregated, f, indent=2)
    print(f"Saved aggregated summary → {results / 'aggregated_summary.json'}")

if __name__ == "__main__":
    main()

