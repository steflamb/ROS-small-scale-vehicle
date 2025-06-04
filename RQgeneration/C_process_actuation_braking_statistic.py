#!/usr/bin/env python3
import sys
import math
import re
import json
from pathlib import Path

import rosbag
import numpy as np
import tf
from scipy.signal import savgol_filter

# Constants
SIZE_FACTOR = 7.33
X_MAP_SHIFT = 48
Y_MAP_SHIFT = 50

SIM_TOPIC = '/sim/euler'
REAL_TOPIC = '/donkey/pose'

def rotate(points: np.ndarray, degrees: float = 0) -> np.ndarray:
    theta = np.deg2rad(degrees)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    return points.dot(R.T)

def detect_braking(acceleration, time, threshold_acceleration):
    for i, a in enumerate(acceleration):
        if a < threshold_acceleration:
            print(f"[DEBUG] Detected braking start at index {i} (acc={a:.3f})")
            return i
    print("[DEBUG] No braking detected")
    return None

def extract_messages(bag: rosbag.Bag, topic: str):
    return [(t.to_sec(), msg) for _, msg, t in bag.read_messages(topics=[topic])]

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

def compute_smoothed_acceleration(positions: np.ndarray, times: np.ndarray, window: int = 30, poly: int = 3):
    if len(times) < window or len(positions) < window:
        return np.array([]), np.array([])
    x = savgol_filter(positions[:, 0], window, poly)
    y = savgol_filter(positions[:, 1], window, poly)
    dt = np.diff(times)
    vx = np.diff(x) / dt
    vy = np.diff(y) / dt
    speed = np.sqrt(vx**2 + vy**2)
    smoothed_speed = savgol_filter(speed, window, poly)
    dv = np.diff(smoothed_speed)
    dt2 = dt[1:]
    acceleration = dv / dt2
    return times[2:], savgol_filter(acceleration, window, poly) / SIZE_FACTOR

def extract_throttle_value(filename: str):
    match = re.search(r'brake([0-9.]+)', filename)
    return match.group(1) if match else None

def process_pair(name: str, sim_path: Path, real_path: Path, out_dir: Path):
    sim_bag = rosbag.Bag(str(sim_path))
    real_bag = rosbag.Bag(str(real_path))

    def get_waypoint_time(bag):
        for _, _, t in bag.read_messages(topics=["/waypoints"]):
            return t.to_sec()
        return None

    t0_real = get_waypoint_time(real_bag) or extract_messages(real_bag, REAL_TOPIC)[0][0]
    t0_sim = get_waypoint_time(sim_bag) or extract_messages(sim_bag, SIM_TOPIC)[0][0]

    def to_xy(data, is_sim, t0):
        result = []
        for t, msg in data:
            if t < t0:
                continue
            if is_sim:
                x = (msg.x - X_MAP_SHIFT) / SIZE_FACTOR
                y = (msg.z - Y_MAP_SHIFT) / SIZE_FACTOR
            else:
                x = msg.pose.position.x
                y = msg.pose.position.y
            result.append((t - t0, x, y))
        return result

    sim_data = extract_messages(sim_bag, SIM_TOPIC)
    mapped_data = extract_messages(real_bag, SIM_TOPIC)
    real_data = extract_messages(real_bag, REAL_TOPIC)

    sim_pts = to_xy(sim_data, True, t0_sim)
    mapped_pts = to_xy(mapped_data, True, t0_real)
    real_pts = to_xy(real_data, False, t0_real)

    def to_arr(pts):
        if not pts:
            return np.zeros((0, 2))
        return np.array([[x, y] for _, x, y in pts]) - np.array([[x, y] for _, x, y in pts])[0]

    arr_sim = to_arr(sim_pts)
    arr_mapped = to_arr(mapped_pts)
    arr_real = to_arr(real_pts)

    q = real_data[0][1].pose.orientation
    yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    deg = math.degrees(yaw) + 90
    arr_sim = rotate(arr_sim, -deg)
    arr_mapped = rotate(arr_mapped, -deg)
    arr_real = rotate(arr_real, -deg)

    times_sim = np.array([t for t, _, _ in sim_pts])
    times_mapped = np.array([t for t, _, _ in mapped_pts])
    times_real = np.array([t for t, _, _ in real_pts])

    ts_sim_spd, sim_spd = compute_speed(arr_sim, times_sim)
    ts_map_spd, map_spd = compute_speed(arr_mapped, times_mapped)
    ts_real_spd, real_spd = compute_speed(arr_real, times_real)

    ts_sim_acc, sim_acc = compute_smoothed_acceleration(arr_sim, times_sim)
    ts_map_acc, map_acc = compute_smoothed_acceleration(arr_mapped, times_mapped)
    ts_real_acc, real_acc = compute_smoothed_acceleration(arr_real, times_real)

    common_a = np.unique(np.concatenate([ts_sim_acc, ts_map_acc, ts_real_acc]))
    sim_acc_common = np.interp(common_a, ts_sim_acc, sim_acc, left=np.nan, right=np.nan)
    map_acc_common = np.interp(common_a, ts_map_acc, map_acc, left=np.nan, right=np.nan)
    real_acc_common = np.interp(common_a, ts_real_acc, real_acc, left=np.nan, right=np.nan)

    common_t = np.unique(np.concatenate([ts_sim_spd, ts_map_spd, ts_real_spd]))
    sim_spd_common = np.interp(common_t, ts_sim_spd, sim_spd, left=np.nan, right=np.nan)
    map_spd_common = np.interp(common_t, ts_map_spd, map_spd, left=np.nan, right=np.nan)
    real_spd_common = np.interp(common_t, ts_real_spd, real_spd, left=np.nan, right=np.nan)

    threshold_acc = -0.2
    brake_start_idx = detect_braking(real_acc_common, common_a, threshold_acc)
    if brake_start_idx is None:
        return None

    brake_time = common_a[brake_start_idx]
    brake_idx = np.searchsorted(times_real, brake_time)
    brake_idx_sim = np.searchsorted(times_sim, brake_time)
    brake_idx_mapped = np.searchsorted(times_mapped, brake_time)

    braking_real = np.linalg.norm(arr_real[-1] - arr_real[brake_idx]) if brake_idx < len(arr_real) else None
    braking_sim = np.linalg.norm(arr_sim[-1] - arr_sim[brake_idx_sim]) / SIZE_FACTOR if brake_idx_sim < len(arr_sim) else None
    braking_mapped = np.linalg.norm(arr_mapped[-1] - arr_mapped[brake_idx_mapped]) if brake_idx_mapped < len(arr_mapped) else None

    return {
        'braking_real': braking_real,
        'braking_sim': braking_sim,
        'braking_mapped': braking_mapped,
        'avg_speed_real': float(np.nanmean(real_spd_common)),
        # 'std_speed_real': float(np.nanstd(real_spd_common)),
        'avg_speed_mapped': float(np.nanmean(map_spd_common)),
        # 'std_speed_mapped': float(np.nanstd(map_spd_common)),
        'avg_speed_sim': float(np.nanmean(sim_spd_common)),
        # 'std_speed_sim': float(np.nanstd(sim_spd_common)),
        'avg_accel_real': float(np.nanmean(real_acc_common)),
        # 'std_accel_real': float(np.nanstd(real_acc_common)),
        'avg_accel_mapped': float(np.nanmean(map_acc_common)),
        # 'std_accel_mapped': float(np.nanstd(map_acc_common)),
        'avg_accel_sim': float(np.nanmean(sim_acc_common)),
        # 'std_accel_sim': float(np.nanstd(sim_acc_common)),
        # RAW DATA
        'raw_accel_real': real_acc_common.tolist(),
        'raw_accel_sim': sim_acc_common.tolist(),
        'raw_accel_mapped': map_acc_common.tolist(),
        'raw_speed_real': real_spd_common.tolist(),
        'raw_speed_sim': sim_spd_common.tolist(),
        'raw_speed_mapped': map_spd_common.tolist()
    }

def compute_pvalue_and_cohen(real: np.ndarray, other: np.ndarray):
    mask = ~np.isnan(real) & ~np.isnan(other)
    real = real[mask]
    other = other[mask]
    if len(real) < 2 or len(other) < 2:
        return np.nan, np.nan
    from scipy.stats import ttest_ind
    _, p = ttest_ind(real, other, equal_var=False)
    pooled_std = np.sqrt((np.var(real, ddof=1) + np.var(other, ddof=1)) / 2)
    d = (np.mean(real) - np.mean(other)) / pooled_std if pooled_std != 0 else 0.0
    return p, d

def main():
    base = Path(__file__).resolve().parent / 'ACTUATION_OUTPUTS' / 'BRAKING'
    sim_dir = base / 'sim'
    real_dir = base / 'real'
    results = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/Braking")
    results.mkdir(parents=True, exist_ok=True)

    grouped_metrics = {}
    all_braking_real=[]
    all_braking_sim=[]
    all_braking_mapped=[]
    all_accel_real=[]
    all_accel_sim=[]
    all_accel_mapped=[]
    all_speed_real=[]
    all_speed_sim=[]
    all_speed_mapped=[]


    for sim_path in sorted(sim_dir.glob('*.bag')):
        name = sim_path.stem
        real_path = real_dir / f"{name}.bag"
        if not real_path.exists():
            continue

        throttle_value = extract_throttle_value(name)
        print(f"[INFO] Processing throttle={throttle_value} from file {name}")
        if throttle_value is None:
            print(f"[WARN] Skipping file {name}: unable to extract throttle")
            continue

        metrics = process_pair(name, sim_path, real_path, results)
        if metrics is None:
            print(f"[WARN] Skipping file {name}: no braking detected")
            continue

        grouped_metrics.setdefault(throttle_value, []).append(metrics)
        all_braking_real.append(metrics['braking_real'])
        all_braking_sim.append(metrics['braking_sim'])
        all_braking_mapped.append(metrics['braking_mapped'])

        all_accel_real.extend(metrics['raw_accel_real'])
        all_accel_sim.extend(metrics['raw_accel_sim'])
        all_accel_mapped.extend(metrics['raw_accel_mapped'])

        all_speed_real.extend(metrics['raw_speed_real'])
        all_speed_sim.extend(metrics['raw_speed_sim'])
        all_speed_mapped.extend(metrics['raw_speed_mapped'])

    excluded_keys = {
        'raw_accel_real', 'raw_accel_sim', 'raw_accel_mapped',
        'raw_speed_real', 'raw_speed_sim', 'raw_speed_mapped'
    }

    aggregated = {}
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

    p_speed_sim, d_speed_sim = compute_pvalue_and_cohen(np.array(all_speed_real), np.array(all_speed_sim))
    p_speed_map, d_speed_map = compute_pvalue_and_cohen(np.array(all_speed_real), np.array(all_speed_mapped))

    p_accel_sim, d_accel_sim = compute_pvalue_and_cohen(np.array(all_accel_real), np.array(all_accel_sim))
    p_accel_map, d_accel_map = compute_pvalue_and_cohen(np.array(all_accel_real), np.array(all_accel_mapped))

    # p_brake_sim, d_brake_sim = compute_pvalue_and_cohen(np.array(all_braking_real), np.array(all_braking_sim))
    # p_brake_map, d_brake_map = compute_pvalue_and_cohen(np.array(all_braking_real), np.array(all_braking_mapped))

    aggregated["global_stats"] = {
        "p_speed_sim_vs_real": p_speed_sim,
        "d_speed_sim_vs_real": d_speed_sim,
        "p_speed_mapped_vs_real": p_speed_map,
        "d_speed_mapped_vs_real": d_speed_map,
        "p_accel_sim_vs_real": p_accel_sim,
        "d_accel_sim_vs_real": d_accel_sim,
        "p_accel_mapped_vs_real": p_accel_map,
        "d_accel_mapped_vs_real": d_accel_map,
        # "p_brake_sim_vs_real": p_brake_sim,
        # "d_brake_sim_vs_real": d_brake_sim,
        # "p_brake_mapped_vs_real": p_brake_map,
        # "d_brake_mapped_vs_real": d_brake_map,
    }

    out_path = results / "aggregated_summary.json"
    with open(out_path, "w") as f:
        json.dump(aggregated, f, indent=2)
    print(f"[INFO] Saved aggregated summary to {out_path}")

if __name__ == '__main__':
    main()