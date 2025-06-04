import rosbag
import numpy as np
import json
from pathlib import Path
from bisect import bisect_left

# Constants
max_time_diff = 0.05
SIZE_FACTOR = 7.33
SMOOTH_DEGREE = 150

def extract_speed_request_and_timestamps(bag, topic):
    speed, timestamps = [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        speed.append(float(msg.data))
        timestamps.append(t.to_sec())
    return speed, timestamps

def extract_pose_and_timestamps_sim(bag, topic):
    pose, timestamps = [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        if msg.name == "donkey":
            pose.append([float(msg.x)/SIZE_FACTOR, float(msg.z)/SIZE_FACTOR])
            timestamps.append(t.to_sec())
    return pose, timestamps

def extract_speed_data_and_timestamps(bag, topic):
    speed, timestamps = [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        speed.append(float(msg.data))
        timestamps.append(t.to_sec())
    return speed, timestamps

def initialize_timestamps(timestamps, initial):
    return [t - initial for t in timestamps]

def match_messages(t1, t2, m1, m2, max_diff):
    matched1, matched2, ts = [], [], []
    for i, t in enumerate(t1):
        diffs = np.abs(np.array(t2) - t)
        j = np.argmin(diffs)
        if diffs[j] <= max_diff:
            matched1.append(m1[i])
            matched2.append(m2[j])
            ts.append(t)
    return matched1, matched2, ts

def generate_range(start, end, num):
    return np.linspace(start, end, num).tolist()

def match_values_to_time_range(time_range, values, value_ts):
    matched, last = [], None
    for t in time_range:
        idx = bisect_left(value_ts, t)
        last = values[idx] if idx < len(values) else last
        matched.append(last)
    return matched

def compute_errors(ref, actual):
    return np.abs(np.array(ref) - np.array(actual))

def segment_errors_by_command(requests, errors):
    grouped = {}
    for r, e in zip(requests, errors):
        if r is None:
            continue
        grouped.setdefault(r, []).append(e)
    return grouped

def summarize_segmented_errors(grouped):
    return {str(k): {
        "mean": float(np.mean(v)),
        "std": float(np.std(v))
    } for k, v in grouped.items() if len(v) > 0}

def extract_real_speed_bag(path):
    with rosbag.Bag(path, 'r') as bag:
        real_req, real_req_ts = extract_speed_request_and_timestamps(bag, '/keyboard/speed')
        real_spd, real_spd_ts = extract_speed_data_and_timestamps(bag, '/donkey/speed')
        sim_pose, sim_pose_ts = extract_pose_and_timestamps_sim(bag, '/sim/euler')
        base_ts = min(real_spd_ts[0], real_req_ts[0], sim_pose_ts[0])
        real_spd_ts = initialize_timestamps(real_spd_ts, base_ts)
        real_req_ts = initialize_timestamps(real_req_ts, base_ts)
        sim_pose_ts = initialize_timestamps(sim_pose_ts, base_ts)

        pose = np.array(sim_pose)
        ts = np.array(sim_pose_ts)
        uniq = np.where((np.diff(pose[:, 0]) != 0) | (np.diff(pose[:, 1]) != 0))[0] + 1
        uniq = np.insert(uniq, 0, 0)
        pose, ts = pose[uniq], ts[uniq]

        dxy = np.linalg.norm(np.diff(pose, axis=0), axis=1)
        dt = np.diff(ts)
        speed = np.insert(np.nan_to_num(dxy / dt), 0, 0)

        poly_sim = np.polyfit(ts, speed, SMOOTH_DEGREE)
        sim_clean = np.polyval(poly_sim, ts)
        sim_clean[sim_clean < 0] = 0

        poly_real = np.polyfit(real_spd_ts, real_spd, SMOOTH_DEGREE)
        real_clean = np.polyval(poly_real, real_spd_ts)
        real_clean[real_clean < 0] = 0

        return real_req, real_req_ts, real_clean, sim_clean, real_spd_ts, ts

def extract_sim_speed_bag(path):
    with rosbag.Bag(path, 'r') as bag:
        req, req_ts = extract_speed_request_and_timestamps(bag, '/keyboard/speed')
        spd_raw, spd_ts = extract_speed_data_and_timestamps(bag, '/sim/speed')
        spd = [s / SIZE_FACTOR for s in spd_raw]
        pose, pose_ts = extract_pose_and_timestamps_sim(bag, '/sim/euler')
        pose, spd, ts = match_messages(pose_ts, spd_ts, pose, spd, max_time_diff)
        base_ts = min(ts[0], req_ts[0])
        ts = initialize_timestamps(ts, base_ts)
        req_ts = initialize_timestamps(req_ts, base_ts)

        poly_sim = np.polyfit(ts, spd, SMOOTH_DEGREE)
        sim_clean = np.polyval(poly_sim, ts)
        sim_clean[sim_clean < 0] = 0

        return req, req_ts, sim_clean, ts

def process_trial(index):
    mix_path = f"/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/ACTUATION_OUTPUTS/PID/real/pid_mix_{index}.bag"
    real_path = f"/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/ACTUATION_OUTPUTS/PID/real/pid_car_{index}.bag"

    sim_req, sim_req_ts, sim_spd, sim_ts = extract_sim_speed_bag(mix_path)
    real_req, real_req_ts, real_spd, mix_spd, real_ts, mix_ts = extract_real_speed_bag(real_path)

    offset = sim_req_ts[0] - real_req_ts[0]
    sim_ts = initialize_timestamps(sim_ts, offset)
    sim_req_ts = initialize_timestamps(sim_req_ts, offset)

    end = max(sim_ts[-1], mix_ts[-1], real_ts[-1])
    n = min(len(sim_ts), len(real_ts), len(mix_ts))
    timeline = generate_range(0, end, n)

    matched_sim = match_values_to_time_range(timeline, sim_spd, sim_ts)
    matched_real = match_values_to_time_range(timeline, real_spd, real_ts)
    matched_mix = match_values_to_time_range(timeline, mix_spd, mix_ts)
    matched_req = match_values_to_time_range(timeline, real_req, real_req_ts)

    errors_sim = compute_errors(matched_req, matched_sim)
    errors_real = compute_errors(matched_req, matched_real)
    errors_mix = compute_errors(matched_req, matched_mix)

    return {
        "sim": errors_sim,
        "real": errors_real,
        "mapped": errors_mix,
        "target": matched_req,
        "sim_raw": matched_sim,
        "real_raw": matched_real,
        "mapped_raw": matched_mix
    }

from scipy.stats import ttest_ind

def compute_pvalue_and_cohen(x, y):
    x, y = np.array(x), np.array(y)
    mask = ~np.isnan(x) & ~np.isnan(y)
    x, y = x[mask], y[mask]
    if len(x) < 2 or len(y) < 2:
        return np.nan, np.nan
    _, p = ttest_ind(x, y, equal_var=False)
    pooled_std = np.sqrt((np.var(x, ddof=1) + np.var(y, ddof=1)) / 2)
    d = (np.mean(x) - np.mean(y)) / pooled_std if pooled_std > 0 else 0.0
    return float(p), float(d)

def aggregate_all(trials):
    all_errors = {"sim": [], "real": [], "mapped": []}
    all_speeds = {"sim": [], "real": [], "mapped": []}
    all_targets = []
    per_target_errors = {"sim": [], "real": [], "mapped": []}
    per_target_speeds = {"sim": [], "real": [], "mapped": []}

    for trial in trials:
        all_targets.extend(trial["target"])
        for domain in ["sim", "real", "mapped"]:
            all_errors[domain].extend(trial[domain])
            all_speeds[domain].extend(trial[f"{domain}_raw"])
            per_target_errors[domain].append(segment_errors_by_command(trial["target"], trial[domain]))
            per_target_speeds[domain].append(segment_errors_by_command(trial["target"], trial[domain]))

    output = {}
    for domain in ["sim", "real", "mapped"]:
        domain_errors = all_errors[domain]
        output[domain] = {
            "Average speed error": {
                "mean": float(np.mean(domain_errors)),
                "std": float(np.std(domain_errors))
            },
            "Target speed error": {}
        }

        # Aggregate per target level
        all_target_errors = {}
        for trial_group in per_target_errors[domain]:
            for k, v in trial_group.items():
                all_target_errors.setdefault(k, []).extend(v)

        output[domain]["Target speed error"] = summarize_segmented_errors(all_target_errors)

    # --- p-values and d-values ---
    output["stats_vs_real"] = {
        "p_value_sim_vs_real": None,
        "d_value_sim_vs_real": None,
        "p_value_mapped_vs_real": None,
        "d_value_mapped_vs_real": None
    }

    output["raw_speed_vs_real"] = {
        "p_value_sim_vs_real": None,
        "d_value_sim_vs_real": None,
        "p_value_mapped_vs_real": None,
        "d_value_mapped_vs_real": None
    }

    # Global comparisons (all errors)
    p_sim, d_sim = compute_pvalue_and_cohen(all_errors["real"], all_errors["sim"])
    p_map, d_map = compute_pvalue_and_cohen(all_errors["real"], all_errors["mapped"])

    output["stats_vs_real"]["p_value_sim_vs_real"] = p_sim
    output["stats_vs_real"]["d_value_sim_vs_real"] = d_sim
    output["stats_vs_real"]["p_value_mapped_vs_real"] = p_map
    output["stats_vs_real"]["d_value_mapped_vs_real"] = d_map

    p_sim_speed, d_sim_speed = compute_pvalue_and_cohen(all_speeds["real"], all_speeds["sim"])
    p_map_speed, d_map_speed = compute_pvalue_and_cohen(all_speeds["real"], all_speeds["mapped"])

    output["raw_speed_vs_real"]["p_value_sim_vs_real"] = p_sim_speed
    output["raw_speed_vs_real"]["d_value_sim_vs_real"] = d_sim_speed
    output["raw_speed_vs_real"]["p_value_mapped_vs_real"] = p_map_speed
    output["raw_speed_vs_real"]["d_value_mapped_vs_real"] = d_map_speed

    return output

def main():
    trials = [process_trial(i) for i in range(1, 6)]
    summary = aggregate_all(trials)
    out_path = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/Pid/aggregated_summary.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"Saved → {out_path}")

if __name__ == "__main__":
    main()