import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
import matplotlib.pyplot as plt
import matplotlib
from pathlib import Path

from bisect import bisect_left

# Constants
max_time_diff = 0.05
SIZE_FACTOR = 7.33

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
            pose.append([float(msg.x)/SIZE_FACTOR, float(msg.z)/SIZE_FACTOR, float(msg.yaw)])
            timestamps.append(t.to_sec())
    return pose, timestamps

def extract_speed_data_and_timestamps(bag, topic):
    speed, timestamps = [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        speed.append(float(msg.data))
        timestamps.append(t.to_sec())
    return speed, timestamps

def extract_speed_sim_data_and_timestamps(bag, topic):
    speed, timestamps = [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        speed.append(float(msg.data) / SIZE_FACTOR)
        timestamps.append(t.to_sec())
    return speed, timestamps

def initialize_timestamps(timestamp_list, initial_timestamp):
    return [t - initial_timestamp for t in timestamp_list]

def match_messages(timestamps1, timestamps2, messages1, messages2, max_time_diff):
    matched_list_1, matched_list_2, timestamp_list = [], [], []
    for i, t1 in enumerate(timestamps1):
        diffs = np.abs(np.array(timestamps2) - t1)
        min_idx = np.argmin(diffs)
        if diffs[min_idx] <= max_time_diff:
            matched_list_1.append(messages1[i])
            matched_list_2.append(messages2[min_idx])
            timestamp_list.append(t1)
    return matched_list_1, matched_list_2, timestamp_list

def generate_range(start, end, values):
    return np.linspace(start, end, values).tolist()

def match_values_to_time_range(time_range, values_list, values_timestamps_list):
    matched_values = []
    last_valid_value = 0  # Default to 0 before any data is available

    for timestamp in time_range:
        pos = bisect_left(values_timestamps_list, timestamp)

        # Before first available data
        if pos == 0:
            matched_values.append(0)

        # Exact match or after first data
        elif pos < len(values_timestamps_list):
            # Go one step back to get last known good value
            last_valid_value = values_list[pos - 1]
            matched_values.append(last_valid_value)

        else:
            # Past last timestamp: hold the last value
            matched_values.append(0)

    return matched_values

def average_element_wise_distance(array1, array2):
    return np.mean(np.abs(np.array(array1) - np.array(array2)))

def find_change_indexes(array):
    return [i for i in range(1, len(array)) if array[i] != array[i - 1]]

def average_distances_by_segment(array1, array2):
    change_indexes = find_change_indexes(array1) + [len(array1)]
    segment_distances = []
    start = 0
    for end in change_indexes:
        avg_dist = average_element_wise_distance(array1[start:end], array2[start:end])
        segment_distances.append((start, end-1, array1[start], avg_dist))
        start = end
    return segment_distances

def extract_real_speed_bag(bag_file_path):
    with rosbag.Bag(bag_file_path, 'r') as bag:
        real_req, real_req_ts = extract_speed_request_and_timestamps(bag, '/keyboard/speed')
        real_spd, real_spd_ts = extract_speed_data_and_timestamps(bag, '/donkey/speed')
        sim_pose, sim_pose_ts = extract_pose_and_timestamps_sim(bag, '/sim/euler')
        min_ts = min(real_spd_ts[0], real_req_ts[0], sim_pose_ts[0])
        real_spd_ts = initialize_timestamps(real_spd_ts, min_ts)
        real_req_ts = initialize_timestamps(real_req_ts, min_ts)
        sim_pose_ts = initialize_timestamps(sim_pose_ts, min_ts)

        # Filter and clean
        pose = np.array(sim_pose)
        ts = np.array(sim_pose_ts)
        unique_idx = np.where((np.diff(pose[:, 0]) != 0) | (np.diff(pose[:, 1]) != 0))[0] + 1
        unique_idx = np.insert(unique_idx, 0, 0)
        pose = pose[unique_idx]
        ts = ts[unique_idx]

        dist = np.linalg.norm(np.diff(pose[:, :2], axis=0), axis=1)
        dt = np.diff(ts)
        sim_spd = np.nan_to_num(dist / dt)
        sim_spd = np.insert(sim_spd, 0, 0)

        poly_sim = np.polyfit(ts, sim_spd, 150)
        sim_clean = np.polyval(poly_sim, ts)
        sim_clean[sim_clean < 0] = 0

        poly_real = np.polyfit(real_spd_ts, real_spd, 150)
        real_clean = np.polyval(poly_real, real_spd_ts)
        real_clean[real_clean < 0] = 0

        return real_req, real_req_ts, real_clean, sim_clean, real_spd_ts, ts

def extract_sim_speed_bag(bag_file_path):
    with rosbag.Bag(bag_file_path, 'r') as bag:
        req, req_ts = extract_speed_request_and_timestamps(bag, '/keyboard/speed')
        spd, spd_ts = extract_speed_sim_data_and_timestamps(bag, '/sim/speed')
        pose, pose_ts = extract_pose_and_timestamps_sim(bag, '/sim/euler')
        pose, spd, global_ts = match_messages(pose_ts, spd_ts, pose, spd, max_time_diff)
        min_ts = min(global_ts[0], req_ts[0])
        global_ts = initialize_timestamps(global_ts, min_ts)
        req_ts = initialize_timestamps(req_ts, min_ts)
        return req, req_ts, spd, global_ts

def process_file_pair(index, results_dir):
    mix_path = f"/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/ACTUATION_OUTPUTS/PID/real/pid_mix_{index}.bag"
    real_path = f"/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/ACTUATION_OUTPUTS/PID/real/pid_car_{index}.bag"
    
    sim_req, sim_req_ts, sim_spd, sim_ts = extract_sim_speed_bag(mix_path)
    real_req, real_req_ts, real_spd, mix_spd, real_ts, mix_ts = extract_real_speed_bag(real_path)

    offset = sim_req_ts[0] - real_req_ts[0]
    sim_ts = initialize_timestamps(sim_ts, offset)
    sim_req_ts = initialize_timestamps(sim_req_ts, offset)

    end = max(real_ts[-1], mix_ts[-1], sim_ts[-1])
    n = min(len(real_ts), len(sim_ts), len(mix_ts))
    time_range = generate_range(0, end, n)

    matched_sim = match_values_to_time_range(time_range, sim_spd, sim_ts)
    matched_real = match_values_to_time_range(time_range, real_spd, real_ts)
    matched_mix = match_values_to_time_range(time_range, mix_spd, mix_ts)
    matched_requests = match_values_to_time_range(time_range, real_req, real_req_ts)

    matplotlib.rcParams.update({'font.size': 11})
    plt.figure(figsize=(8, 6))
    plt.plot(time_range, matched_real, color="#FDAE61", label="real", linewidth=4)
    plt.plot(time_range[1:], matched_mix[1:], color="#2B83BA", label="mapped", linewidth=4)
    plt.plot(time_range, matched_sim, color="#ABDDA4", label="simulation", linewidth=4)

    # Request curve
    req_ts_ext, req_ext = [0], [0]
    req_ts_ext.append(time_range[0] - 1e-10)
    req_ext.append(0)
    for i, v in enumerate(real_req):
        if i > 0 and matched_requests[i] != matched_requests[i-1]:
            req_ts_ext.append(time_range[i] - 1e-10)
            req_ext.append(matched_requests[i-1])
        req_ts_ext.append(time_range[i])
        req_ext.append(matched_requests[i])
    req_ts_ext.append(req_ts_ext[-1] + 1e-7)
    req_ext.append(0)
    req_ts_ext.append(time_range[-1])
    req_ext.append(0)
    plt.plot(req_ts_ext, req_ext, linestyle='--', color="black", label="target speed", linewidth=2)

    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title(f'Speed evolution over time (Trial {index})')
    plt.grid(True)
    plt.legend(loc=1)
    plt.xlim([0, sim_ts[-1]])
    plt.tight_layout()
    
    output_file = results_dir / f"speed_plot_trial_{index}.png"
    plt.savefig(output_file)
    plt.close()
    print(f"Saved plot: {output_file}")

def main():
    results_dir = Path("PROCESSED RESULTS/RQ2/Pid/plots")
    results_dir.mkdir(parents=True, exist_ok=True)

    for i in range(1, 6):
        process_file_pair(i, results_dir)

if __name__ == "__main__":
    main()