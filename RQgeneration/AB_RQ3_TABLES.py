import json
import statistics
import os

# Perception inputs
# RQ1


import matplotlib.pyplot as plt
import numpy as np

import matplotlib.pyplot as plt
import numpy as np

def draw_obstacle(ax, x, y, yaw, width, length, color):
    """
    Plot a rectangular obstacle on the given axis.
    """
    rect = np.array([
        [-length/2, -width/2],
        [ length/2, -width/2],
        [ length/2,  width/2],
        [-length/2,  width/2]
    ])
    # Rotate
    R = np.array([[np.cos(yaw), -np.sin(yaw)],
                  [np.sin(yaw),  np.cos(yaw)]])
    rotated = rect @ R.T
    # Translate
    translated = rotated + np.array([x, y])
    polygon = plt.Polygon(translated, color=color, alpha=0.5)
    ax.add_patch(polygon)

def plot_obstacles(
    obstacle_list_1, obstacle_list_2,
    filename="obstacle_plot.png",
    width=0.3, length=0.08,
    show=False, color="red",
    fov_origin=None, fov_yaw=None, fov_angle=60, fov_range=2.0
):
    """
    Plots two lists of obstacles on a top-down plot, draws field of view lines, and saves the image.
    
    Parameters:
    - fov_origin: (x, y) tuple indicating sensor position.
    - fov_yaw: orientation angle (in degrees).
    - fov_angle: total field of view angle (in degrees).
    - fov_range: range/length of the field of view lines.
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    xs, ys = [], []

    for obs in obstacle_list_1:
        x = obs["x"]
        y = obs["y"]
        yaw = np.deg2rad(obs["yaw"] + 45)
        draw_obstacle(ax, x, y, yaw, width, length, color="green")
        xs.append(x)
        ys.append(y)

    for obs in obstacle_list_2:
        x = obs["x"]
        y = obs["y"]
        yaw = np.deg2rad(obs["yaw"] + 45)
        draw_obstacle(ax, x, y, yaw, width, length, color="blue")
        xs.append(x)
        ys.append(y)

    # Draw FoV lines if specified
    if fov_origin and fov_yaw is not None:
        origin_x, origin_y = fov_origin
        yaw_rad = np.deg2rad(fov_yaw)
        half_angle = np.deg2rad(fov_angle / 2)

        for angle_offset in [-half_angle, half_angle]:
            angle = yaw_rad + angle_offset
            end_x = origin_x + fov_range * np.cos(angle)
            end_y = origin_y + fov_range * np.sin(angle)
            ax.plot([origin_x, end_x], [origin_y, end_y], color='orange', linestyle='--', linewidth=2)

        ax.plot(origin_x, origin_y, 'o', color='black')  # Sensor origin

    if xs and ys:
        margin = max(width, length) * 4
        ax.set_xlim(min(xs) - margin, max(xs) + margin)
        ax.set_ylim(min(ys) - margin, max(ys) + margin)

    ax.set_aspect('equal')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)
    plt.title("Obstacle Positions with Field of View")
    plt.savefig(filename)
    print(f"Plot saved to {filename}")

    if show:
        plt.show()
    else:
        plt.close()

# TRACKING QUALITY EEEXTERNALLY COLLECTED -> PROCESSED RESULTS/RQ1/Tracking_quality/tracking_measures.json
def process_rq1_tracking(base="/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/",filename="PROCESSED RESULTS/RQ1/Tracking_quality/tracking_measures.json"):
    filepath=base+filename
    with open(filepath, 'r') as file:
        data = json.load(file)
    
    measures = data.get("measures", [])
    
    if not measures:
        avg = std = None
    else:
        avg = statistics.mean(measures)
        std = statistics.stdev(measures) if len(measures) > 1 else 0.0
    
    return {
        "raw_tracking_quality": {
            "values": measures,
            "average": avg,
            "average_error": avg-240,
            "std": std
        }
    }



# A_process_inputs_camera_lanes -> A_process_lanes_results -> PROCESSED RESULTS/RQ1/Camera_mapping/lanes/*.json
# A_process_inputs_camera_obstacles -> A_process_obstacles_results -> PROCESSED RESULTS/RQ1/Camera_mapping/obstacles/*.json

import json
import os
import statistics

def process_rq1_camera(
    base="/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/",
    foldername_lanes="PROCESSED RESULTS/RQ1/Camera_mapping/lanes/",
    foldername_obstacles="PROCESSED RESULTS/RQ1/Camera_mapping/obstacles/"
):
    def compute_stats(values_by_file):
        # Flatten values across files
        all_values = [val for values in values_by_file.values() for val in values]
        if not all_values:
            return {"values": [], "average": None, "std": None, "by_file": values_by_file}
        avg = statistics.mean(all_values)
        std = statistics.stdev(all_values) if len(all_values) > 1 else 0.0
        return {
            "values": all_values,
            "average": avg,
            "std": std,
            "by_file": values_by_file
        }

    def extract_average_distances(folder_path):
        result = {
            "mixed_vs_real": {},
            "sim_vs_real": {}
        }
        for fname in os.listdir(folder_path):
            if not fname.endswith(".json"):
                continue
            fpath = os.path.join(folder_path, fname)
            with open(fpath, 'r') as file:
                data = json.load(file)
                for key in result:
                    values = []
                    for entry in data.get(key, []):
                        avg_dist = entry.get("average_distance")
                        if avg_dist is not None:
                            values.append(avg_dist)
                    if values:
                        result[key][fname] = values
        return result

    def extract_average_ious(folder_path):
        result = {
            "mixed_vs_real": {},
            "sim_vs_real": {}
        }
        for fname in os.listdir(folder_path):
            if not fname.endswith(".json"):
                continue
            fpath = os.path.join(folder_path, fname)
            with open(fpath, 'r') as file:
                data = json.load(file)
                for key in result:
                    values = []
                    for entry in data.get(key, []):
                        avg_iou = entry.get("average_iou")
                        if avg_iou is not None:
                            values.append(avg_iou)
                    if values:
                        result[key][fname] = values
        return result

    # Paths
    folderpath_lanes = os.path.join(base, foldername_lanes)
    folderpath_obstacles = os.path.join(base, foldername_obstacles)

    # Extract
    lane_data = extract_average_distances(folderpath_lanes)
    obstacle_data = extract_average_ious(folderpath_obstacles)

    # Build result
    return {
        "camera_mapping_lanes": {
            "mixed_vs_real": compute_stats(lane_data["mixed_vs_real"]),
            "sim_vs_real": compute_stats(lane_data["sim_vs_real"])
        },
        "camera_mapping_obstacles": {
            "mixed_vs_real": compute_stats(obstacle_data["mixed_vs_real"]),
            "sim_vs_real": compute_stats(obstacle_data["sim_vs_real"])
        }
    }


# B_process_inputs_lidar[ /_multiple] -> B_process_obstacles_results[ /_dual] -> PROCESSED RESULTS/RQ1/Lidar_mapping
def process_rq1_lidar(
    base="/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/",
    foldername="PROCESSED RESULTS/RQ1/Lidar_mapping"
):
    folderpath = os.path.join(base, foldername)

    def compute_stats(values_by_file):
        all_values = [v for sublist in values_by_file.values() for v in sublist]
        if not all_values:
            return {"values": [], "average": None, "std": None, "by_file": values_by_file}
        avg = statistics.mean(all_values)
        std = statistics.stdev(all_values) if len(all_values) > 1 else 0.0
        return {
            "values": all_values,
            "average": avg,
            "std": std,
            "by_file": values_by_file
        }

    def extract_lidar_metrics(folder_path):
        result = {
            "center_distance": {
                "mixed_vs_real": {},
                "sim_vs_real": {}
            },
            "angle_diff_deg": {
                "mixed_vs_real": {},
                "sim_vs_real": {}
            }
        }
        for fname in os.listdir(folder_path):
            if not fname.endswith(".json"):
                continue
            fpath = os.path.join(folder_path, fname)
            with open(fpath, 'r') as file:
                data = json.load(file)
                for key in ["mixed_vs_real", "sim_vs_real"]:
                    entry = data.get(key, {})
                    cd = entry.get("average_center_distance")
                    ad = entry.get("average_normal_angle_diff_deg")
                    if cd is not None:
                        result["center_distance"][key][fname] = [cd]
                    if ad is not None:
                        result["angle_diff_deg"][key][fname] = [ad]
        return result

    data = extract_lidar_metrics(folderpath)

    return {
        "lidar_mapping_center_distance": {
            "mixed_vs_real": compute_stats(data["center_distance"]["mixed_vs_real"]),
            "sim_vs_real": compute_stats(data["center_distance"]["sim_vs_real"])
        },
        "lidar_mapping_angle_diff_deg": {
            "mixed_vs_real": compute_stats(data["angle_diff_deg"]["mixed_vs_real"]),
            "sim_vs_real": compute_stats(data["angle_diff_deg"]["sim_vs_real"])
        }
    }

import re

def normalize_camera_filename(fname):
    """Extract base ID from camera annotation filename."""
    return fname.replace("_annotations.json", "")

def normalize_lidar_filename(fname):
    """Extract base ID from lidar filename before '__3d'."""
    return fname.split("__")[0]

def is_obstacle_file(fname):
    """Filter out lane-related or irrelevant files."""
    ignore_keywords = ["clock", "counter", "margin"]
    return all(kw not in fname for kw in ignore_keywords)

def process_image_metrics(folder_path):
    result = {}

    for fname in os.listdir(folder_path):
        if not fname.endswith(".json"):
            continue

        fpath = os.path.join(folder_path, fname)
        with open(fpath, "r") as f:
            data = json.load(f)

        metrics_summary = {
            "sim_vs_camera": {},
            "mixed_vs_camera": {}
        }

        # Get all metric keys from the first frame
        first_frame = data["metrics"][0]
        for mode in ["sim_vs_camera", "mixed_vs_camera"]:
            keys = first_frame[mode].keys()
            for key in keys:
                values = [frame[mode][key] for frame in data["metrics"] if key in frame[mode]]
                mean = statistics.mean(values)
                std = statistics.stdev(values) if len(values) > 1 else 0.0
                metrics_summary[mode][key] = (mean, std)

        result[fname] = metrics_summary

    return result

def generate_lane_table(camera_data):
    exp_data = {}

    # CAMERA lane distances
    for mode in ["sim_vs_real", "mixed_vs_real"]:
        for fname, values in camera_data["camera_mapping_lanes"][mode]["by_file"].items():
            exp_data.setdefault(fname, {})[mode] = values

    header = r"""
\begin{tabular}{l|c|c}
\hline
\textbf{Experiment} & \textbf{Sim Lane Dist (m)} & \textbf{Mix Lane Dist (m)} \\
\hline
"""
    rows = []

    for exp_id in sorted(exp_data.keys()):  # lexicographic sort
        row = [exp_id]
        for mode in ["sim_vs_real", "mixed_vs_real"]:
            values = exp_data[exp_id].get(mode, [])
            if values:
                mean = statistics.mean(values)
                std = statistics.stdev(values) if len(values) > 1 else 0.0
                row.append(f"{mean:.2f} ± {std:.2f}")
            else:
                row.append("-")
        rows.append(" & ".join(row) + r" \\")

    footer = r"""
\hline
\end{tabular}
"""
    return header + "\n".join(rows) + footer

def generate_obstacle_table(camera_data, lidar_data):
    exp_data = {}

    # CAMERA
    for mode in ["sim_vs_real", "mixed_vs_real"]:
        for fname, values in camera_data["camera_mapping_obstacles"][mode]["by_file"].items():
            if not fname.endswith("_annotations.json"):
                continue
            exp_id = normalize_camera_filename(fname)
            exp_data.setdefault(exp_id, {}).setdefault("camera", {})[mode] = values

    # LIDAR
    for lidar_type in ["lidar_mapping_center_distance", "lidar_mapping_angle_diff_deg"]:
        metric = "center" if "center" in lidar_type else "angle"
        for mode in ["sim_vs_real", "mixed_vs_real"]:
            for fname, values in lidar_data[lidar_type][mode]["by_file"].items():
                if "__3d" not in fname:
                    continue
                exp_id = normalize_lidar_filename(fname)
                exp_data.setdefault(exp_id, {}).setdefault("lidar", {}).setdefault(metric, {})[mode] = values

    header = r"""
\begin{tabular}{l|c|c|c|c|c|c}
\hline
\textbf{Experiment} & \multicolumn{3}{c|}{\textbf{Sim}} & \multicolumn{3}{c}{\textbf{Mix}} \\
 & Camera IoU & Lidar Dist & Lidar Angle & Camera IoU & Lidar Dist & Lidar Angle \\
\hline
"""
    rows = []

    def extract_sort_key(exp_id):
        # This will sort by the first number only (as int)
        parts = re.findall(r'\d+', exp_id)
        return int(parts[0]) if parts else 0

    for exp_id in sorted(exp_data.keys(), key=extract_sort_key):
        row = [exp_id]
        entry = exp_data[exp_id]

        for mode in ["sim_vs_real", "mixed_vs_real"]:
            # Camera IoU
            cam_vals = entry.get("camera", {}).get(mode, [])
            row.append(f"{statistics.mean(cam_vals):.2f}" if cam_vals else "-")

            # Lidar Center Distance
            cd_vals = entry.get("lidar", {}).get("center", {}).get(mode, [])
            row.append(f"{statistics.mean(cd_vals):.2f}" if cd_vals else "-")

            # Lidar Angle
            ad_vals = entry.get("lidar", {}).get("angle", {}).get(mode, [])
            row.append(f"{statistics.mean(ad_vals):.2f}" if ad_vals else "-")

        rows.append(" & ".join(row) + r" \\")

    footer = r"""
\hline
\end{tabular}
"""
    return header + "\n".join(rows) + footer


def process_lidar_metrics(folder_path):
    result = {}

    for fname in os.listdir(folder_path):
        if not fname.endswith(".json"):
            continue

        fpath = os.path.join(folder_path, fname)
        with open(fpath, "r") as f:
            data = json.load(f)

        summary = {
            "sim_vs_real": {},
            "sim_vs_mixed": {}
        }

        for mode in ["sim_vs_real", "sim_vs_mixed"]:
            keys = data["metrics"][0][mode].keys()
            for key in keys:
                values = [frame[mode][key] for frame in data["metrics"] if key in frame[mode]]
                mean = statistics.mean(values)
                std = statistics.stdev(values) if len(values) > 1 else 0.0
                summary[mode][key] = (mean, std)

        result[fname] = summary

    return result

def generate_summary_image_metric_table(metrics_by_file):
    from collections import defaultdict

    # Accumulate values across files per metric
    metrics_accumulator = {
        "sim_vs_camera": defaultdict(list),
        "mixed_vs_camera": defaultdict(list)
    }

    for file_data in metrics_by_file.values():
        for mode in ["sim_vs_camera", "mixed_vs_camera"]:
            for key, (mean, _) in file_data[mode].items():
                metrics_accumulator[mode][key].append(mean)

    # Build LaTeX table
    metric_keys = sorted(metrics_accumulator["sim_vs_camera"].keys())
    header = r"""
\begin{tabular}{l|c|c}
\hline
\textbf{Metric} & \textbf{Sim vs Camera} & \textbf{Mixed vs Camera} \\
\hline
"""
    rows = []
    for key in metric_keys:
        sim_vals = metrics_accumulator["sim_vs_camera"][key]
        mix_vals = metrics_accumulator["mixed_vs_camera"][key]

        def fmt(vals):
            mean = statistics.mean(vals)
            std = statistics.stdev(vals) if len(vals) > 1 else 0.0
            if abs(mean) < 0.01 or abs(std) < 0.01:
                return f"{mean:.2e} ± {std:.2e}"
            else:
                return f"{mean:.2f} ± {std:.2f}"

        row = f"{key} & {fmt(sim_vals)} & {fmt(mix_vals)}"
        rows.append(row + r" \\")

    footer = r"""
\hline
\end{tabular}
"""
    return header + "\n".join(rows) + footer


def generate_summary_lidar_metric_table(metrics_by_file):
    from collections import defaultdict

    metrics_acc = {
        "sim_vs_real": defaultdict(list),
        "sim_vs_mixed": defaultdict(list)
    }

    for file_data in metrics_by_file.values():
        for mode in ["sim_vs_real", "sim_vs_mixed"]:
            for key, (mean, _) in file_data[mode].items():
                metrics_acc[mode][key].append(mean)

    metric_keys = sorted(metrics_acc["sim_vs_real"].keys())
    header = r"""
\begin{tabular}{l|c|c}
\hline
\textbf{Metric} & \textbf{Sim vs Real} & \textbf{Sim vs Mixed} \\
\hline
"""
    rows = []
    for key in metric_keys:
        real_vals = metrics_acc["sim_vs_real"][key]
        mix_vals = metrics_acc["sim_vs_mixed"][key]

        def fmt(vals):
            mean = statistics.mean(vals)
            std = statistics.stdev(vals) if len(vals) > 1 else 0.0
            if abs(mean) < 0.01 or abs(std) < 0.01:
                return f"{mean:.2e} ± {std:.2e}"
            else:
                return f"{mean:.3f} ± {std:.3f}"

        row = f"{key} & {fmt(real_vals)} & {fmt(mix_vals)}"
        rows.append(row + r" \\")

    footer = r"""
\hline
\end{tabular}
"""
    return header + "\n".join(rows) + footer


def run_rq1():
    print(process_rq1_tracking())
    camera = process_rq1_camera()
    lidar = process_rq1_lidar()
    
    table = generate_obstacle_table(camera, lidar)

    obstacles_1 = [
    {"x": 0, "y": 0.4, "yaw": 45},
    {"x": 0, "y": 0.8, "yaw": 45},
    {"x": 0, "y": 1.2, "yaw": 45},
    {"x": 0, "y": 1.6, "yaw": 45}
    ]

    obstacles_2 = [
    {"x": -0.17, "y": 0.4, "yaw": 90-17},
    {"x": 0.17, "y": 0.4, "yaw": 17},
    {"x": -0.5, "y": 0.8, "yaw": 90-20},
    {"x": 0.5, "y": 0.8, "yaw": 20},
    {"x": -0.9, "y": 1.2, "yaw": 90-30},
    {"x": 0.9, "y": 1.2, "yaw": 30},
    {"x": -1.3, "y": 1.6, "yaw": 90-40},
    {"x": 1.3, "y": 1.6, "yaw": 40}
    ]

    plot_obstacles(obstacles_1,obstacles_2, filename="my_obstacles.png", show=True,fov_origin=(0, 0),fov_yaw=90,fov_angle=95,fov_range=2.5  )
    print(table)
    lane_table = generate_lane_table(camera)
    print(lane_table)
    print("DONE")

def run_rq1_2():
    base = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/camera/"
    compare_lanes = process_image_metrics(os.path.join(base, "compare_lanes"))
    compare_obstacles = process_image_metrics(os.path.join(base, "compare_obstacles"))

    print("=== LANE IMAGE METRICS ===")
    print(generate_summary_image_metric_table(compare_lanes))

    print("\n=== OBSTACLE IMAGE METRICS ===")
    print(generate_summary_image_metric_table(compare_obstacles))

def run_rq1_3():
    folder = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/lidar/compare"
    lidar_metrics = process_lidar_metrics(folder)
    table = generate_summary_lidar_metric_table(lidar_metrics)

    print("=== LIDAR METRIC SUMMARY ===")
    print(table)


run_rq1_2()
run_rq1_3()
