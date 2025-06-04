import os
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def euclidean_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def average_spline_distance(spline1, spline2):
    min_len = min(len(spline1), len(spline2))
    distances = [euclidean_distance(spline1[i], spline2[i]) for i in range(min_len)]
    return float(np.mean(distances)), distances

# def plot_splines(msg_index, spline_ref, spline_cmp, output_dir, title):
#     plt.figure(figsize=(6, 6))
#     ref = np.array(spline_ref)
#     cmp = np.array(spline_cmp)

#     plt.plot(ref[:, 0], ref[:, 1], label='Reference', linewidth=2)
#     plt.plot(cmp[:, 0], cmp[:, 1], label='Comparison', linewidth=2)
#     plt.scatter(ref[:, 0], ref[:, 1], s=10, alpha=0.5)
#     plt.scatter(cmp[:, 0], cmp[:, 1], s=10, alpha=0.5)

#     plt.title(f"{title} | msg_index: {msg_index}")
#     plt.xlabel("X")
#     plt.ylabel("Y")
#     plt.legend()
#     plt.axis("equal")
#     plt.grid(True)

#     output_dir.mkdir(parents=True, exist_ok=True)
#     plt.savefig(output_dir / f"{title}_msg_{msg_index}.png")
#     plt.close()

def compare_splines(json_data, plot_dir):
    camera_data = {item["msg_index"]: item["spline"] for item in json_data.get("/camera", [])}
    print(f"Camera msg_index count: {len(camera_data)}")

    results = {
        "mixed_vs_real": [],
        "sim_vs_real": []
    }

    for source, key in [("mixed_vs_real", "/mixed_image"), ("sim_vs_real", "/sim/image")]:
        for item in json_data.get(key, []):
            msg_index = item["msg_index"]
            if msg_index not in camera_data:
                print(f"[{source}] Missing in /camera: msg_index {msg_index}")
                continue

            spline_ref = camera_data[msg_index]
            avg_dist, distances = average_spline_distance(spline_ref, item["spline"])
            results[source].append({
                "msg_index": msg_index,
                "average_distance": avg_dist,
                "distances": distances,
                "spline_ref": spline_ref,
                "spline_cmp": item["spline"]
            })
            # plot_splines(msg_index, spline_ref, item["spline"], plot_dir / source, source)
            print(f"[{source}] Processed msg_index: {msg_index}")
    return results

def process_folder(input_folder, output_folder):
    input_path = Path(input_folder)
    output_path = Path(output_folder)
    output_path.mkdir(parents=True, exist_ok=True)

    plot_dir = output_path / "plots"

    for json_file in input_path.glob("*.json"):
        with open(json_file, 'r') as f:
            data = json.load(f)
        result = compare_splines(data, plot_dir)

        output_file = output_path / json_file.name
        with open(output_file, 'w') as f:
            json.dump(result, f, indent=2)

# Example usage
if __name__ == "__main__":
    input_folder = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/camera/annotations_lanes"   # Replace with your folder path
    output_folder = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ1/Camera_mapping/lanes"
    process_folder(input_folder, output_folder)
