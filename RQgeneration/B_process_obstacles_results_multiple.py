import os
import json
import numpy as np
from pathlib import Path

def corners_to_center(corners):
    corners = np.array(corners)
    return np.mean(corners, axis=0)

def compute_center_distance(c1, c2):
    return float(np.linalg.norm(np.array(c1) - np.array(c2)))

def estimate_plane_normal(corners):
    corners = np.array(corners)
    v1 = corners[1] - corners[0]
    v2 = corners[2] - corners[0]
    normal = np.cross(v1, v2)
    norm = np.linalg.norm(normal)
    return normal / norm if norm != 0 else np.zeros(3)

def compute_angle_between_normals(n1, n2):
    dot_product = np.dot(n1, n2)
    dot_product = np.clip(dot_product, -1.0, 1.0)
    return float(np.degrees(np.arccos(dot_product)))

def extract_boxes(json_path, key):
    with open(json_path, 'r') as f:
        data = json.load(f)

    all_boxes = []
    for frame_data in data.get(key, []):
        for obs in frame_data.get("obstacles", []):
            all_boxes.append(obs["front_face_corners"])
    return all_boxes

def match_boxes_by_center_and_orientation(ref_boxes, cmp_boxes):
    matched = []
    used = set()

    ref_props = [(corners_to_center(b), estimate_plane_normal(b)) for b in ref_boxes]
    cmp_props = [(corners_to_center(b), estimate_plane_normal(b)) for b in cmp_boxes]

    for i, (ref_c, ref_n) in enumerate(ref_props):
        best_score = float('inf')
        best_j = -1
        best_data = None
        for j, (cmp_c, cmp_n) in enumerate(cmp_props):
            if j in used:
                continue
            center_dist = compute_center_distance(ref_c, cmp_c)
            angle_diff = compute_angle_between_normals(ref_n, cmp_n)
            score = center_dist + angle_diff * 0.01  # optional weight
            if score < best_score:
                best_score = score
                best_j = j
                best_data = {
                    "ref_center": ref_c.tolist(),
                    "cmp_center": cmp_c.tolist(),
                    "center_distance": center_dist,
                    "normal_angle_diff_deg": angle_diff
                }
        if best_j != -1:
            used.add(best_j)
            matched.append(best_data)
    return matched

def compare_3d_annotations(lidar_path, mixed_path, sim_path, output_path):
    lidar_key = "/lidar/pointcloud"
    mixed_key = "/lidar/pointcloud_mixed"
    sim_key = "/sim/pointcloud"

    ref_boxes = extract_boxes(lidar_path, lidar_key)
    mixed_boxes = extract_boxes(mixed_path, mixed_key)
    sim_boxes = extract_boxes(sim_path, sim_key)

    mixed_matches = match_boxes_by_center_and_orientation(ref_boxes, mixed_boxes)
    sim_matches = match_boxes_by_center_and_orientation(ref_boxes, sim_boxes)

    result = {
        "mixed_vs_real": {
            "average_center_distance": float(np.mean([m["center_distance"] for m in mixed_matches])) if mixed_matches else None,
            "average_normal_angle_diff_deg": float(np.mean([m["normal_angle_diff_deg"] for m in mixed_matches])) if mixed_matches else None,
            "matches": mixed_matches
        },
        "sim_vs_real": {
            "average_center_distance": float(np.mean([m["center_distance"] for m in sim_matches])) if sim_matches else None,
            "average_normal_angle_diff_deg": float(np.mean([m["normal_angle_diff_deg"] for m in sim_matches])) if sim_matches else None,
            "matches": sim_matches
        }
    }

    with open(output_path, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"Saved: {output_path.name}")

def process_folder(input_folder, output_folder):
    input_path = Path(input_folder)
    output_path = Path(output_folder)
    output_path.mkdir(parents=True, exist_ok=True)

    for lidar_file in input_path.glob("*_lidar_pointcloud_pc_annotations_frontface.json"):
        base = lidar_file.stem.replace("_lidar_pointcloud_pc_annotations_frontface", "")
        mixed_file = input_path / f"{base}_lidar_pointcloud_mixed_pc_annotations_frontface.json"
        sim_file = input_path / f"{base}_sim_pointcloud_pc_annotations_frontface.json"

        if not mixed_file.exists() or not sim_file.exists():
            print(f"[!] Missing paired files for {base}")
            continue

        output_file = output_path / f"{base}_3d_center_orientation_comparison.json"
        compare_3d_annotations(lidar_file, mixed_file, sim_file, output_file)

if __name__ == "__main__":
    input_folder = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/lidar/annotations_dual"
    output_folder = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ1/Lidar_mapping"
    process_folder(input_folder, output_folder)