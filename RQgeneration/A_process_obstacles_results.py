import os
import json
import numpy as np
from pathlib import Path

def corners_to_bbox(corners):
    """Convert a 4-corner polygon to an axis-aligned bounding box"""
    corners = np.array(corners)
    xmin = np.min(corners[:, 0])
    ymin = np.min(corners[:, 1])
    xmax = np.max(corners[:, 0])
    ymax = np.max(corners[:, 1])
    return [xmin, ymin, xmax, ymax]

def compute_iou(boxA, boxB):
    """Compute IoU between two axis-aligned bounding boxes"""
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
    if interArea == 0:
        return 0.0

    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou

def match_bounding_boxes_iou(ref_boxes, cmp_boxes):
    """Greedy 1-to-1 matching using highest IoU"""
    ref_bboxes = [corners_to_bbox(b) for b in ref_boxes]
    cmp_bboxes = [corners_to_bbox(b) for b in cmp_boxes]

    matched = []
    used_cmp = set()

    for i, ref in enumerate(ref_bboxes):
        best_match = None
        best_iou = -1
        best_j = -1
        for j, cmp in enumerate(cmp_bboxes):
            if j in used_cmp:
                continue
            iou = compute_iou(ref, cmp)
            if iou > best_iou:
                best_iou = iou
                best_j = j
        if best_j != -1:
            matched.append({
                "bbox_ref": ref_boxes[i],
                "bbox_cmp": cmp_boxes[best_j],
                "iou": best_iou
            })
            used_cmp.add(best_j)
    return matched

def compare_bounding_boxes(json_data):
    camera_data = {item["frame_index"]: item["corners"] for item in json_data.get("/camera", [])}
    print(f"Camera frame_index count: {len(camera_data)}")

    results = {
        "mixed_vs_real": [],
        "sim_vs_real": []
    }

    for source, key in [("mixed_vs_real", "/mixed_image"), ("sim_vs_real", "/sim/image")]:
        for item in json_data.get(key, []):
            frame_index = item["frame_index"]
            if frame_index not in camera_data:
                print(f"[{source}] Missing in /camera: frame_index {frame_index}")
                continue

            ref_boxes = camera_data[frame_index]
            cmp_boxes = item["corners"]
            matches = match_bounding_boxes_iou(ref_boxes, cmp_boxes)

            avg_iou = float(np.mean([m["iou"] for m in matches])) if matches else None

            results[source].append({
                "frame_index": frame_index,
                "average_iou": avg_iou,
                "matches": matches
            })
            print(f"[{source}] Processed frame_index: {frame_index}")
    return results

def process_folder(input_folder, output_folder):
    input_path = Path(input_folder)
    output_path = Path(output_folder)
    output_path.mkdir(parents=True, exist_ok=True)

    for json_file in input_path.glob("*.json"):
        with open(json_file, 'r') as f:
            data = json.load(f)
        result = compare_bounding_boxes(data)

        output_file = output_path / json_file.name
        with open(output_file, 'w') as f:
            json.dump(result, f, indent=2)


# Example usage
if __name__ == "__main__":
    input_folder = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/camera/annotations_obstacles"  # Change to your actual folder
    output_folder = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ1/Camera_mapping/obstacles"
    process_folder(input_folder, output_folder)