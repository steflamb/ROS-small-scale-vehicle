
#!/usr/bin/env python3
"""
compare_bag_pointclouds_rotated.py

Enhanced comparison of sim, real, and mixed pointclouds from rosbag with a 180° rotation
around the Y-axis applied to the clouds before analysis and visualization.
  - Uses fixed BAG_DIR with subfolders sim_real/ and mixed/
  - Extracts up to MAX_FRAMES frames from /sim/pointcloud, /lidar/pointcloud, and /lidar/pointcloud_mixed
  - Rotates each pointcloud by 180° about Y-axis (flip X and Z)
  - Computes nearest-neighbor distance metrics (mean, max, std) for sim→real and sim→mixed per frame
  - Saves JSON metrics per frame and a comparison screenshot for the first frame to OUT_DIR
"""
import sys
import json
from pathlib import Path
import numpy as np
import open3d as o3d
import rosbag
from sensor_msgs.point_cloud2 import read_points

# ---------- CONFIG ----------
BAG_DIR      = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/lidar")
SIM_REAL_DIR = BAG_DIR / "sim_real"
MIXED_DIR    = BAG_DIR / "mixed"
OUT_DIR      = BAG_DIR.parent / "lidar" / "compare"
MAX_FRAMES   = 100
# ----------------------------------

def read_frames(bag_path, topic, max_frames):
    """
    Read up to max_frames frames from topic, return list of Nx3 arrays.
    """
    frames = []
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            pts = np.array(list(read_points(msg, field_names=('x','y','z'), skip_nans=True)))
            frames.append(pts)
            if len(frames) >= max_frames:
                break
    return frames


def rotate_y_180(points):
    """
    Rotate a pointcloud by 180 degrees around the Y-axis.
    This flips X and Z for each point.
    """
    rotated = points.copy()
    rotated[:, 0] *= -1  # flip X
    rotated[:, 2] *= -1  # flip Z
    return rotated


def compute_nn_metrics(src_pts, tgt_pts):
    """
    Compute nearest-neighbor distances from src_pts to tgt_pts.
    Returns dict with mean, max, std (floats) or None if empty.
    """
    if src_pts.size == 0 or tgt_pts.size == 0:
        return {'mean': None, 'max': None, 'std': None}
    tgt_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(tgt_pts))
    kd = o3d.geometry.KDTreeFlann(tgt_pcd)
    dists = []
    for p in src_pts:
        _, idx, _ = kd.search_knn_vector_3d(p, 1)
        closest = tgt_pts[idx[0]]
        dists.append(np.linalg.norm(p - closest))
    d = np.array(dists)
    return {'mean': float(d.mean()), 'max': float(d.max()), 'std': float(d.std())}


def visualize_and_save(sim_pts, real_pts, mixed_pts, name):
    """
    Show an Open3D window with sim (blue), real (green), mixed (red).
    Save screenshot of first frame to OUT_DIR/<name>_compare.png
    """
    sim_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sim_pts))
    sim_pcd.paint_uniform_color([0,0,1])
    real_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(real_pts))
    real_pcd.paint_uniform_color([0,1,0])
    mixed_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(mixed_pts))
    mixed_pcd.paint_uniform_color([1,0,0])

    vis = o3d.visualization.Visualizer()
    vis.create_window(f"Compare First Frame: {name}", width=1024, height=768)
    vis.add_geometry(sim_pcd)
    vis.add_geometry(real_pcd)
    vis.add_geometry(mixed_pcd)
    vis.run()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    img_path = OUT_DIR / f"{name}_compare_first.png"
    vis.capture_screen_image(str(img_path), do_render=True)
    print(f"→ saved comparison screenshot: {img_path}")
    vis.destroy_window()


def main():
    if not SIM_REAL_DIR.is_dir() or not MIXED_DIR.is_dir():
        print(f"Error: Required directories not found", file=sys.stderr)
        sys.exit(1)

    for sim_bag in sorted(SIM_REAL_DIR.glob("*.bag")):
        name = sim_bag.stem
        mixed_bag = MIXED_DIR / f"{name}.bag"
        if not mixed_bag.is_file():
            print(f"Warning: missing mixed bag for {name}, skipping.")
            continue

        # Read up to MAX_FRAMES
        sim_frames   = read_frames(sim_bag, "/sim/pointcloud", MAX_FRAMES)
        real_frames  = read_frames(sim_bag, "/lidar/pointcloud", MAX_FRAMES)
        mixed_frames = read_frames(mixed_bag, "/lidar/pointcloud_mixed", MAX_FRAMES)

        metrics = []
        for idx in range(MAX_FRAMES):
            sim_pts_raw   = sim_frames[idx]   if idx < len(sim_frames) else np.empty((0,3))
            real_pts_raw  = real_frames[idx]  if idx < len(real_frames) else np.empty((0,3))
            mixed_pts_raw = mixed_frames[idx] if idx < len(mixed_frames) else np.empty((0,3))

            # Rotate
            sim_pts   = rotate_y_180(sim_pts_raw)
            real_pts  = rotate_y_180(real_pts_raw)
            mixed_pts = rotate_y_180(mixed_pts_raw)

            # Compute metrics
            m_real  = compute_nn_metrics(sim_pts, real_pts)
            m_mixed = compute_nn_metrics(mixed_pts, real_pts)
            metrics.append({'frame': idx, 'sim_vs_real': m_real, 'sim_vs_mixed': m_mixed})

        # Save JSON metrics for all frames
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        json_path = OUT_DIR / f"{name}_compare_metrics.json"
        with open(json_path, 'w') as f:
            json.dump({'bag': name, 'metrics': metrics}, f, indent=2)
        print(f"→ wrote metrics: {json_path}")

        # Visualize first frame only
        if sim_frames and real_frames and mixed_frames:
            visualize_and_save(rotate_y_180(sim_frames[0]), rotate_y_180(real_frames[0]), rotate_y_180(mixed_frames[0]), name)

if __name__ == '__main__':
    main()
