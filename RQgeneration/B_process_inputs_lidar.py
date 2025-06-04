#!/usr/bin/env python3
"""
annotate_bag_pointclouds_frontface_interactive_bottomup.py

For each bag in sim_real/ and mixed.bag:
  1) Show a bottom-up view via camera control so you pick exactly ONE point (height).
  2) Preview the red horizontal slice only.
  3) Pop up a second window showing just that red slice so you pick 4 corners.
  4) Show a single interactive 3D window of the original cloud + the red box;
     you can rotate/zoom/pan until you close it.
  5) Write out annotations JSON per bag.
"""
import sys, json, math
from pathlib import Path

import rosbag
import numpy as np
import open3d as o3d
from sensor_msgs.point_cloud2 import read_points

# ---------- CONFIG ----------
BAG_DIR      = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/lidar")
SIM_REAL_DIR = BAG_DIR/"sim_real"
MIXED_BAG    = BAG_DIR/"mixed.bag"
OUT_DIR      = BAG_DIR.parent/"lidar"/"annotations"
MAX_FRAMES   = 1
# ----------------------------------

def pick_plane_height(pcd, tol=0.05):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window("1) Bottom-up: pick 1 point (height)", 800, 600)
    vis.add_geometry(pcd)
    vc = vis.get_view_control()
    # point camera straight down (-Z) with Y up
    vc.set_front([0, 0, -1])
    vc.set_up([0, 1, 0])
    vc.set_lookat(pcd.get_center().tolist())
    vc.set_zoom(0.5)
    print("→ CLICK exactly one point to define the plane height, then CLOSE")
    vis.run()
    picks = vis.get_picked_points()
    vis.destroy_window()
    if len(picks) != 1:
        print(f"⚠️ got {len(picks)} picks; expected 1")
        return None

    pts = np.asarray(pcd.points)
    height = float(pts[picks[0], 2])  # Z axis as height
    mask = np.isclose(pts[:, 2], height, atol=tol)
    slice_pts = pts[mask]
    if slice_pts.size == 0:
        print("⚠️ no points in that slice!")
        return None

    # color slice by height gradient
    z_vals = slice_pts[:, 2]
    z_min, z_max = z_vals.min(), z_vals.max()
    z_range = z_max - z_min if z_max > z_min else 1.0
    norms = (z_vals - z_min) / z_range
    try:
        import matplotlib.pyplot as plt
        cmap = plt.get_cmap('viridis')
        colors = [cmap(v)[:3] for v in norms]
    except ImportError:
        colors = [[v, v, v] for v in norms]

    slice_pcd = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(slice_pts)
    )
    slice_pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    print(f"→ previewing gradient horizontal slice at height={height:.3f} m")
    o3d.visualization.draw_geometries(
        [slice_pcd], window_name="2) Slice Preview", width=600, height=600
    )
    return height, slice_pts


def pick_front_corners_on_slice(slice_pts, pick_count=4):
    red_slice = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(slice_pts)
    )

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window("3) Front: pick 4 corners", 800, 600)
    vis.add_geometry(red_slice)
    print(f"→ CLICK exactly {pick_count} points on the RED slice, then CLOSE")
    vis.run()
    picks = vis.get_picked_points()
    vis.destroy_window()
    if len(picks) != pick_count:
        print(f"⚠️ got {len(picks)} picks; expected {pick_count}")
        return None
    return slice_pts[picks]


def show_cloud_with_box(pcd_full, corners, screenshot_path=None, width=2048, height=2048):
    # make point cloud semi-transparent by lowering point size and using a light gray color
    pcd_vis = pcd_full

    corners_np = np.array(corners)
    center = corners_np.mean(axis=0)
    scale = 1.2  # widening factor; >1 makes box wider
    expanded = center + (corners_np - center) * scale

    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(expanded),
        lines=o3d.utility.Vector2iVector([[0, 1], [1, 2], [2, 3], [3, 0]])
    )
    ls.paint_uniform_color([1, 0, 0])

    vis = o3d.visualization.Visualizer()
    # create a high-res window for better screenshots
    vis.create_window("4) Interactive view: rotate & close", width, height)
    ro = vis.get_render_option()
    ro.point_size = 1  # smaller points for more transparency effect
    ro.line_width = 5
    vis.add_geometry(pcd_vis)
    vis.add_geometry(ls)

    # force a consistent camera view for the screenshot
    vc = vis.get_view_control()
    vc.set_front([0, 0, -1])
    vc.set_up([0, 1, 0])
    vc.set_lookat(pcd_full.get_center().tolist())
    vc.set_zoom(0.5)

    print("→ orbit/zoom/pan as desired, then CLOSE")
    vis.run()

    # after interactive session, re-enforce the view and capture high-res screenshot
    vc = vis.get_view_control()
    vc.set_front([0, 0, -1])
    vc.set_up([0, 1, 0])
    vc.set_lookat(pcd_full.get_center().tolist())
    vc.set_zoom(0.5)
    vis.poll_events()
    vis.update_renderer()
    if screenshot_path:
        vis.capture_screen_image(screenshot_path, do_render=True)
        print(f"→ saved screenshot: {screenshot_path}")
    vis.destroy_window()


def annotate_one(bag_path, topics):
    name = bag_path.stem
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    ann = {t: [] for t in topics}

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic in topics:

            print(f"\n{name}.bag ▶ topic {topic} ")
            frame = 0
            for _, msg, _ in bag.read_messages(topics=[topic]):
                if frame >= MAX_FRAMES:
                    break

                pts3d = np.array(list(
                    read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                ))
                if pts3d.size == 0:
                    continue
                pcd_full = o3d.geometry.PointCloud(
                    o3d.utility.Vector3dVector(pts3d)
                )

                # choose wider slice for LiDAR topic
                if topic == "/lidar/pointcloud":
                    out = pick_plane_height(pcd_full, tol=0.2)
                else:
                    out = pick_plane_height(pcd_full)
                if out is None:
                    continue
                height, slice_pts = out

                picked = pick_front_corners_on_slice(slice_pts, pick_count=4)
                if picked is None:
                    continue

                real_corners = [[float(x), float(y), float(z)] for x, y, z in picked]

                # show original cloud + your box and capture screenshot
                fname = f"{name}_{topic.split('/')[0]}_{topic.split('/')[1]}_{topic.split('/')[2]}_frame{frame}_bbox.png"
                screenshot_path = OUT_DIR / fname
                show_cloud_with_box(pcd_full, real_corners, str(screenshot_path))

                ann[topic].append({
                    "frame": frame,
                    "plane_height": height,
                    "front_face_corners": real_corners
                })
                frame += 1

            out = OUT_DIR / f"{name}_{topic.split('/')[0]}_{topic.split('/')[1]}_{topic.split('/')[2]}_pc_annotations_frontface.json"
            with open(out, "w") as f:
                json.dump(ann, f, indent=2)
            print(f"→ wrote {out}")


def list_topics(bag_path):
    """
    Print all topics and their message counts/types in the given rosbag.
    """
    print(f"--- {bag_path.name} topics ---")
    with rosbag.Bag(str(bag_path),'r') as bag:
        info = bag.get_type_and_topic_info()
        for topic, topic_info in info.topics.items():
            print(f"{topic}: {topic_info.msg_type}, messages = {topic_info.message_count}")


def main():
    if not SIM_REAL_DIR.is_dir():
        print(f"Error: {SIM_REAL_DIR} missing", file=sys.stderr)
        sys.exit(1)

    MIXED_DIR = BAG_DIR / "mixed"
    if not MIXED_DIR.is_dir():
        print(f"Warning: mixed folder {MIXED_DIR} missing, skipping mixed.", file=sys.stderr)

    for bag in sorted(SIM_REAL_DIR.glob("80*.bag")):
        annotate_one(bag, ["/lidar/pointcloud", "/sim/pointcloud"])
        mixed_path = MIXED_DIR / bag.name
        if mixed_path.is_file():
            annotate_one(mixed_path, ["/lidar/pointcloud_mixed"])
        else:
            print(f"Warning: {mixed_path.name} not found in mixed folder, skipping.", file=sys.stderr)

if __name__ == "__main__":
    main()
