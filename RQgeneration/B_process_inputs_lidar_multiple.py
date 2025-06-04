#!/usr/bin/env python3
"""
annotate_bag_pointclouds_frontface_interactive_bottomup_dual.py

For each bag in sim_real/ and mixed.bag:
  1) For each of two obstacles:
     a) Show a bottom-up view to pick exactly ONE point (height).
     b) Preview the colored horizontal slice only.
     c) Pop up a second window showing that slice so you pick 4 corners.
  2) Show a single interactive 3D window of the original cloud + BOTH red boxes;
     you can rotate/zoom/pan until you close it.
  3) Write out annotations JSON per bag, including both obstacles per frame.
"""
import sys, json
from pathlib import Path

import rosbag
import numpy as np
import open3d as o3d
from sensor_msgs.point_cloud2 import read_points

# ---------- CONFIG ----------
BAG_DIR      = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/lidar")
SIM_REAL_DIR = BAG_DIR/"sim_real"
MIXED_BAG    = BAG_DIR/"mixed.bag"
OUT_DIR      = BAG_DIR.parent/"lidar"/"annotations_dual"
MAX_FRAMES   = 1
# ----------------------------------

def pick_plane_height(pcd, tol=0.3, window_title_prefix="1"):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(f"{window_title_prefix}) Bottom-up: pick height", 800, 600)
    vis.add_geometry(pcd)
    vc = vis.get_view_control()
    # top-down view
    vc.set_front([0, 0, -1]); vc.set_up([0, 1, 0])
    vc.set_lookat(pcd.get_center().tolist()); vc.set_zoom(0.5)
    print(f"→ CLICK exactly one point (height), then CLOSE")
    vis.run()
    picks = vis.get_picked_points(); vis.destroy_window()
    if len(picks) != 1:
        print(f"⚠️ got {len(picks)} picks; expected 1"); return None
    pts = np.asarray(pcd.points)
    height = float(pts[picks[0], 2])
    mask = np.isclose(pts[:, 2], height, atol=tol)
    slice_pts = pts[mask]
    if slice_pts.size == 0:
        print(f"⚠️ no points at height={height:.3f}"); return None
    # color by height gradient
    z = slice_pts[:,2]; zmin, zmax = z.min(), z.max()
    rng = zmax-zmin if zmax>zmin else 1.0
    norms = (z-zmin)/rng
    try:
        import matplotlib.pyplot as plt
        cmap = plt.get_cmap('viridis'); colors = [cmap(v)[:3] for v in norms]
    except ImportError:
        colors = [[v,v,v] for v in norms]
    slice_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(slice_pts))
    slice_pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    print(f"→ previewing slice at height={height:.3f}")
    o3d.visualization.draw_geometries([slice_pcd], window_name=f"{window_title_prefix}) Slice Preview", width=600, height=600)
    return height, slice_pts


def pick_front_corners_on_slice(slice_pts, pick_count=4, window_title_prefix="3"):
    slice_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(slice_pts))
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(f"{window_title_prefix}) Front: pick corners", 800, 600)
    vis.add_geometry(slice_pcd)
    print(f"→ CLICK {pick_count} points on slice, then CLOSE")
    vis.run(); picks = vis.get_picked_points(); vis.destroy_window()
    if len(picks)!=pick_count:
        print(f"⚠️ got {len(picks)} picks; expected {pick_count}"); return None
    return slice_pts[picks]


def show_cloud_with_two_boxes(pcd_full, corners_list, screenshot_path=None, res=2048):
    # prepare original cloud
    pcd_vis = pcd_full.paint_uniform_color([0.7,0.7,0.7])
    # create LineSets for each obstacle
    lines = [[0,1],[1,2],[2,3],[3,0]]
    boxes = []
    for corners in corners_list:
        cn = np.array(corners)
        center = cn.mean(axis=0); scaled = center + (cn-center)*1.2
        ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(scaled),
                                   o3d.utility.Vector2iVector(lines))
        ls.paint_uniform_color([1,0,0]); boxes.append(ls)
    vis = o3d.visualization.Visualizer()
    vis.create_window("4) Dual View: rotate & close", res, res)
    ro = vis.get_render_option(); ro.point_size=1; ro.line_width=5
    vis.add_geometry(pcd_vis)
    for b in boxes: vis.add_geometry(b)
    # force same view
    vc=vis.get_view_control(); vc.set_front([0,0,-1]); vc.set_up([0,1,0])
    vc.set_lookat(pcd_full.get_center().tolist()); vc.set_zoom(0.5)
    print("→ orbit/zoom/pan if desired, then CLOSE")
    vis.run()
    # recapture forced view
    vc=vis.get_view_control(); vc.set_front([0,0,-1]); vc.set_up([0,1,0])
    vc.set_lookat(pcd_full.get_center().tolist()); vc.set_zoom(0.5)
    vis.poll_events(); vis.update_renderer()
    if screenshot_path:
        vis.capture_screen_image(screenshot_path, do_render=True)
        print(f"→ saved screenshot: {screenshot_path}")
    vis.destroy_window()


def annotate_one(bag_path, topics):
    name = bag_path.stem
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    ann = {t: [] for t in topics}
    with rosbag.Bag(str(bag_path),'r') as bag:
        for topic in topics:
            print(f"\n{name}.bag ▶ topic {topic}")
            frame=0
            for _,msg,_ in bag.read_messages(topics=[topic]):
                if frame>=MAX_FRAMES: break
                pts=np.array(list(read_points(msg,field_names=("x","y","z"),skip_nans=True)))
                if pts.size==0: continue
                pcd=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
                obs_data=[]
                for i in [1,2]:
                    tag=f"{i}"
                    out = pick_plane_height(pcd, tol=0.3 if topic=="/lidar/pointcloud" else 0.3,
                                             window_title_prefix=tag)
                    if out is None: break
                    h,slice_pts = out
                    picked = pick_front_corners_on_slice(slice_pts, 4, window_title_prefix=tag)
                    if picked is None: break
                    obs_data.append({"plane_height":h,
                                     "front_face_corners":[list(map(float,p)) for p in picked]})
                if len(obs_data)!=2:
                    print("⚠️ skipped frame due to incomplete picks")
                    continue
                # show dual boxes
                fname=f"{name}_{topic.split('/')[1]}_{topic.split('/')[2]}_frame{frame}_dual_bbox.png"
                sp=OUT_DIR/fname
                show_cloud_with_two_boxes(pcd, [o["front_face_corners"] for o in obs_data], str(sp))
                ann[topic].append({"frame":frame, "obstacles":obs_data})
                frame+=1
            # write JSON
            outp=OUT_DIR/f"{name}_{topic.split('/')[0]}_{topic.split('/')[1]}_{topic.split('/')[2]}_pc_annotations_frontface.json"
            with open(outp,'w') as f: json.dump(ann, f, indent=2)
            print(f"→ wrote {outp}")


def list_topics(bag_path):
    print(f"--- {bag_path.name} topics ---")
    with rosbag.Bag(str(bag_path),'r') as bag:
        info=bag.get_type_and_topic_info()
        for t,ti in info.topics.items(): print(f"{t}: {ti.msg_type}, msgs={ti.message_count}")


def main():
    if not SIM_REAL_DIR.is_dir():
        print(f"Error: {SIM_REAL_DIR} missing",file=sys.stderr); sys.exit(1)
    MIXED_DIR=BAG_DIR/"mixed"
    if not MIXED_DIR.is_dir(): print(f"Warning: mixed missing, skipping mixed.",file=sys.stderr)
    for bag in sorted(SIM_REAL_DIR.glob("2_16*.bag")):
        annotate_one(bag,["/lidar/pointcloud"])
        # ,"/sim/pointcloud"
        # mp=MIXED_DIR/bag.name
        # if mp.is_file(): annotate_one(mp,["/lidar/pointcloud_mixed"])
        # else: print(f"Warning: {mp.name} not found, skipping.",file=sys.stderr)

if __name__=="__main__": main()
