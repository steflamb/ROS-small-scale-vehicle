import sys
import json
from pathlib import Path
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# --- CONFIG ----------------------------------------------------------
BAG_DIR = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/camera/lanes")
OUT_DIR = BAG_DIR.parent / "annotations_lanes"
TOPICS = ["/camera", "/sim/image", "/mixed_image"]
MAX_FRAMES = 5
# ---------------------------------------------------------------------

def annotate_bag(bag_path, bridge):
    name = bag_path.stem
    ann = { topic: [] for topic in TOPICS }

    # --- Step 1: Index all messages by timestamp
    topic_msgs = {topic: [] for topic in TOPICS}
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=TOPICS):
            topic_msgs[topic].append((t.to_sec(), msg))

    if not topic_msgs["/camera"]:
        print(f"{name}: No /camera messages")
        return ann

    # --- Step 2: Choose 5 evenly spaced timestamps from /camera
    camera_times = [t for t, _ in topic_msgs["/camera"]]
    if len(camera_times) < MAX_FRAMES:
        print(f"{name}: Not enough /camera frames ({len(camera_times)})")
        return ann

    frame_times = np.linspace(camera_times[0], camera_times[-1], MAX_FRAMES)

    # --- Step 3: For each frame time, align other topics by nearest timestamp
    for frame_idx, target_time in enumerate(frame_times):
        for topic in TOPICS:
            times_msgs = topic_msgs[topic]
            if not times_msgs:
                continue

            # Find the closest timestamped message
            closest = min(times_msgs, key=lambda x: abs(x[0] - target_time))
            timestamp, msg = closest

            # Convert image
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # Show and collect clicks
            plt.figure(figsize=(8, 6))
            plt.imshow(rgb)
            height, width = rgb.shape[:2]
            for i in [3.5, 4, 4.25, 4.5, 5]:
                y = height * i / 10
                plt.hlines(y, xmin=0, xmax=width, colors='white', linestyles='--')
            plt.title(f"{name} | {topic} frame {frame_idx+1}/{MAX_FRAMES}")
            plt.axis('off')
            pts = plt.ginput(5)
            plt.close()

            if len(pts) < 5:
                print(f"{name} {topic} frame {frame_idx}: fewer than 5 points, skipping")
                continue

            pts_arr = np.array(pts)
            tck, _ = splprep([pts_arr[:,0], pts_arr[:,1]], s=0)
            x_fine, y_fine = splev(np.linspace(0,1,100), tck)

            # Draw overlay
            disp = img.copy()
            for x, y in pts_arr.astype(int):
                cv2.circle(disp, (x, y), 5, (0, 255, 0), -1)
            spline = np.vstack([x_fine, y_fine]).T.astype(np.int32)
            cv2.polylines(disp, [spline], isClosed=False, color=(255, 0, 0), thickness=2)

            # Save image
            topic_safe = topic.strip("/").replace("/", "_")
            out_name = f"{name}_{topic_safe}_frame{frame_idx}.png"
            out_path = OUT_DIR / out_name
            cv2.imwrite(str(out_path), disp)

            # Save data
            ann[topic].append({
                "msg_index": frame_idx,
                "clicks": [[float(x), float(y)] for x, y in pts_arr],
                "spline": [[int(x), int(y)] for x, y in spline.tolist()],
                "annotated_image": out_name
            })

    return ann

def main():
    if not BAG_DIR.is_dir():
        print(f"Error: {BAG_DIR} is not a directory", file=sys.stderr)
        sys.exit(1)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    bridge = CvBridge()

    bags = sorted(BAG_DIR.glob("left_counter*.bag"))
    if not bags:
        print(f"No .bag files found in {BAG_DIR}", file=sys.stderr)
        sys.exit(0)

    for bag_file in bags:
        print(f"\n--- Annotating {bag_file.name} ---")
        annotations = annotate_bag(bag_file, bridge)
        json_path = OUT_DIR / f"{bag_file.stem}_annotations.json"
        with open(json_path, 'w') as jf:
            json.dump(annotations, jf, indent=2)
        print(f"Saved annotations → {json_path}")

    print("\nAll done.")

if __name__ == '__main__':
    main()