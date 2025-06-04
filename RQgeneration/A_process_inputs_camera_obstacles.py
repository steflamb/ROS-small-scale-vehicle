#!/usr/bin/env python3
"""
annotate_bag_images.py

For each .bag in a directory, extract up to 10 frames from each of:
/camera, /sim/image, /mixed_image
Display each frame, let the user click 4 corners per obstacle (4 clicks for one obstacle,
8 clicks for two), then save:
  - annotated image with the polygon(s)
  - a JSON file per bag recording all corner lists
"""

import sys, json
from pathlib import Path

import rosbag
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np

# --- CONFIG ----------------------------------------------------------
BAG_DIR = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PERCEPTION_INPUTS/camera/obstacles")
OUT_DIR = BAG_DIR.parent / "annotations_obstacles"
TOPICS = ["/camera", "/sim/image", "/mixed_image"]
MAX_FRAMES_PER_TOPIC = 1
# ---------------------------------------------------------------------

def annotate_bag(bag_path, bridge):
    """
    Extract up to MAX_FRAMES_PER_TOPIC frames from each topic,
    let the user draw 4 corners per obstacle (4 or 8 clicks),
    save images + return annotations dict.
    """
    name = bag_path.stem
    is_double = name.startswith("2_")
    # how many corners total
    n_clicks = 8 if is_double else 4

    # prepare annotation structure
    ann = { topic: [] for topic in TOPICS }

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic in TOPICS:
            print(f"\n{name}.bag → topic {topic}:")
            count = 0

            for _, msg, _ in bag.read_messages(topics=[topic]):
                if count >= MAX_FRAMES_PER_TOPIC:
                    break

                # convert ROS Image → OpenCV
                try:
                    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                except Exception as e:
                    print(f"  ⚠️  failed to convert image: {e}")
                    break

                # show
                plt.figure(figsize=(6,6))
                plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                plt.title(f"{name} | {topic} [{count+1}/{MAX_FRAMES_PER_TOPIC}]")
                plt.axis("off")

                print(f"  → click {n_clicks} corners{' (2 obstacles)' if is_double else ''}, then close")
                pts = plt.ginput(n_clicks)
                plt.close()

                if len(pts) < n_clicks:
                    print("  ✖️  not enough clicks, skipping remainder of this topic")
                    break

                # parse corners into one or two lists
                boxes = []
                for i in range(0, n_clicks, 4):
                    corners = pts[i:i+4]
                    # draw polygon on copy of image
                    pts_np = np.array(corners, dtype=np.int32)
                    cv2.polylines(img, [pts_np.reshape(-1,1,2)], isClosed=True, color=(255,255,255), thickness=10)
                    # store as list of [x,y]
                    boxes.append([[int(x),int(y)] for x,y in corners])

                # save annotated image
                topic_sanitized = topic.strip("/").replace("/", "_")
                out_img_name = f"{name}_{topic_sanitized}_{count}.png"
                out_img_path = OUT_DIR / out_img_name
                cv2.imwrite(str(out_img_path), img)

                # record annotation
                ann[topic].append({
                    "frame_index": count,
                    "corners": boxes,
                    "annotated_image": out_img_name
                })

                count += 1

    return ann


def main():
    if not BAG_DIR.is_dir():
        print(f"Error: {BAG_DIR} is not a directory", file=sys.stderr)
        sys.exit(1)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    bridge = CvBridge()

    bags = sorted(BAG_DIR.glob("*.bag"))
    if not bags:
        print(f"No .bag files found in {BAG_DIR}", file=sys.stderr)
        sys.exit(0)

    for bag_file in bags:
        print(f"\n--- Annotating {bag_file.name} ---")
        annotations = annotate_bag(bag_file, bridge)
        # write per-bag JSON
        json_path = OUT_DIR / f"{bag_file.stem}_annotations.json"
        with open(json_path, "w") as jf:
            json.dump(annotations, jf, indent=2)
        print(f"Saved annotations → {json_path}")

    print("\nAll done.")


if __name__ == "__main__":
    main()
