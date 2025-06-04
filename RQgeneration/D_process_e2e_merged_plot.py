import os
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from bagpy import bagreader
from pathlib import Path

SIZE_FACTOR = 7.33
X_MAP_SHIFT = 48
Y_MAP_SHIFT = 50

COLORS = {
    'real': '#2B83BA',
    'sim': '#ABDDA4',
    'mixed_obstacles': '#D95F02',
    'mixed_entire': '#7570B3',
    'vil': '#E7298A'
}

TRAJ_TYPES = ['real', 'sim', 'mixed_obstacles', 'mixed_entire', 'vil']

def draw_obstacle(ax, x, y, yaw, width=0.2, length=0.6, color="red"):
    rect = np.array([
        [-length/2, -width/2],
        [ length/2, -width/2],
        [ length/2,  width/2],
        [-length/2,  width/2]
    ])
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rotated = rect @ R.T
    translated = rotated + np.array([x, y])
    polygon = plt.Polygon(translated, color=color, alpha=0.5)
    ax.add_patch(polygon)

def load_pose(path, topic, xkey, ykey):
    b = bagreader(path)
    try:
        msg_path = b.message_by_topic(topic)
        if not os.path.exists(msg_path):
            raise FileNotFoundError(f"CSV not found for topic {topic}")
        df = pd.read_csv(msg_path)
        if xkey not in df.columns or ykey not in df.columns:
            raise KeyError(f"Columns {xkey} or {ykey} not in {msg_path}")

        x = df[xkey]
        y = df[ykey]

        # Apply shift/scale only for simulation topic
        if topic == "/sim/euler":
            x = (x - X_MAP_SHIFT) / SIZE_FACTOR
            y = (y - Y_MAP_SHIFT) / SIZE_FACTOR

        return np.vstack([x, y]).T
    except Exception as e:
        raise RuntimeError(f"Failed to load pose from {path} [{topic}]: {e}")

def main():
    path_prefix = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/E2E/"
    path_suffix = ".bag"
    map_path = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/Nominal.json"

    with open(map_path, 'r') as f:
        map_data = json.load(f)
    wp = (np.array(map_data["sim_waypoint_list"]) - np.array([X_MAP_SHIFT, Y_MAP_SHIFT])) / SIZE_FACTOR
    left = (np.array(map_data["left_margin"]) - np.array([X_MAP_SHIFT, Y_MAP_SHIFT])) / SIZE_FACTOR
    right = (np.array(map_data["right_margin"]) - np.array([X_MAP_SHIFT, Y_MAP_SHIFT])) / SIZE_FACTOR

    OBSTACLE_OVERRIDES = {
        "red": [
            {"x": 42.9615, "y": 58.7911, "yaw": 81.8364},
            {"x": 60.0200, "y": 40.5686, "yaw": 80.2531},
        ],
        "green": [
            {"x": 35.2869, "y": 44.6222, "yaw": 148.4602},
            {"x": 59.3340, "y": 57.7600, "yaw": 143.9614},
        ],
    }

    for test in ["green", "red"]:
        fig, axes = plt.subplots(1, 5, figsize=(20, 4.5), sharex=True, sharey=True)
        fig.suptitle(f"Trajectory Overview – {test.capitalize()}", fontsize=14, weight="bold")

        for idx, traj_type in enumerate(TRAJ_TYPES):
            ax = axes[idx]
            ax.set_title(traj_type.replace("_", " ").title(), fontsize=10)

            # Plot lane center and margins
            ax.plot(wp[:, 0], wp[:, 1], '--', color='black', linewidth=0.1, label="Center")
            ax.plot(left[:, 0], left[:, 1], '--', color='gray', linewidth=0.1)
            ax.plot(right[:, 0], right[:, 1], '--', color='gray', linewidth=0.1)

            # Plot obstacles
            for obs in OBSTACLE_OVERRIDES[test]:
                x = (obs["x"] - X_MAP_SHIFT) / SIZE_FACTOR
                y = (obs["y"] - Y_MAP_SHIFT) / SIZE_FACTOR
                yaw = np.deg2rad(obs["yaw"] + 45)
                draw_obstacle(ax, x, y, yaw, color='red')

            base_color = COLORS[traj_type]
            for i in range(1, 6):
                bag_path = os.path.join(path_prefix, traj_type, f"{test}{i}{path_suffix}")
                try:
                    print(traj_type)
                    if traj_type == "sim":
                        points = load_pose(bag_path, "/sim/euler", "x", "z")
                    else:
                        points = load_pose(bag_path, "/donkey/pose", "pose.position.x", "pose.position.y")
                        # print(traj_type)
                    alpha = 1
                    ax.plot(points[:, 0], points[:, 1], color=base_color, alpha=alpha, linewidth=5)
                except Exception as e:
                    print(f"Skipping {traj_type}/{test}{i}: {e}")

            ax.set_xlim([-2.5, 2.5])
            ax.set_ylim([-2.5, 2.5])
            ax.set_aspect('equal')
            ax.grid(True, linestyle="--", alpha=0.5)
            if idx == 0:
                ax.set_ylabel("y [m]")
            ax.set_xlabel("x [m]")

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        save_dir = Path("PROCESSED RESULTS/RQ3/E2E/plots")
        save_dir.mkdir(parents=True, exist_ok=True)
        out_path = save_dir / f"{test}_overview.pdf"
        plt.savefig(out_path)
        plt.show()
        plt.close()
        print(f"Saved {test}_overview.pdf")

if __name__ == '__main__':
    main()