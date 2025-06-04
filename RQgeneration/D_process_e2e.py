import math
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from hampel import hampel
import numpy as np
import os
import tf
from scipy.optimize import least_squares
import re
from scipy.spatial import cKDTree
from typing import Callable


SIZE_FACTOR=7.33
X_MAP_SHIFT=48
Y_MAP_SHIFT=50

COLORS = {
    'real': '#2B83BA',         # Blue
    'mapped': '#FDAE61',       # Orange
    'sim': '#ABDDA4',          # Light green
    'mixed': '#D95F02',        # Burnt orange (deep contrast with 'mapped')
    'mixed_entire': '#7570B3', # Purple (distinct from all others)
    'vil': '#E7298A'           # Pink/magenta
}

import json
from pathlib import Path
from scipy.stats import ttest_ind
import numpy as np

def cohen_d(x, y):
    """Compute Cohen's d for two independent samples."""
    nx = len(x)
    ny = len(y)
    dof = nx + ny - 2
    pooled_std = np.sqrt(((nx - 1) * np.std(x, ddof=1) ** 2 + (ny - 1) * np.std(y, ddof=1) ** 2) / dof)
    return (np.mean(x) - np.mean(y)) / pooled_std if pooled_std != 0 else 0

def rotate(p, degrees=0):
    angle = np.deg2rad(degrees)
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    p = np.atleast_2d(p)

    return p @ R.T

def euclidean(p: np.ndarray, q: np.ndarray) -> float:
    d = p - q
    return math.sqrt(np.dot(d, d))

def _bresenham_pairs(x0: int, y0: int,
                     x1: int, y1: int) -> np.ndarray:
    """Generates the diagonal coordinates

    Parameters
    ----------
    x0 : int
        Origin x value
    y0 : int
        Origin y value
    x1 : int
        Target x value
    y1 : int
        Target y value

    Returns
    -------
    np.ndarray
        Array with the diagonal coordinates
    """
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    dim = max(dx, dy)
    pairs = np.zeros((dim, 2), dtype=np.int64)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx // 2
        for i in range(dx):
            pairs[i, 0] = x
            pairs[i, 1] = y
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        for i in range(dy):
            pairs[i, 0] = x
            pairs[i, 1] = y
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return pairs

def _fast_distance_matrix(p, q, diag, dist_func):
    n_diag = diag.shape[0]
    diag_max = 0.0
    i_min = 0
    j_min = 0
    p_count = p.shape[0]
    q_count = q.shape[0]

    # Create the distance array
    dist = np.full((p_count, q_count), np.inf, dtype=np.float64)

    # Fill in the diagonal with the seed distance values
    for k in range(n_diag):
        i0 = diag[k, 0]
        j0 = diag[k, 1]
        d = dist_func(p[i0], q[j0])
        diag_max = max(diag_max, d)
        dist[i0, j0] = d

    for k in range(n_diag - 1):
        i0 = diag[k, 0]
        j0 = diag[k, 1]
        p_i0 = p[i0]
        q_j0 = q[j0]

        for i in range(i0 + 1, p_count):
            if np.isinf(dist[i, j0]):
                d = dist_func(p[i], q_j0)
                if d < diag_max or i < i_min:
                    dist[i, j0] = d
                else:
                    break
            else:
                break
        i_min = i

        for j in range(j0 + 1, q_count):
            if np.isinf(dist[i0, j]):
                d = dist_func(p_i0, q[j])
                if d < diag_max or j < j_min:
                    dist[i0, j] = d
                else:
                    break
            else:
                break
        j_min = j
    return dist

def _get_corner_min_array(f_mat: np.ndarray, i: int, j: int) -> float:
    if i > 0 and j > 0:
        a = min(f_mat[i - 1, j - 1],
                f_mat[i, j - 1],
                f_mat[i - 1, j])
    elif i == 0 and j == 0:
        a = f_mat[i, j]
    elif i == 0:
        a = f_mat[i, j - 1]
    else:  # j == 0:
        a = f_mat[i - 1, j]
    return a

def _fast_frechet_matrix(dist: np.ndarray,
                         diag: np.ndarray,
                         p: np.ndarray,
                         q: np.ndarray) -> np.ndarray:

    for k in range(diag.shape[0]):
        i0 = diag[k, 0]
        j0 = diag[k, 1]

        for i in range(i0, p.shape[0]):
            if np.isfinite(dist[i, j0]):
                c = _get_corner_min_array(dist, i, j0)
                if c > dist[i, j0]:
                    dist[i, j0] = c
            else:
                break

        # Add 1 to j0 to avoid recalculating the diagonal
        for j in range(j0 + 1, q.shape[0]):
            if np.isfinite(dist[i0, j]):
                c = _get_corner_min_array(dist, i0, j)
                if c > dist[i0, j]:
                    dist[i0, j] = c
            else:
                break
    return dist


def _fdfd_matrix(p: np.ndarray,
                 q: np.ndarray,
                 dist_func: Callable[[np.array, np.array], float]) -> float:
    diagonal = _bresenham_pairs(0, 0, p.shape[0], q.shape[0])
    ca = _fast_distance_matrix(p, q, diagonal, dist_func)
    ca = _fast_frechet_matrix(ca, diagonal, p, q)
    return ca

class FastDiscreteFrechetMatrix(object):

    def __init__(self, dist_func):
        """

        Parameters
        ----------
        dist_func:
        """
        self.times = []
        self.dist_func = dist_func
        self.ca = np.zeros((1, 1))
        # JIT the numba code
        self.distance(np.array([[0.0, 0.0], [1.0, 1.0]]),
                      np.array([[0.0, 0.0], [1.0, 1.0]]))



    def distance(self, p: np.ndarray, q: np.ndarray) -> float:
        ca = _fdfd_matrix(p, q, self.dist_func)
        self.ca = ca
        return ca[p.shape[0]-1, q.shape[0]-1]

def resample_trajectory(traj: np.ndarray, num_points: int) -> np.ndarray:
    """
    Resample a 2D trajectory to have a fixed number of evenly spaced points.

    Parameters
    ----------
    traj : np.ndarray
        Original trajectory as N x 2 array.
    num_points : int
        Number of points for resampling.

    Returns
    -------
    np.ndarray
        Resampled trajectory of shape (num_points, 2).
    """
    from scipy.interpolate import interp1d

    # Remove consecutive duplicate points
    diffs = np.diff(traj, axis=0)
    keep = np.any(diffs != 0, axis=1)
    keep = np.concatenate(([True], keep))  # Always keep the first point
    traj = traj[keep]

    if len(traj) == 1:
        # Static point, no trajectory
        return np.tile(traj[0], (num_points, 1))

    distances = np.cumsum(np.r_[0, np.linalg.norm(np.diff(traj, axis=0), axis=1)])
    total_length = distances[-1]
    if total_length == 0:
        return np.tile(traj[0], (num_points, 1))

    interpolator = interp1d(distances, traj, axis=0, kind='linear')
    new_distances = np.linspace(0, total_length, num=num_points)
    return interpolator(new_distances)


def draw_obstacle(ax, x, y, yaw, width, length, color="red"):
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

"""
The path format is
path_prefix ## SPEED ## ITERATION ## path_suffix
"""
path_prefix = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/E2E/"
path_suffix = ".bag"

#summary dictionary
averages = {}
frechet = FastDiscreteFrechetMatrix(euclidean)
avg_frechet = {}




# Constants for obstacle size
OBS_W = 0.2
OBS_L = 0.6

# Obstacle definitions
OBSTACLE_OVERRIDES = {
    "red": [
        {"x": 42.96153951304803, "y": 58.79113806025087, "yaw": 81.83640216273398},
        {"x": 60.02001188200146, "y": 40.56861047131106, "yaw": 80.253125220876818},
    ],
    "green": [
        {"x": 35.28691769215104, "y": 44.62221548986447, "yaw": 148.46028799484392},
        {"x": 59.33407100338138, "y": 57.759962751282885, "yaw": 143.96145109317095},
    ],
}

#["green"]
for test in ["green","red"]:
    #storage for data in this category
    averages[test] = {}
    real_final = []   
    real_cumulative = [] 
    mixed_final = []
    mixed_cumulative = []
    mixed_entire_final = []
    mixed_entire_cumulative = []
    vil_final = []
    vil_cumulative = []
    sim_final = []
    sim_cumulative = []

    avg_frechet[test] = {}
    frechet_from_sim = []
    frechet_from_mixed_entire = []
    frechet_from_vil = []
    frechet_from_mixed = []

    mean_dist_sim, mean_dist_mixed, mean_dist_mixed_entire, mean_dist_vil = [], [], [], []


    

    for iteration in ["1","2","3","4","5"]:
        fig = plt.figure()
        
        path = path_prefix + "real/" + test  + str(iteration) + path_suffix
        print(f"Path is {path}")
        b = bagreader(path)

        frames = {}
        for t in ["/donkey/pose", "/sim/euler", "/sim/speed"]:
            data = b.message_by_topic(t)
            frames[t] = pd.read_csv(data).set_index("Time")
        
        mixed_path = path_prefix + "mixed_obstacles/" + test  + str(iteration) + path_suffix
        print(f"Path is {mixed_path}")
        b = bagreader(mixed_path)

        mixed_frames = {}
        for t in ["/donkey/pose", "/sim/euler", "/sim/speed"]:
            data = b.message_by_topic(t)
            mixed_frames[t] = pd.read_csv(data).set_index("Time")
        
        mixed_entire_path = path_prefix + "mixed_entire/" + test  + str(iteration) + path_suffix
        print(f"Path is {mixed_entire_path}")
        b = bagreader(mixed_entire_path)

        mixed_entire_frames = {}
        for t in ["/donkey/pose", "/sim/euler", "/sim/speed"]:
            data = b.message_by_topic(t)
            mixed_entire_frames[t] = pd.read_csv(data).set_index("Time")
        
        vil_path = path_prefix + "vil/" + test  + str(iteration) + path_suffix
        print(f"Path is {vil_path}")
        b = bagreader(vil_path)

        vil_frames = {}
        for t in ["/donkey/pose", "/sim/euler", "/sim/speed"]:
            data = b.message_by_topic(t)
            vil_frames[t] = pd.read_csv(data).set_index("Time")
        

        sim_path = path_prefix + "sim/" +test  + str(iteration) + path_suffix
        sim_b = bagreader(sim_path)
        sim_frames = {}
        for t in ("/sim/euler", "/sim/speed"):
            data = sim_b.message_by_topic(t)
            sim_frames[t] = pd.read_csv(data).set_index("Time")
        
        
        sim_frames["/sim/euler"]["x"] = sim_frames["/sim/euler"]["x"].apply(lambda x: (x-X_MAP_SHIFT)/SIZE_FACTOR)
        sim_frames["/sim/euler"]["z"] = sim_frames["/sim/euler"]["z"].apply(lambda x: (x-Y_MAP_SHIFT)/SIZE_FACTOR)
        
        map_path = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/Nominal.json"
        with open(map_path, 'r') as f:
            map_data = json.load(f)
        sim_waypoints_raw = map_data["sim_waypoint_list"]
        wp = (np.array(sim_waypoints_raw) - np.array([X_MAP_SHIFT, Y_MAP_SHIFT])) / SIZE_FACTOR

        try:
            obstacle_path = b.message_by_topic("/obstacles")
            if obstacle_path:
                frames["/obstacles"] = pd.read_csv(obstacle_path).set_index("Time")
            else:
                print(f"No /obstacles topic in: {path}")
                frames["/obstacles"] = None
        except Exception as e:
            print(f"Failed to load /obstacles from {path}: {e}")
            frames["/obstacles"] = None

        
        # PLOT
        fig, ax1 = plt.subplots(figsize=(14.42 / 2.54, 10.16 / 2.54))  # Set size in inches (converted from cm)
        ax1.set_aspect("equal")
        ax1.set_ylim([-2.5, 2.5])
        ax1.set_xlim([-2.5, 2.5])
        ax1.set_ylabel("y [m]", fontsize=11)
        ax1.set_xlabel("x [m]", fontsize=11)
        ax1.set_title(f"Trajectory Comparison – {test} (Iteration {iteration})", fontsize=12, fontweight='bold')

        # Plot lane center and boundaries
        ax1.plot(wp.T[0], wp.T[1], "--", color="black", label="Center Line", linewidth=1.2)
        for margin_label in ["left_margin", "right_margin"]:
            if margin_label in map_data:
                margin = (np.array(map_data[margin_label]) - np.array([X_MAP_SHIFT, Y_MAP_SHIFT])) / SIZE_FACTOR
                ax1.plot(margin[:, 0], margin[:, 1], linestyle='--', linewidth=0.8, color="gray")

        # Plot each trajectory with distinct style
        ax1.plot(frames["/donkey/pose"]["pose.position.x"],
                frames["/donkey/pose"]["pose.position.y"],
                color=COLORS['real'], label="Real", linewidth=2.0)

        ax1.plot(sim_frames["/sim/euler"]["x"],
                sim_frames["/sim/euler"]["z"],
                color=COLORS['sim'], label="Simulation", linewidth=1.2, alpha=0.9)

        ax1.plot(mixed_frames["/donkey/pose"]["pose.position.x"],
                mixed_frames["/donkey/pose"]["pose.position.y"],
                color=COLORS['mixed'], label="Mixed Obstacles", linewidth=1.2, alpha=0.9)

        ax1.plot(mixed_entire_frames["/donkey/pose"]["pose.position.x"],
                mixed_entire_frames["/donkey/pose"]["pose.position.y"],
                color=COLORS['mixed_entire'], label="Mixed Entire", linewidth=1.2, alpha=0.9)

        ax1.plot(vil_frames["/donkey/pose"]["pose.position.x"],
                vil_frames["/donkey/pose"]["pose.position.y"],
                color=COLORS['vil'], label="VIL", linewidth=1.2, alpha=0.9)

        # Plot obstacles
        override_key = test.lower()
        if override_key in OBSTACLE_OVERRIDES:
            for obs in OBSTACLE_OVERRIDES[override_key]:
                x = (obs["x"] - X_MAP_SHIFT) / SIZE_FACTOR
                y = (obs["y"] - Y_MAP_SHIFT) / SIZE_FACTOR
                yaw = np.deg2rad(obs["yaw"] + 45)
                draw_obstacle(ax1, x, y, yaw, width=OBS_W, length=OBS_L)
        elif frames.get("/obstacles") is not None:
            for _, row in frames["/obstacles"].iterrows():
                draw_obstacle(ax1, row["x"], row["y"], np.deg2rad(row["yaw"]), width=OBS_W, length=OBS_L)

        # Plot start and waypoint markers
        ax1.plot(wp.T[0], wp.T[1], 'kx', label="Waypoints", markersize=4, alpha=0.6)

        # Clean legend (remove duplicates and default labels)
        handles, labels = ax1.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        ax1.legend(unique.values(), unique.keys(), loc="best", fontsize=9)

        ax1.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
        plt.tight_layout()

        # Compute mean Euclidean distance across all resampled points

        handles, labels = ax1.get_legend_handles_labels()
        filtered = [(h, l) for h, l in zip(handles, labels) if l and not l.startswith('_')]
        if filtered:
            ax1.legend(*zip(*filtered))

        plt.gcf().set_size_inches(14.42/2.54, 10.16/2.54)
        plt.savefig(f"PROCESSED RESULTS/RQ3/E2E/plots/{test}_{iteration}.pdf")
        
        plt.close()



        #calculate distance between stopping position and final waypoint
        #calculate general closeness to all waypoints: the sum of the distance from each waypoint to the closest point in trajectory
        real_points = frames["/donkey/pose"][["pose.position.x","pose.position.y"]].to_numpy()
        real_tree = cKDTree(real_points)
        real_distances, _ = real_tree.query(wp)
        real_cumulative.append(np.sum(real_distances))
        real_final.append(math.dist(wp[-1], real_points[-1]))

        mixed_entire_points = mixed_entire_frames["/donkey/pose"][["pose.position.x","pose.position.y"]].to_numpy()
        mixed_entire_tree = cKDTree(mixed_entire_points)
        mixed_entire_distances, _ = mixed_entire_tree.query(wp)
        mixed_entire_cumulative.append(np.sum(mixed_entire_distances))
        mixed_entire_final.append(math.dist(wp[-1], mixed_entire_points[-1]))

        vil_points = vil_frames["/donkey/pose"][["pose.position.x","pose.position.y"]].to_numpy()
        vil_tree = cKDTree(vil_points)
        vil_distances, _ = vil_tree.query(wp)
        vil_cumulative.append(np.sum(vil_distances))
        vil_final.append(math.dist(wp[-1], vil_points[-1]))

        mixed_points = mixed_frames["/donkey/pose"][["pose.position.x","pose.position.y"]].to_numpy()
        mixed_tree = cKDTree(mixed_points)
        mixed_distances, _ = mixed_tree.query(wp)
        mixed_cumulative.append(np.sum(mixed_distances))
        mixed_final.append(math.dist(wp[-1], mixed_points[-1]))

        sim_points = sim_frames["/sim/euler"][["x","z"]].to_numpy()
        sim_tree = cKDTree(sim_points)
        sim_distances, _ = sim_tree.query(wp)
        sim_cumulative.append(np.sum(sim_distances))
        sim_final.append(math.dist(wp[-1], sim_points[-1]))

        N = 1000  # or any consistent value depending on desired resolution
        real_points = resample_trajectory(real_points, N)
        sim_points = resample_trajectory(sim_points, N)
        mixed_points = resample_trajectory(mixed_points, N)
        mixed_entire_points = resample_trajectory(mixed_entire_points, N)
        vil_points = resample_trajectory(vil_points, N)
        frechet_from_mixed.append(frechet.distance(real_points, mixed_points))
        frechet_from_mixed_entire.append(frechet.distance(real_points, mixed_entire_points))
        frechet_from_vil.append(frechet.distance(real_points, vil_points))
        frechet_from_sim.append(frechet.distance(real_points, sim_points))

        # Compute mean Euclidean distance across all resampled points
        def mean_traj_distance(p1, p2):
            return np.mean(np.linalg.norm(p1 - p2, axis=1))

        mean_dist_sim.append(mean_traj_distance(real_points, sim_points))
        mean_dist_mixed.append(mean_traj_distance(real_points, mixed_points))
        mean_dist_mixed_entire.append(mean_traj_distance(real_points, mixed_entire_points))
        mean_dist_vil.append(mean_traj_distance(real_points, vil_points))

        


    #summarize data for this test category
    averages[test]["real"] = {}

    
    averages[test]["real"]["stopping distance to final waypoint"] = (np.mean(real_final), np.std(real_final))
    averages[test]["real"]["cummulative distance to waypoints"] = (np.mean(real_cumulative), np.std(real_cumulative))

    averages[test]["mixed"] = {}
    averages[test]["mixed"]["stopping distance to final waypoint"] = (np.mean(mixed_final), np.std(mixed_final))
    averages[test]["mixed"]["cummulative distance to waypoints"] = (np.mean(mixed_cumulative), np.std(mixed_cumulative))

    averages[test]["mixed_entire"] = {}
    averages[test]["mixed_entire"]["stopping distance to final waypoint"] = (np.mean(mixed_entire_final), np.std(mixed_entire_final))
    averages[test]["mixed_entire"]["cummulative distance to waypoints"] = (np.mean(mixed_entire_cumulative), np.std(mixed_entire_cumulative))

    averages[test]["vil"] = {}
    averages[test]["vil"]["stopping distance to final waypoint"] = (np.mean(vil_final), np.std(vil_final))
    averages[test]["vil"]["cummulative distance to waypoints"] = (np.mean(vil_cumulative), np.std(vil_cumulative))


    averages[test]["sim"] = {}
    averages[test]["sim"]["stopping distance to final waypoint"] = (np.mean(sim_final), np.std(sim_final))
    averages[test]["sim"]["cummulative distance to waypoints"] = (np.mean(sim_cumulative), np.std(sim_cumulative))

    avg_frechet[test]["sim to real"] = (np.mean(frechet_from_sim), np.std(frechet_from_sim))
    avg_frechet[test]["mixed to real"] = (np.mean(frechet_from_mixed), np.std(frechet_from_mixed))
    avg_frechet[test]["mixed_entire to real"] = (np.mean(frechet_from_mixed_entire), np.std(frechet_from_mixed_entire))
    avg_frechet[test]["vil to real"] = (np.mean(frechet_from_vil), np.std(frechet_from_vil))

    avg_frechet[test]["sim_to_real_raw"] = frechet_from_sim
    avg_frechet[test]["mixed_to_real_raw"] = frechet_from_mixed
    avg_frechet[test]["mixed_entire_to_real_raw"] = frechet_from_mixed_entire
    avg_frechet[test]["vil_to_real_raw"] = frechet_from_vil

    avg_frechet[test]["sim_to_real_mean_dist"] = mean_dist_sim
    avg_frechet[test]["mixed_to_real_mean_dist"] = mean_dist_mixed
    avg_frechet[test]["mixed_entire_to_real_mean_dist"] = mean_dist_mixed_entire
    avg_frechet[test]["vil_to_real_mean_dist"] = mean_dist_vil

    # Save raw lists for statistical tests later
    averages[test]["real"]["stopping distance to final waypoint_raw"] = real_final
    averages[test]["real"]["cummulative distance to waypoints_raw"] = real_cumulative

    averages[test]["mixed"]["stopping distance to final waypoint_raw"] = mixed_final
    averages[test]["mixed"]["cummulative distance to waypoints_raw"] = mixed_cumulative

    averages[test]["mixed_entire"]["stopping distance to final waypoint_raw"] = mixed_entire_final
    averages[test]["mixed_entire"]["cummulative distance to waypoints_raw"] = mixed_entire_cumulative

    averages[test]["vil"]["stopping distance to final waypoint_raw"] = vil_final
    averages[test]["vil"]["cummulative distance to waypoints_raw"] = vil_cumulative

    averages[test]["sim"]["stopping distance to final waypoint_raw"] = sim_final
    averages[test]["sim"]["cummulative distance to waypoints_raw"] = sim_cumulative

    avg_frechet[test]["mixed_to_real_raw"] = frechet_from_mixed
    avg_frechet[test]["mixed_entire_to_real_raw"] = frechet_from_mixed_entire
    avg_frechet[test]["vil_to_real_raw"] = frechet_from_vil
    avg_frechet[test]["sim_to_real_raw"] = frechet_from_sim



print("PRINTING SUMMARY")

import json
from pathlib import Path
from scipy.stats import ttest_ind
import numpy as np
import json
from pathlib import Path


def cohen_d(x, y):
    """Compute Cohen's d for two independent samples."""
    nx = len(x)
    ny = len(y)
    dof = nx + ny - 2
    pooled_std = np.sqrt(((nx - 1) * np.std(x, ddof=1)**2 + (ny - 1) * np.std(y, ddof=1)**2) / dof)
    return (np.mean(x) - np.mean(y)) / pooled_std if pooled_std != 0 else 0



def ttest_and_cohen(x, y):
    x = np.array(x)
    y = np.array(y)
    mask = ~np.isnan(x) & ~np.isnan(y)
    if len(x[mask]) < 2 or len(y[mask]) < 2:
        return np.nan, np.nan
    p = ttest_ind(x[mask], y[mask], equal_var=False).pvalue
    d = cohen_d(x[mask], y[mask])
    return p, d

stat_results = {}

for test in avg_frechet:
    stat_results[test] = {}

    for method in ["sim", "mixed", "mixed_entire", "vil"]:
        # Frechet
        p_f, d_f = ttest_and_cohen(
            avg_frechet[test][f"{method}_to_real_raw"],
            [0]*len(avg_frechet[test][f"{method}_to_real_raw"])  # real vs. itself = 0
        )
        # Mean trajectory distance
        p_e, d_e = ttest_and_cohen(
            avg_frechet[test][f"{method}_to_real_mean_dist"],
            [0]*len(avg_frechet[test][f"{method}_to_real_mean_dist"])
        )

        stat_results[test][method] = {
            "frechet_vs_real": {
                "p_value": p_f,
                "cohen_d": d_f,
                "mean": float(np.mean(avg_frechet[test][f"{method}_to_real_raw"])),
                "std": float(np.std(avg_frechet[test][f"{method}_to_real_raw"]))
            },
            "mean_euclidean_vs_real": {
                "p_value": p_e,
                "cohen_d": d_e,
                "mean": float(np.mean(avg_frechet[test][f"{method}_to_real_mean_dist"])),
                "std": float(np.std(avg_frechet[test][f"{method}_to_real_mean_dist"]))
            }
        }

save_path = Path("PROCESSED RESULTS/RQ3/E2E/aggregated_summary.json")
save_path.parent.mkdir(parents=True, exist_ok=True)
with open(save_path, 'w') as f:
    json.dump(stat_results, f, indent=2)
print(f"Saved trajectory analysis results to: {save_path}")





from shapely.geometry import LineString, Point
import matplotlib.path as mpath

OBSTACLE_THRESHOLD = 0.8
cte_results = {}

# Generate hybrid trajectory statistics for each test
for test in averages:
    cte_results[test] = {}

    # Load map margins
    map_path = "/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/Nominal.json"
    with open(map_path, 'r') as f:
        map_data = json.load(f)

    left_margin = np.array(map_data["left_margin"])
    right_margin = np.array(map_data["right_margin"])

    # Apply map shift and scale
    left_margin = (left_margin - np.array([X_MAP_SHIFT, Y_MAP_SHIFT])) / SIZE_FACTOR
    right_margin = (right_margin - np.array([X_MAP_SHIFT, Y_MAP_SHIFT])) / SIZE_FACTOR

    # Create road boundary polygon
    left_pts = left_margin
    right_pts = right_margin[::-1]
    verts = np.vstack([left_pts, right_pts])
    road_boundary = mpath.Path(verts)

    path_line = LineString((np.array(map_data["sim_waypoint_list"]) - np.array([X_MAP_SHIFT, Y_MAP_SHIFT])) / SIZE_FACTOR)

    obstacle_points = np.array([
        [(obs["x"] - X_MAP_SHIFT) / SIZE_FACTOR, (obs["y"] - Y_MAP_SHIFT) / SIZE_FACTOR]
        for obs in OBSTACLE_OVERRIDES[test]
    ])
    obstacle_tree = cKDTree(obstacle_points)

    for traj_type, points in zip(
        ["real", "sim","mixed","mixed_entire","vil"],
        [resample_trajectory(np.array(frames["/donkey/pose"][["pose.position.x", "pose.position.y"]]), 500),
         resample_trajectory(np.array(sim_frames["/sim/euler"][["x", "z"]]), 500),
         resample_trajectory(np.array(mixed_frames["/donkey/pose"][["pose.position.x", "pose.position.y"]]), 500),
         resample_trajectory(np.array(mixed_entire_frames["/donkey/pose"][["pose.position.x", "pose.position.y"]]), 500),
         resample_trajectory(np.array(vil_frames["/donkey/pose"][["pose.position.x", "pose.position.y"]]), 500)
         ]
    ):
        cte_values = []
        obstacle_dists = []
        offroad_count = 0

        # Completion rate logic: discretize the road path into 100 points and check if each is reached
        num_segments = 100
        discretized_path = np.linspace(0, path_line.length, num_segments)
        path_points = [path_line.interpolate(dist) for dist in discretized_path]
        reached = 0
        for seg_pt in path_points:
            seg_xy = np.array([seg_pt.x, seg_pt.y])
            # Find the closest trajectory point
            dists = np.linalg.norm(points - seg_xy, axis=1)
            min_dist = np.min(dists)
            closest_idx = np.argmin(dists)
            # Check if within 0.5 m and inside road boundary
            if min_dist <= 0.5 and road_boundary.contains_point(points[closest_idx]):
                reached += 1
            else:
                break  # Stop at first missed segment
        completion_rate = reached / num_segments

        for pt in points:
            dist_to_obs, _ = obstacle_tree.query(pt)
            if dist_to_obs < OBSTACLE_THRESHOLD:
                obstacle_dists.append(dist_to_obs)
            else:
                cte_values.append(Point(pt).distance(path_line))
                if road_boundary.contains_point(pt):
                    pass
                else:
                    offroad_count += 1

        cte_results[test][traj_type] = {
            "cte_mean": np.mean(cte_values) if cte_values else None,
            "cte_std": np.std(cte_values) if cte_values else None,
            "obstacle_min_dist": np.min(obstacle_dists) if obstacle_dists else None,
            "obstacle_mean_dist": np.mean(obstacle_dists) if obstacle_dists else None,
            "offroad_ratio": offroad_count / len(points),
            "completion_rate": completion_rate
        }

# Save hybrid CTE summary
cte_path = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ4/E2E/hybrid_trajectory_summary.json")
cte_path.parent.mkdir(parents=True, exist_ok=True)
with open(cte_path, 'w') as f:
    json.dump(cte_results, f, indent=2)

print(f"Saved hybrid trajectory summary to: {cte_path}")