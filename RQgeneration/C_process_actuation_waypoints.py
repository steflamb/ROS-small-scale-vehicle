# Pure Pursuit Analysis Script (Updated: Frechet removed, Wasserstein + pointwise + plotting)

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
import re
from scipy.spatial import cKDTree
from typing import Callable
from pathlib import Path
from scipy.stats import ttest_ind, wasserstein_distance

SIZE_FACTOR = 7.33
X_MAP_SHIFT = 48
Y_MAP_SHIFT = 50

COLORS = {'real': '#2B83BA', 'mapped': '#FDAE61', 'sim': '#ABDDA4'}


def resample_trajectory(traj: np.ndarray, num_points: int = 100) -> np.ndarray:
    from scipy.interpolate import interp1d
    diffs = np.diff(traj, axis=0)
    keep = np.any(diffs != 0, axis=1)
    keep = np.concatenate(([True], keep))
    traj = traj[keep]
    if len(traj) == 1:
        return np.tile(traj[0], (num_points, 1))
    distances = np.cumsum(np.r_[0, np.linalg.norm(np.diff(traj, axis=0), axis=1)])
    total_length = distances[-1]
    if total_length == 0:
        return np.tile(traj[0], (num_points, 1))
    interpolator = interp1d(distances, traj, axis=0, kind='linear')
    new_distances = np.linspace(0, total_length, num=num_points)
    return interpolator(new_distances)


def compute_pvalue_and_cohen(real: np.ndarray, other: np.ndarray):
    mask = ~np.isnan(real) & ~np.isnan(other)
    real = real[mask]
    other = other[mask]
    if len(real) < 2 or len(other) < 2:
        return np.nan, np.nan
    _, p = ttest_ind(real, other, equal_var=False)
    pooled_std = np.sqrt((np.var(real, ddof=1) + np.var(other, ddof=1)) / 2)
    d = (np.mean(real) - np.mean(other)) / pooled_std if pooled_std > 0 else 0
    return float(p), float(d)

# Initialize output containers
output = {}
stat_results = {"trajectory_similarity": {}}

# Loop over test types
for test in ["straight", "close", "far", "sharp", "curve", "stest"]:
    real_traj = []
    sim_traj = []
    mapped_traj = []
    wass_sim = []
    wass_mapped = []
    pointwise_sim = []
    pointwise_mapped = []

    for iteration in range(1, 6):
        # Load bag, extract data, align time, adjust coordinates (omitted here)
        # Example placeholder arrays:
        real_points = np.random.rand(200, 2)
        sim_points = real_points + np.random.normal(0, 0.05, size=real_points.shape)
        mapped_points = real_points + np.random.normal(0, 0.1, size=real_points.shape)

        # Resample trajectories
        real_resampled = resample_trajectory(real_points)
        sim_resampled = resample_trajectory(sim_points)
        mapped_resampled = resample_trajectory(mapped_points)

        # Store flattened
        real_traj.extend(real_resampled.flatten())
        sim_traj.extend(sim_resampled.flatten())
        mapped_traj.extend(mapped_resampled.flatten())

        # Wasserstein distances
        wass_sim.append(wasserstein_distance(real_resampled.flatten(), sim_resampled.flatten()))
        wass_mapped.append(wasserstein_distance(real_resampled.flatten(), mapped_resampled.flatten()))

        # Pointwise Euclidean distances
        pointwise_sim.append(np.mean(np.linalg.norm(real_resampled - sim_resampled, axis=1)))
        pointwise_mapped.append(np.mean(np.linalg.norm(real_resampled - mapped_resampled, axis=1)))

        # Plotting
        plt.figure()
        plt.plot(real_resampled[:, 0], real_resampled[:, 1], label='Real', color=COLORS['real'])
        plt.plot(sim_resampled[:, 0], sim_resampled[:, 1], label='Simulation', color=COLORS['sim'])
        plt.plot(mapped_resampled[:, 0], mapped_resampled[:, 1], label='Mapped', color=COLORS['mapped'])
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(f"{test} - Iteration {iteration}")
        Path("plots").mkdir(exist_ok=True)
        plt.savefig(f"plots/{test}_{iteration}.pdf")
        plt.close()

    # Aggregation
    output[test] = {
        "wass_dist_sim_vs_real": {
            "mean": np.mean(wass_sim),
            "std": np.std(wass_sim)
        },
        "wass_dist_mapped_vs_real": {
            "mean": np.mean(wass_mapped),
            "std": np.std(wass_mapped)
        },
        "pointwise_dist_sim_vs_real": {
            "mean": np.mean(pointwise_sim),
            "std": np.std(pointwise_sim)
        },
        "pointwise_dist_mapped_vs_real": {
            "mean": np.mean(pointwise_mapped),
            "std": np.std(pointwise_mapped)
        }
    }

    # Statistical tests
    p_sim, d_sim = compute_pvalue_and_cohen(np.array(real_traj), np.array(sim_traj))
    p_map, d_map = compute_pvalue_and_cohen(np.array(real_traj), np.array(mapped_traj))

    stat_results["trajectory_similarity"][test] = {
        "p_value_wass_sim_vs_real": p_sim,
        "d_value_wass_sim_vs_real": d_sim,
        "p_value_wass_mapped_vs_real": p_map,
        "d_value_wass_mapped_vs_real": d_map
    }
import json
# Save JSON
final_data = {
    "aggregated_summary": output,
    "trajectory_statistical_significance": stat_results["trajectory_similarity"]
}
Path("plots").mkdir(exist_ok=True)
with open("aggregated_summary.json", 'w') as f:
    json.dump(final_data, f, indent=2)

print("Saved summary with Wasserstein, pointwise distances, and statistical tests.")
