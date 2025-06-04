import json
import pandas as pd
from pathlib import Path

import json
import pandas as pd
from pathlib import Path




def generate_rq21_throttle_summary():
    folder = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/Throttle")
    summary_path = folder / "aggregated_summary.json"

    with open(summary_path) as f:
        data = json.load(f)

    for case_id, metrics in data.items():
        if case_id == "global_stats":
            continue

        throttle_label = {
            "34": "Throttle = 0.34",
            "365": "Throttle = 0.365",
            "39": "Throttle = 0.39"
        }.get(case_id, case_id)

        rows = []

        # Distance Δ from Real
        sim_mean = metrics["dist_sim"]["mean"] - metrics["dist_real"]["mean"]
        sim_std = metrics["dist_sim"]["std"]
        mapped_mean = metrics["dist_mapped"]["mean"] - metrics["dist_real"]["mean"]
        mapped_std = metrics["dist_mapped"]["std"]
        rows.append({
            "Metric": "Distance (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Avg Speed Δ from Real
        sim_mean = metrics["avg_speed_sim"]["mean"] - metrics["avg_speed_real"]["mean"]
        sim_std = metrics["avg_speed_sim"]["std"]
        mapped_mean = metrics["avg_speed_mapped"]["mean"] - metrics["avg_speed_real"]["mean"]
        mapped_std = metrics["avg_speed_mapped"]["std"]
        rows.append({
            "Metric": "Average Speed (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Trajectory Diff (Pointwise)
        sim_mean = metrics["traj_diff_sim_vs_real"]["mean"]
        sim_std = metrics["traj_diff_sim_vs_real"]["std"]
        mapped_mean = metrics["traj_diff_mapped_vs_real"]["mean"]
        mapped_std = metrics["traj_diff_mapped_vs_real"]["std"]
        rows.append({
            "Metric": "Trajectory Diff (Pointwise)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Trajectory Diff (Wasserstein)
        sim_mean = metrics["wass_sim_vs_real"]["mean"]
        sim_std = metrics["wass_sim_vs_real"]["std"]
        mapped_mean = metrics["wass_mapped_vs_real"]["mean"]
        mapped_std = metrics["wass_mapped_vs_real"]["std"]
        rows.append({
            "Metric": "Trajectory Diff (Wasserstein)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Add p/d value for Trajectory
        sim_p = metrics["p_value_traj_sim_vs_real"]["mean"]
        sim_d = metrics["d_value_traj_sim_vs_real"]["mean"]
        mapped_p = metrics["p_value_traj_mapped_vs_real"]["mean"]
        mapped_d = metrics["d_value_traj_mapped_vs_real"]["mean"]
        rows.append({
            "Metric": "p/d (Trajectory vs Real)",
            "Sim": f"p={sim_p:.2e}, d={sim_d:.2f}",
            "Mapped": f"p={mapped_p:.2e}, d={mapped_d:.2f}",
        })

        # Add p/d value for Speed
        sim_p = metrics["p_value_speed_sim_vs_real"]["mean"]
        sim_d = metrics["d_value_speed_sim_vs_real"]["mean"]
        mapped_p = metrics["p_value_speed_mapped_vs_real"]["mean"]
        mapped_d = metrics["d_value_speed_mapped_vs_real"]["mean"]
        rows.append({
            "Metric": "p/d (Speed vs Real)",
            "Sim": f"p={sim_p:.2e}, d={sim_d:.2f}",
            "Mapped": f"p={mapped_p:.2e}, d={mapped_d:.2f}",
        })

        df = pd.DataFrame(rows)
        print(f"\n=== RQ2.1 THROTTLE RESULTS FOR {throttle_label} ===")
        print(df.to_latex(index=False, escape=False, column_format='lll'))

generate_rq21_throttle_summary()


# \begin{tabular}{llrrrrrrrr}
# \toprule
# ID & Metric & Sim & Mapped & Real & p (Sim vs Real) & d (Sim vs Real) & p (Mapped vs Real) & d (Mapped vs Real) \\
# \midrule
# 34 & avg_speed & 0.169 & 0.380 & 0.380 & 7.295e-31 & 0.83 & 7.303e-01 & 0.02 \\
# 34 & dist & 3.452 & 1.867 & 1.872 & 1.127e-04 & -0.43 & 9.970e-01 & -0.00 \\
# 365 & avg_speed & 0.184 & 0.546 & 0.754 & 3.704e-09 & 1.05 & 6.798e-01 & 0.06 \\
# 365 & dist & 3.760 & 2.775 & 2.781 & 7.212e-02 & -0.21 & 9.989e-01 & -0.00 \\
# 39 & avg_speed & 0.199 & 0.700 & 0.701 & 1.033e-93 & 1.35 & 6.782e-01 & 0.02 \\
# 39 & dist & 4.022 & 3.954 & 3.961 & 6.486e-01 & -0.03 & 9.983e-01 & 0.00 \\
# \bottomrule
# \end{tabular}

import json
import pandas as pd
from pathlib import Path

def generate_rq21_steering_summary():
    folder = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/Steering")
    summary_path = folder / "steering_summary.json"

    with open(summary_path) as f:
        data = json.load(f)

    for steering_value, metrics in data.items():
        if steering_value == "global_stats":
            continue

        label = f"Steering = {steering_value}"
        rows = []

        # Radius (Δ from Real)
        sim_mean = metrics["mean_radius_sim"] - metrics["mean_radius_real"]
        sim_std = metrics["std_radius_sim"]
        mapped_mean = metrics["mean_radius_mapped"] - metrics["mean_radius_real"]
        mapped_std = metrics["std_radius_mapped"]
        rows.append({
            "Metric": "Radius (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Length (Δ from Real)
        sim_mean = metrics["mean_length_sim"] - metrics["mean_length_real"]
        sim_std = metrics["std_length_sim"]
        mapped_mean = metrics["mean_length_mapped"] - metrics["mean_length_real"]
        mapped_std = metrics["std_length_mapped"]
        rows.append({
            "Metric": "Length (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Trajectory Diff (Pointwise)
        sim_mean = metrics["mean_trajectory_diff_sim_vs_real"]
        sim_std = metrics["std_trajectory_diff_sim_vs_real"]
        mapped_mean = metrics["mean_trajectory_diff_mapped_vs_real"]
        mapped_std = metrics["std_trajectory_diff_mapped_vs_real"]
        rows.append({
            "Metric": "Trajectory Diff (Pointwise)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Wasserstein Distance
        sim_mean = metrics["mean_wass_sim_vs_real"]
        sim_std = metrics["std_wass_sim_vs_real"]
        mapped_mean = metrics["mean_wass_mapped_vs_real"]
        mapped_std = metrics["std_wass_mapped_vs_real"]
        rows.append({
            "Metric": "Trajectory Diff (Wasserstein)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # p/d for Trajectory
        sim_p = metrics["mean_p_value_traj_sim_vs_real"]
        sim_d = metrics["mean_d_value_traj_sim_vs_real"]
        mapped_p = metrics["mean_p_value_traj_mapped_vs_real"]
        mapped_d = metrics["mean_d_value_traj_mapped_vs_real"]
        rows.append({
            "Metric": "p/d (Trajectory vs Real)",
            "Sim": f"p={sim_p:.2e}, d={sim_d:.2f}",
            "Mapped": f"p={mapped_p:.2e}, d={mapped_d:.2f}",
        })

        df = pd.DataFrame(rows)
        print(f"\n=== RQ2.1 STEERING RESULTS FOR {label} ===")
        print(df.to_latex(index=False, escape=False, column_format='lll'))

generate_rq21_steering_summary()

# \begin{tabular}{llrrrrrrrr}
# \toprule
# Steering & Metric & Sim & Mapped & Real & p (Sim vs Real) & d (Sim vs Real) & p (Mapped vs Real) & d (Mapped vs Real) \\
# \midrule
# 03 & length & 6.452 & 6.794 & 6.984 &  &  &  &  \\
# 03 & radius & 1.613 & 1.737 & 1.735 &  &  &  &  \\
# 03 & traj_diff & 0.172 & 0.014 &  & 6.412e-01 & 0.03 & 9.346e-01 & 0.01 \\
# 03 & wass & 0.180 & 0.020 &  &  &  &  &  \\
# 06 & length & 5.847 & 6.675 & 6.685 &  &  &  &  \\
# 06 & radius & 0.815 & 0.970 & 0.970 &  &  &  &  \\
# 06 & traj_diff & 0.109 & 0.012 &  & 3.250e-01 & -0.10 & 9.989e-01 & 0.00 \\
# 06 & wass & 0.114 & 0.001 &  &  &  &  &  \\
# 1 & length & 3.879 & 6.319 & 6.329 &  &  &  &  \\
# 1 & radius & 0.507 & 0.707 & 0.708 &  &  &  &  \\
# 1 & traj_diff & 0.185 & 0.011 &  & 2.526e-01 & -0.12 & 9.983e-01 & 0.00 \\
# 1 & wass & 0.175 & 0.001 &  &  &  &  &  \\
# m03 & length & 6.915 & 6.288 & 6.327 &  &  &  &  \\
# m03 & radius & 1.615 & 3.326 & 3.323 &  &  &  &  \\
# m03 & traj_diff & 1.581 & 0.011 &  & 2.550e-07 & 0.59 & 9.835e-01 & 0.00 \\
# m03 & wass & 0.880 & 0.005 &  &  &  &  &  \\
# m06 & length & 5.737 & 6.245 & 6.254 &  &  &  &  \\
# m06 & radius & 0.816 & 1.313 & 1.313 &  &  &  &  \\
# m06 & traj_diff & 0.552 & 0.012 &  & 2.709e-07 & 0.56 & 9.960e-01 & -0.00 \\
# m06 & wass & 0.624 & 0.001 &  &  &  &  &  \\
# m1 & length & 3.859 & 5.774 & 5.787 &  &  &  &  \\
# m1 & radius & 0.509 & 0.747 & 0.747 &  &  &  &  \\
# m1 & traj_diff & 0.231 & 0.010 &  & 3.615e-02 & 0.37 & 9.969e-01 & -0.00 \\
# m1 & wass & 0.250 & 0.001 &  &  &  &  &  \\
# \bottomrule
# \end{tabular}

import json
import pandas as pd
from pathlib import Path

def generate_rq21_braking_summary():
    folder = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/Braking")
    summary_path = folder / "aggregated_summary.json"

    with open(summary_path) as f:
        data = json.load(f)

    for throttle_val, metrics in data.items():
        if throttle_val == "global_stats":
            continue

        label = f"Throttle = {throttle_val}"
        rows = []

        # Braking (Δ from Real)
        sim_mean = metrics["braking_sim"]["mean"] - metrics["braking_real"]["mean"]
        sim_std = metrics["braking_sim"]["std"]
        mapped_mean = metrics["braking_mapped"]["mean"] - metrics["braking_real"]["mean"]
        mapped_std = metrics["braking_mapped"]["std"]
        rows.append({
            "Metric": "Braking (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Average Speed (Δ from Real)
        sim_mean = metrics["avg_speed_sim"]["mean"] - metrics["avg_speed_real"]["mean"]
        sim_std = metrics["avg_speed_sim"]["std"]
        mapped_mean = metrics["avg_speed_mapped"]["mean"] - metrics["avg_speed_real"]["mean"]
        mapped_std = metrics["avg_speed_mapped"]["std"]
        rows.append({
            "Metric": "Average Speed (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Average Acceleration (Δ from Real)
        sim_mean = metrics["avg_accel_sim"]["mean"] - metrics["avg_accel_real"]["mean"]
        sim_std = metrics["avg_accel_sim"]["std"]
        mapped_mean = metrics["avg_accel_mapped"]["mean"] - metrics["avg_accel_real"]["mean"]
        mapped_std = metrics["avg_accel_mapped"]["std"]
        rows.append({
            "Metric": "Average Accel. (Δ from Real)",
            "Sim": f"{sim_mean:.6f} ± {sim_std:.6f}",
            "Mapped": f"{mapped_mean:.6f} ± {mapped_std:.6f}",
        })

        # Pull global p/d values
        global_stats = data["global_stats"]

        # p/d Speed
        sim_p = global_stats["p_speed_sim_vs_real"]
        sim_d = global_stats["d_speed_sim_vs_real"]
        mapped_p = global_stats["p_speed_mapped_vs_real"]
        mapped_d = global_stats["d_speed_mapped_vs_real"]
        rows.append({
            "Metric": "p/d (Speed vs Real)",
            "Sim": f"p={sim_p:.2e}, d={sim_d:.2f}",
            "Mapped": f"p={mapped_p:.2e}, d={mapped_d:.2f}",
        })

        # p/d Acceleration
        sim_p = global_stats["p_accel_sim_vs_real"]
        sim_d = global_stats["d_accel_sim_vs_real"]
        mapped_p = global_stats["p_accel_mapped_vs_real"]
        mapped_d = global_stats["d_accel_mapped_vs_real"]
        rows.append({
            "Metric": "p/d (Accel vs Real)",
            "Sim": f"p={sim_p:.2e}, d={sim_d:.2f}",
            "Mapped": f"p={mapped_p:.2e}, d={mapped_d:.2f}",
        })

        df = pd.DataFrame(rows)
        print(f"\n=== RQ2.1 BRAKING RESULTS FOR {label} ===")
        print(df.to_latex(index=False, escape=False, column_format='lll'))

generate_rq21_braking_summary()

# === RQ2.1 BRAKING RESULTS (LaTeX) ===
# \begin{tabular}{llrrrrrrrr}
# \toprule
# Braking & Metric & Sim & Mapped & Real & p (Sim vs Real) & d (Sim vs Real) & p (Mapped vs Real) & d (Mapped vs Real) \\
# \midrule
# 0.34 & avg_accel & 0.008 & -0.001 & -0.000 & 2.165e-35 & -0.23 & 5.157e-01 & -0.01 \\
# 0.34 & avg_speed & 0.278 & 0.353 & 0.371 & 1.488e-161 & 0.51 & 3.575e-03 & 0.05 \\
# 0.34 & braking & 0.133 & 0.086 & 0.065 &  &  &  &  \\
# 0.365 & avg_accel & 0.009 & 0.000 & -0.001 & 2.165e-35 & -0.23 & 5.157e-01 & -0.01 \\
# 0.365 & avg_speed & 0.290 & 0.410 & 0.425 & 1.488e-161 & 0.51 & 3.575e-03 & 0.05 \\
# 0.365 & braking & 0.175 & 0.292 & 0.225 &  &  &  &  \\
# 0.39 & avg_accel & 0.008 & 0.002 & 0.001 & 2.165e-35 & -0.23 & 5.157e-01 & -0.01 \\
# 0.39 & avg_speed & 0.294 & 0.482 & 0.503 & 1.488e-161 & 0.51 & 3.575e-03 & 0.05 \\
# 0.39 & braking & 0.185 & 0.337 & 0.274 &  &  &  &  \\
# \bottomrule
# \end{tabular}

def generate_rq21_speed_regulation_summary():
    folder = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/Pid")
    summary_path = folder / "aggregated_summary.json"

    with open(summary_path) as f:
        data = json.load(f)

    rows = []

    # Average Speed Error
    sim_mean = data["sim"]["Average speed error"]["mean"] - data["real"]["Average speed error"]["mean"]
    sim_std = data["sim"]["Average speed error"]["std"]
    mapped_mean = data["mapped"]["Average speed error"]["mean"] - data["real"]["Average speed error"]["mean"]
    mapped_std = data["mapped"]["Average speed error"]["std"]
    rows.append({
        "Metric": "Average Speed Error (Δ from Real)",
        "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
        "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
    })

    # Target Speed Errors
    for target in ["0.0", "0.4", "0.6", "0.8"]:
        sim_mean = data["sim"]["Target speed error"][target]["mean"] - data["real"]["Target speed error"][target]["mean"]
        sim_std = data["sim"]["Target speed error"][target]["std"]
        mapped_mean = data["mapped"]["Target speed error"][target]["mean"] - data["real"]["Target speed error"][target]["mean"]
        mapped_std = data["mapped"]["Target speed error"][target]["std"]
        rows.append({
            "Metric": f"Target Speed Error {target} (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

    # p/d (Speed vs Real)
    sim_p = data["raw_speed_vs_real"]["p_value_sim_vs_real"]
    sim_d = data["raw_speed_vs_real"]["d_value_sim_vs_real"]
    mapped_p = data["raw_speed_vs_real"]["p_value_mapped_vs_real"]
    mapped_d = data["raw_speed_vs_real"]["d_value_mapped_vs_real"]
    rows.append({
        "Metric": "p/d (Speed vs Real)",
        "Sim": f"p={sim_p:.2e}, d={sim_d:.2f}",
        "Mapped": f"p={mapped_p:.2e}, d={mapped_d:.2f}",
    })

    df = pd.DataFrame(rows)
    print("\n=== RQ2.1 CLOSED-LOOP SPEED REGULATION RESULTS ===")
    print(df.to_latex(index=False, escape=False, column_format='lll'))

generate_rq21_speed_regulation_summary()

# === RQ2.1 PID RESULTS (LaTeX) ===
# \begin{tabular}{llrrrrrrrr}
# \toprule
# Target & Metric & Sim & Mapped & Real & p (Sim vs Real) & d (Sim vs Real) & p (Mapped vs Real) & d (Mapped vs Real) \\
# \midrule
# 0.0 & Target Speed Error & 0.383 & 0.304 & 0.235 & 3.097e-33 & -0.26 & 2.000e-13 & -0.16 \\
# 0.4 & Target Speed Error & 0.399 & 0.329 & 0.387 & 3.097e-33 & -0.26 & 2.000e-13 & -0.16 \\
# 0.6 & Target Speed Error & 0.097 & 0.170 & 0.142 & 3.097e-33 & -0.26 & 2.000e-13 & -0.16 \\
# 0.8 & Target Speed Error & 0.456 & 0.453 & 0.456 & 3.097e-33 & -0.26 & 2.000e-13 & -0.16 \\
# avg_error & Avg Speed Error & 0.342 & 0.308 & 0.267 & 6.959e-44 & -0.30 & 6.850e-15 & -0.17 \\
# \bottomrule
# \end{tabular}

import json
import pandas as pd
from pathlib import Path

def generate_rq21_trajectory_tracking_summary():
    folder = Path("/media/ast/1868aaf7-73b4-4e22-9fd8-ff4f3eb47346/ASE/PROCESSED RESULTS/RQ2/PurePursuit")
    summary_path = folder / "aggregated_summary.json"

    with open(summary_path) as f:
        data = json.load(f)

    aggregated = data["aggregated_summary"]
    stats = data["trajectory_statistical_significance"]

    for scenario, metrics in aggregated.items():
        label = f"Scenario = {scenario.capitalize()}"
        rows = []

        # Distance (Δ from Real)
        sim_mean = metrics["dist_sim"]["mean"] - metrics["dist_real"]["mean"]
        sim_std = metrics["dist_sim"]["std"]
        mapped_mean = metrics["dist_mapped"]["mean"] - metrics["dist_real"]["mean"]
        mapped_std = metrics["dist_mapped"]["std"]
        rows.append({
            "Metric": "Distance (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Trajectory Diff (Euclidean)
        sim_mean = metrics["traj_diff_sim_vs_real"]["mean"]
        sim_std = metrics["traj_diff_sim_vs_real"]["std"]
        mapped_mean = metrics["traj_diff_mapped_vs_real"]["mean"]
        mapped_std = metrics["traj_diff_mapped_vs_real"]["std"]
        rows.append({
            "Metric": "Trajectory Diff (Euclidean)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Trajectory Diff (Wasserstein)
        sim_mean = metrics["traj_std_sim_vs_real"]["mean"]
        sim_std = metrics["traj_std_sim_vs_real"]["std"]
        mapped_mean = metrics["traj_std_mapped_vs_real"]["mean"]
        mapped_std = metrics["traj_std_mapped_vs_real"]["std"]
        rows.append({
            "Metric": "Trajectory Diff (Wasserstein)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # Cumulative Error (Δ from Real)
        sim_mean = metrics["cumulative_sim"]["mean"] - metrics["cumulative_real"]["mean"]
        sim_std = metrics["cumulative_sim"]["std"]
        mapped_mean = metrics["cumulative_mapped"]["mean"] - metrics["cumulative_real"]["mean"]
        mapped_std = metrics["cumulative_mapped"]["std"]
        rows.append({
            "Metric": "Cumulative Error (Δ from Real)",
            "Sim": f"{sim_mean:.3f} ± {sim_std:.3f}",
            "Mapped": f"{mapped_mean:.3f} ± {mapped_std:.3f}",
        })

        # p/d (Trajectory vs Real)
        sim_p = stats[scenario]["p_value_traj_sim_vs_real"]
        sim_d = stats[scenario]["d_value_traj_sim_vs_real"]
        mapped_p = stats[scenario]["p_value_traj_mapped_vs_real"]
        mapped_d = stats[scenario]["d_value_traj_mapped_vs_real"]
        rows.append({
            "Metric": "p/d (Trajectory vs Real)",
            "Sim": f"p={sim_p:.2e}, d={sim_d:.2f}",
            "Mapped": f"p={mapped_p:.2e}, d={mapped_d:.2f}",
        })

        df = pd.DataFrame(rows)
        print(f"\n=== RQ2.1 TRAJECTORY TRACKING RESULTS FOR {label} ===")
        print(df.to_latex(index=False, escape=False, column_format='lll'))

generate_rq21_trajectory_tracking_summary()

# \begin{tabular}{llrrrrrrrr}
# \toprule
# Scenario & Metric & Sim & Mapped & Real & p (Sim vs Real) & d (Sim vs Real) & p (Mapped vs Real) & d (Mapped vs Real) \\
# \midrule
# close & Cumulative Distance & 0.045 & 0.018 & 0.017 &  &  &  &  \\
# close & Distance to Stop & 0.051 & 0.047 & 0.047 &  &  &  &  \\
# close & Trajectory Diff & 0.086 & 0.008 &  & 4.803e-01 & 0.03 & 9.228e-01 & -0.00 \\
# curve & Cumulative Distance & 0.183 & 0.097 & 0.094 &  &  &  &  \\
# curve & Distance to Stop & 0.203 & 0.072 & 0.072 &  &  &  &  \\
# curve & Trajectory Diff & 0.137 & 0.012 &  & 1.265e-01 & -0.07 & 9.698e-01 & 0.00 \\
# far & Cumulative Distance & 0.002 & 0.007 & 0.006 &  &  &  &  \\
# far & Distance to Stop & 0.175 & 0.074 & 0.074 &  &  &  &  \\
# far & Trajectory Diff & 0.115 & 0.010 &  & 7.679e-01 & -0.01 & 9.930e-01 & 0.00 \\
# sharp & Cumulative Distance & 0.101 & 0.179 & 0.176 &  &  &  &  \\
# sharp & Distance to Stop & 0.165 & 0.133 & 0.133 &  &  &  &  \\
# sharp & Trajectory Diff & 0.385 & 0.012 &  & 1.779e-01 & 0.06 & 9.892e-01 & 0.00 \\
# stest & Cumulative Distance & 0.537 & 0.485 & 0.483 &  &  &  &  \\
# stest & Distance to Stop & 0.176 & 0.103 & 0.103 &  &  &  &  \\
# stest & Trajectory Diff & 0.506 & 0.014 &  & 1.932e-01 & 0.06 & 9.850e-01 & -0.00 \\
# straight & Cumulative Distance & 0.002 & 0.032 & 0.031 &  &  &  &  \\
# straight & Distance to Stop & 0.054 & 0.046 & 0.047 &  &  &  &  \\
# straight & Trajectory Diff & 0.072 & 0.008 &  & 5.296e-01 & -0.03 & 9.827e-01 & 0.00 \\
# \bottomrule
# \end{tabular}