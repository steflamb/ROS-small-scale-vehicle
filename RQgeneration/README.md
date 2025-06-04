# RQ-generation-plots

This folder contains scripts used to process ROS bag files and extract the data required to answer the research questions (RQs) from the study.

---

## Script Categories

### A. Perception Gap (Camera)

Scripts used to analyze camera-based perception performance, including lane and obstacle detection:

- `A_process_inputs_camera_lanes.py` |    Used to manually label lane splines
- `A_process_inputs_camera_obstacles.py`  |    Used to manually label obstacles bounding boxes
- `A_process_lane_results.py`  |    Processes the manually labeled data
- `A_process_obstacles_results.py`  |     Processes the manually labeled data
- `A_compare_camera.py`  |     Compares images for perception gap

---

### B. Perception Gap (LiDAR)

Scripts used to analyze LiDAR-based obstacle detection and perception accuracy:

- `B_process_inputs_lidar.py`    |    Used to manually label obstacles bounding boxes
- `B_process_inputs_lidar_multiple.py`    |    Used to manually label obstacles bounding boxes (2 obstacles per frame)
- `B_process_obstacles_results.py`  |    Processes the manually labeled data
- `B_process_obstacles_results_multiple.py`  |    Processes the manually labeled data
- `B_compare_lidar.py`  |     Compares pointclouds for perception gap

---

### C. Actuation Gap

Scripts for analyzing actuation behavior in throttle, steering, braking, and PID responses:

- `C_process_actuation_throttle.py`  
- `C_process_actuation_throttle_statistics.py`  
- `C_process_actuation_steering.py`  
- `C_process_actuation_steering_statistics.py`  
- `C_process_actuation_braking.py`  
- `C_process_actuation_braking_statistics.py`  
- `C_process_actuation_pid.py`  
- `C_process_actuation_pid_plots.py`  
- `C_process_actuation_waypoints.py`  

---

### D. Behaviour Gap (End-to-End)

Scripts focused on evaluating the behavior of end-to-end (E2E) autonomous driving systems:

- `D_process_e2e.py`  
- `D_process_e2e_merged_plot.py`
- `D_process_e2e_generalization.py`  

---

### E. Behaviour Gap (Modular)

Scripts focused on evaluating the modular autonomous driving pipeline:

- `D_process_modular.py`  
- `D_process_modular_merged_plot.py`  
- `D_process_modular_generalization.py`  
- `D_process_modular_lattice.py`  

---

## Notes

- Each script is designed to operate on `.bag` files containing recorded ROS topics from experiments.
