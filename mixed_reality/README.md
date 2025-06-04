# ROS Small-Scale Vehicle

## Overview

This package provides a mixed-reality robotics framework for small-scale autonomous vehicle experiments. It supports RW, SiL, ViL, and MR testing using a modular architecture.

---

## Add This Directory as a `catkin_ws` Package

Make sure `mixed_reality` is located inside the `src/` folder of your `catkin_ws` and build using:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Node Overview

### Running Nodes Individually

Use the following command format:

```bash
rosrun mixed_reality [NODE_NAME].py
```

---

### Framework Nodes

| Node Name             | Description                                                                 |
|-----------------------|-----------------------------------------------------------------------------|
| `simulator_interface` | Connects the simulator to the framework (requires simulator binary running) |
| `tracking_interface`  | Connects the Vicon-based tracking system                                    |
| `car_interface`       | Interfaces with DonkeyCar (actuation)                                       |
| `camera_interface`    | Interfaces with DonkeyCar camera                                            |
| `tof_interface`       | Interfaces with DonkeyCar ToF sensor                                        |
| `control`             | Manages low-level and high-level control logic                              |
| `real_pid`            | PID controller for real-world operation                                     |
| `sim_pid`             | PID controller for simulated operation                                      |
| `user_interface`      | Enables user control via keyboard                                           |

---

### Sensor Processing Nodes

| Node Name                    | Description                                      |
|-----------------------------|--------------------------------------------------|
| `camera_mixing`             | Mixes simulated and real camera images           |
| `tof_mixing`                | Mixes simulated and real ToF data                |
| `sim_pointcloud_generation` | Generates pointcloud from simulated ToF          |
| `real_pointcloud_generation`| Generates pointcloud from real ToF               |
| `mixed_pointcloud_generation`| Generates pointcloud from mixed ToF             |

---

### Autonomous Driving System (ADS) Nodes

| Node Name                       | Description                                                                |
|--------------------------------|----------------------------------------------------------------------------|
| `npc_obstacleavoidance`        | Enables GT driving data collection by switching lanes to avoid obstacles             |
| `e2e_model`                    | Loads and runs Dave2 end-to-end model on real/sim/mixed camera data        |
| `modular_[real/sim]_perception`| Processes pointcloud data for obstacle detection                           |
| `modular_latticeplanner`       | Generates local waypoints using obstacle and vehicle pose data             |

---

## Usage

### Start Framework Without Nodes

```bash
roslaunch mixed_reality empty.launch
```

**Parameter configurations:**

- **RW / MR / ViL**:  
  ```bash
  roslaunch mixed_reality empty.launch mapping:=true tracking:=true
  ```

- **SiL (with real scenario positioning)**:  
  ```bash
  roslaunch mixed_reality empty.launch mapping:=true tracking:=false
  ```

- **SiL (isolated)**:  
  ```bash
  roslaunch mixed_reality empty.launch mapping:=false tracking:=false
  ```

---

## Behaviour Gap Experiments

### 1. SiL (No Real Positioning)

```bash
roslaunch mixed_reality empty.launch mapping:=false tracking:=false
rosrun mixed_reality user_interface.py
```

**E2E:**
```bash
rosrun mixed_reality e2e_model.py
```

**Modular:**
```bash
rosrun mixed_reality sim_pointcloud_generation.py
rosrun mixed_reality modular_sim_perception.py
rosrun mixed_reality modular_latticeplanner.py
```

---

### 2. SiL (With Real Scenario Positioning)

```bash
roslaunch mixed_reality SiL.launch
rosrun mixed_reality user_interface.py
```

**E2E:**
```bash
rosrun mixed_reality e2e_model.py
```

**Modular:**
```bash
rosrun mixed_reality sim_pointcloud_generation.py
rosrun mixed_reality modular_sim_perception.py
rosrun mixed_reality modular_latticeplanner.py
```

---

### 3. ViL

```bash
roslaunch mixed_reality ViL.launch
rosrun mixed_reality user_interface.py
```

**E2E:**
```bash
rosrun mixed_reality e2e_model.py
```

**Modular:**
```bash
rosrun mixed_reality real_pid.py
rosrun mixed_reality sim_pointcloud_generation.py
rosrun mixed_reality modular_sim_perception.py
rosrun mixed_reality modular_latticeplanner.py
```

---

### 4. RW (Real-World)

```bash
roslaunch mixed_reality RW.launch
rosrun mixed_reality user_interface.py
```

**E2E:**
```bash
rosrun mixed_reality camera_interface.py
rosrun mixed_reality e2e_model.py
```

**Modular:**
```bash
rosrun mixed_reality tof_interface.py
rosrun mixed_reality real_pid.py
rosrun mixed_reality real_pointcloud_generation.py
rosrun mixed_reality modular_real_perception.py
rosrun mixed_reality modular_latticeplanner.py
```

---

### 5. MR (Mixed-Reality)

```bash
roslaunch mixed_reality MR.launch
rosrun mixed_reality user_interface.py
```

**E2E:**
```bash
rosrun mixed_reality camera_interface.py
rosrun mixed_reality camera_mixing.py
rosrun mixed_reality e2e_model.py
```

**Modular:**
```bash
rosrun mixed_reality tof_interface.py
rosrun mixed_reality tof_mixing.py
rosrun mixed_reality real_pid.py
rosrun mixed_reality mixed_pointcloud_generation.py
rosrun mixed_reality modular_real_perception.py
rosrun mixed_reality modular_latticeplanner.py
```

---

## Actuation Gap Experiments

Start with:

```bash
roslaunch mixed_reality SiL.launch
# or
roslaunch mixed_reality ViL.launch
```

Then:

```bash
rosrun mixed_reality user_interface.py
```

Once running, use the following commands:

### Forward Experiments
```
forward 0.39 3
forward 0.365 3
forward 0.34 3
```

### Steering Experiments
```
arc 1
arc 0.6
arc 0.3
arc -0.6
arc -0.3
arc -1
```

### Braking Experiments
First set throttle multiplier:
```
mult: 0.34
mult: 0.365
mult: 0.39
```

Then:
```
forward
```

### PID Experiment
```
pid
```

### Waypoint Experiments
```
straight
close
far
sharp
curve
stest
```