Replication package

# ROS-small-scale-vehicle

Launchfile descriptions (found in launch/, see their implementations for more details):

model.launch:
Launches keyboard node, simulator node, vicon tracking node, car node, camera node, and image mixer node, loads parameters set in params.yaml
Defaults to mixed reality scenarios, this can be changed by adding mixed:=false and real:=true or sim:=true as command line arguments when launching.
Can launch speed controllers for sim and real donkeycar if throttle_adjust:=true is passed as an argument.
To run the model node separately set model:=false
To launch the lidar node set lidar:=true
Mapping and tracking settings default to true, can be changed through command line arguments. Tracking and car nodes will not be launched with tracking:=false

empty.launch:
Loads parameters from params.yaml to the parameter server.
Launches no nodes.
Defaults to mixed reality scenarios.
Mapping and tracking parameters default to true.

example.launch:
Launches keyboard, simulator, vicon tracking, control, real and sim speed control, car, camera, image mixing, and lane-changing nodes, loads parameters in params.yaml
Tracking, mapping, and mixed reality scenarios are activated by default.
If adjust_throttle:=false is set, the speed control nodes will not be launched.
If tracking:=false is set, tracking, car, and real speed control nodes will not be launched.
If sim:=true is set, the camera node will not be launched.
If mixed:=false, the image mixing node will not be launched.
To launch the lidar node, add lidar:=true to command
To change planning implementation to lattice planner, set lattice:=true and lane_changing:=false
