#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64, String
from mixed_reality.msg import SimPose, WaypointList, Waypoint, Obstacles

import json
import math


def new_obstacle(msg):
    global obstacles
    # Update obstacle positions
    obstacles = {obs.name: [obs.x, obs.y] for obs in msg.data}


def new_current_waypoint_index(msg):
    global current_waypoint_index
    current_waypoint_index = msg.data


def new_sim_pose(msg):
    global simulator_pose
    # Store x, y (z), yaw
    simulator_pose = [msg.x, msg.z, msg.yaw]


def get_closest_lane(pos, left_waypoints, right_waypoints):
    # Determine the closest lane ('left' or 'right') for a given position
    min_dist = float('inf')
    closest = None
    for lane, waypoints in [("left", left_waypoints), ("right", right_waypoints)]:
        for wp in waypoints:
            d = math.hypot(pos[0] - wp[0], pos[1] - wp[1])
            if d < min_dist:
                min_dist = d
                closest = lane
    return closest, min_dist


def obstacleAvoidance_node():
    global obstacles, current_waypoint_index, simulator_pose

    rospy.init_node("obstacleAvoidance_node", anonymous=True)

    # Parameters
    THRESHOLD = rospy.get_param("obstacle_distance_threshold")
    map_name = rospy.get_param("map_name")
    initial_lane = rospy.get_param("initial_lane", "center")
    reverse = rospy.get_param("reverse", False)

    # Load map data
    with open(map_name, 'r') as f:
        map_data = json.load(f)

    # Prepare waypoint lists
    if reverse:
        sim_waypoint_list_left   = map_data["right_lane"][::-1]
        sim_waypoint_list_right  = map_data["left_lane"][::-1]
        sim_waypoint_list_center = map_data["sim_waypoint_list"][::-1]
    else:
        sim_waypoint_list_left   = map_data["left_lane"]
        sim_waypoint_list_right  = map_data["right_lane"]
        sim_waypoint_list_center = map_data["sim_waypoint_list"]

    # State variables
    obstacles = {}
    current_waypoint_index = 0
    simulator_pose = [0.0, 0.0, 0.0]
    current_lane = initial_lane

    # Subscribers & Publisher
    rospy.Subscriber("obstacles", Obstacles, new_obstacle)
    rospy.Subscriber("control/current_waypoint_index", Int64, new_current_waypoint_index)
    rospy.Subscriber("sim/euler", SimPose, new_sim_pose)
    pub_waypoint = rospy.Publisher("waypoints", WaypointList, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        ego_x, ego_y = simulator_pose[0], simulator_pose[1]

        # If in center lane, check for obstacle to decide side shift
        if current_lane == "center":
            for name, pos in obstacles.items():
                d_ego = math.hypot(pos[0] - ego_x, pos[1] - ego_y)
                closest_lane, _ = get_closest_lane(pos, sim_waypoint_list_left, sim_waypoint_list_right)

                # If obstacle within threshold in a side lane -> shift to opposite
                if d_ego < THRESHOLD and closest_lane in ["left", "right"]:
                    target_lane = "right" if closest_lane == "left" else "left"
                    rospy.loginfo(f"Obstacle '{name}' detected in {closest_lane} lane (d={d_ego:.2f}). Shifting to {target_lane}.")
                    waypoints = sim_waypoint_list_right if target_lane == "right" else sim_waypoint_list_left
                    wp_list = [Waypoint(x, y) for x, y in waypoints]
                    pub_waypoint.publish(False, wp_list)
                    current_lane = target_lane
                    break  # Only handle the first triggering obstacle

        # If in a side lane, return only when no obstacle within threshold
        else:
            safe = True
            for name, pos in obstacles.items():
                d_ego = math.hypot(pos[0] - ego_x, pos[1] - ego_y)
                if d_ego < THRESHOLD:
                    safe = False
                    break
            if safe:
                rospy.loginfo(f"No obstacles within threshold. Returning to center lane.")
                wp_list = [Waypoint(x, y) for x, y in sim_waypoint_list_center]
                pub_waypoint.publish(False, wp_list)
                current_lane = "center"

        rate.sleep()

if __name__ == '__main__':
    try:
        obstacleAvoidance_node()
    except rospy.ROSInterruptException:
        pass
