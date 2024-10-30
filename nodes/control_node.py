#!/usr/bin/env python3

import rospy
import json
import time
import websockets
import tf
import math

from std_msgs.msg import String, Bool, Int64
from geometry_msgs.msg import PoseStamped
from mixed_reality.msg import Control, WaypointList, Waypoint, SimPose
from mixed_reality.utils.for_conversions import For_convertion_utils
from mixed_reality.utils.control_utils import Waypoint_control_utils



#TODO: move this to a config file
control_uri = "ws://team10.local:8887/wsDrive"
FIXED_THROTTLE = True
REAL_CONTROLS = True
WAYPOINT_THRESHOLD = 2.5
# WAYPOINT_THRESHOLD = 0.3*7.33
ANGLE_THRESHOLD = 0

SIZE_FACTOR=7.33
X_MAP_SHIFT=48
Y_MAP_SHIFT=50
ANGLE_SHIFT=0
for_conversions = For_convertion_utils(SIZE_FACTOR,X_MAP_SHIFT,Y_MAP_SHIFT,ANGLE_SHIFT)




throttle = 1.0
steering = 0.0
brake = False
collision = "none"
going = False
state = "stopped"   #state can only be driving, stopping, or stopped
sim_pose = [0.,0.,0.]
sim_orientation = [0.,0.,0.]
waypoint_list = []
current_waypoint_index = 0
following_waypoints = False
pub_throttle_steering = None

TRACKING = True




def new_throttle_steering(msg):
    #having a list of waypoints to follow will be prioritised over receiving direct throttle and steering values from the model
    if not following_waypoints:
        global throttle
        global steering
        global brake
        throttle = msg.throttle
        steering = msg.steering
        brake = msg.brake

def new_collision(msg):
    global collision
    collision = msg.data

def new_going(msg):
    global going
    global state
    global brake
    global pub_throttle_steering
    if going and msg.data:
        print(f"CONTROL: State is already driving")
    elif not going and not msg.data:
        print(f"CONTROL: State is already {state}")
        if pub_throttle_steering is None:
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control, queue_size=10)
        pub_throttle_steering.publish(Control(0, steering, True, False, True))
        state = "stopping"
    elif msg.data:
        state = "driving"
        going = True
        brake = False
    else:
        state = "stopping"
        going = False
        if pub_throttle_steering is None:
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control, queue_size=10)
        pub_throttle_steering.publish(Control(0, steering, True, False, True))
    
def new_waypoints(msg):
    global waypoint_list
    global current_waypoint_index
    global following_waypoints
    global state
    global going
    waypoint_list = list(map(lambda wp:[wp.x, wp.y],msg.waypoints))
    if msg.reset: current_waypoint_index = 0
    if len(waypoint_list)==0:
        following_waypoints = False
        state = "stopping"
    else:
        following_waypoints = True
        state = "driving"
        going = True
    print(f"Received waypoint list of length {len(waypoint_list)}\nCurrent waypoint index is {current_waypoint_index}")

# def new_sim_pose(msg):
#     global sim_pose
#     global sim_orientation

#     sim_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
#     quaternion = (
#         msg.pose.orientation.x,
#         msg.pose.orientation.y,
#         msg.pose.orientation.z,
#         msg.pose.orientation.w)
#     euler = tf.transformations.euler_from_quaternion(quaternion) #this gives roll, pitch, yaw
#     angle_rad = euler[1]
#     if angle_rad<0:
#         angle_rad = angle_rad + 2*math.pi
#     sim_orientation = [math.degrees(euler[2]), math.degrees(angle_rad), math.degrees(euler[0])]    #sim_orientation is yaw, pitch, roll
#     print(f"Angle: {round(sim_orientation[1], 3)}, {round(angle_rad, 3)} rad")
#     print(f"Quaternion {quaternion}")

def new_pose(msg):
    global sim_pose
    global sim_orientation
    global for_conversions
    if TRACKING:
        position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        r = math.degrees(euler[2])
        tracked_pose_sim = for_conversions.real2sim_xyp([position[0], position[1], r])
        # print(f"Received position is {tracked_pose_sim}")
        sim_pose = [tracked_pose_sim[0],0,tracked_pose_sim[1]]
        sim_orientation = [0,tracked_pose_sim[2],0]
    else:
        sim_pose = [msg.x,msg.y,msg.z]
        sim_orientation = [0,msg.yaw,0]
    
    




def control_node():
    print("Starting control node")
    global control_uri      #param
    global FIXED_THROTTLE   #param
    global REAL_CONTROLS    #param (maybe unnecessary)
    global following_waypoints #set by keyboard
    global throttle     #
    global steering
    global brake
    global collision
    global state
    global going
    global current_waypoint_index
    global waypoint_list
    global pub_throttle_steering
    global sim_pose
    global sim_orientation
    global TRACKING

    #TODO: get rid of TESTING
    TESTING = True
    
    rospy.init_node("control_node", anonymous=True)
    rospy.Subscriber("model/throttle_steering", Control, new_throttle_steering)
    rospy.Subscriber("collision", String, new_collision)
    rospy.Subscriber("going", Bool, new_going)
    rospy.Subscriber("waypoints", WaypointList, new_waypoints)
    #rospy.Subscriber("sim/pose", PoseStamped, new_sim_pose)
    if TRACKING:
        rospy.Subscriber("donkey/pose", PoseStamped, new_pose)
    else:
        rospy.Subscriber("sim/euler", SimPose, new_pose)
    if pub_throttle_steering is None:
        pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control, queue_size=10)
    pub_current_waypoint_index = rospy.Publisher("control/current_waypoint_index", Int64, queue_size=10)
    #TODO: check if this rate is fine
    rate = rospy.Rate(50)

    waypoint_controller = Waypoint_control_utils(WAYPOINT_THRESHOLD, ANGLE_THRESHOLD)
    
    while not rospy.is_shutdown():
        try:
            if collision!="none" and state!="stopped":
                print("COLLISION!", collision)
                state = "stopping"
                going = False
                break

            if following_waypoints and going:
                #recalculate commands to next waypoint
                current_waypoint = waypoint_list[current_waypoint_index]
                # print("Going to: ", current_waypoint)
                x, y = current_waypoint

                # while abs(waypoint_controller.calculate_angle(x,y,sim_pose,sim_orientation))>90:
                #     current_waypoint_index+=1
                #     if current_waypoint_index >=len(waypoint_list):
                #         print("Remaining waypoints are all behind the car")
                #         state = "stopping"
                #         following_waypoints = False
                #         break
                #     current_waypoint = waypoint_list[current_waypoint_index]
                #     x, y = current_waypoint

                print("Going to: ", current_waypoint)
                x, y = current_waypoint

                distance = waypoint_controller.calculate_distance(x,y,sim_pose)
                if distance <= WAYPOINT_THRESHOLD:
                    print("reached")
                    current_waypoint_index += 1  # Move to the next waypoint
                    if  current_waypoint_index < len(waypoint_list):
                        current_waypoint = waypoint_list[current_waypoint_index]
                        # print("Going to: ", current_waypoint)
                    else:
                        if TESTING:
                            print("reached final waypoint, looping")
                            current_waypoint_index = 0
                        else:
                            if going:
                                print("Reached final waypoint")
                            state = "stopping"
                            following_waypoints = False

                x, y = current_waypoint
                steering, throttle, _, _ = waypoint_controller.calculate_control(x, y, sim_pose, sim_orientation)
                pub_current_waypoint_index.publish(current_waypoint_index)

            if brake:
                #send brake command
                if state=="driving":
                    pub_throttle_steering.publish(Control(-throttle, steering, True, False, not going))
            else:
                #send commands
                if state=="stopping":
                    state = "stopped"
                    going = False
                    pub_throttle_steering.publish(Control(-throttle, steering, True, False, True))
                elif state=="stopped":
                    going = False
                    pub_throttle_steering.publish(Control(0, steering, True, False, True))
                else:
                    # print(f"Going to {current_waypoint} from {sim_pose}")
                    pub_throttle_steering.publish(Control(throttle, steering, False, throttle<0, not going))

            rate.sleep()
        except Exception as e:
            print(f"Error: {e}")
    


if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass