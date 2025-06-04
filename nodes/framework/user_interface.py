#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64, String
from mixed_reality.msg import WaypointList, Waypoint, SimPose
import json
import os
import time

from mixed_reality.msg import Control
from mixed_reality.utils.for_conversions import For_convertion_utils




help = "\nKEYBOARD COMMANDS:\n'g' \t to start the car\n's' \t to stop the car\n'r' \t to reset the simulator scenario\n'k' \t to kill all nodes't:<speed>' \t to set a target speed\n'w:<x,y>' \t to set a waypoint\n'i' \t to increase throttle\n'd' \t to decrease throttle\n'exit' \t to quit this node\n'help' \t to show this message again\n"

"""
TODO: brake
TODO: change lanes
"""

throttle_multiplier = rospy.get_param("default_throttle_multiplier")


SIZE_FACTOR = rospy.get_param("size_factor")
X_MAP_SHIFT = rospy.get_param("x_map_shift")
Y_MAP_SHIFT = rospy.get_param("y_map_shift")
ANGLE_SHIFT = rospy.get_param("angle_shift")
for_conversions = For_convertion_utils(SIZE_FACTOR,X_MAP_SHIFT,Y_MAP_SHIFT,ANGLE_SHIFT)

map_name = rospy.get_param("map_name")


speed = 0
def new_speed(msg):
    global speed
    speed = msg.data

sim_pose = None
def new_pose(msg):
    global sim_pose
    sim_pose = [msg.x,msg.z]


def keyboard_node():
    global throttle_multiplier
    global for_conversions
    global sim_pose


    rospy.init_node("keyboard_node", anonymous=True)
    pub_going=rospy.Publisher("/going",Bool,queue_size=10)
    pub_throttle_multiplier = rospy.Publisher("throttle/multiplier", Float64, queue_size=10)
    pub_target_speed = rospy.Publisher("keyboard/speed", Float64, queue_size=10)
    pub_waypoint = rospy.Publisher("waypoints", WaypointList,queue_size=10)
    pub_reset = rospy.Publisher("reset", Bool, queue_size=10)
    pub_current_lane = rospy.Publisher("current_lane", String, queue_size=10)

    while not rospy.is_shutdown():
        message = input("Enter message: (type 'help' for more info)\n")
        if message == "help":
            print(help)
        elif message == "g":
            print("Starting command received")
            pub_going.publish(True)
        elif message == "s":
            print("Stopping command received")
            pub_going.publish(False)
        elif message == "i":
            throttle_multiplier += 0.005
            print(f"Increase command received, throttle multiplier is now {throttle_multiplier}")
            pub_throttle_multiplier.publish(throttle_multiplier)
        elif message == "d":
            if throttle_multiplier>0.01:
                throttle_multiplier -= 0.01
            elif throttle_multiplier>0:
                throttle_multiplier = 0
            print(f"Decrease command received, throttle multiplier is now {throttle_multiplier}")
            pub_throttle_multiplier.publish(throttle_multiplier)
        elif message == "r":
            pub_reset.publish(True)
            print("Resetting scenario")
        elif message == "exit":
            print("Exiting keyboard node")
            break
        elif message == "k":
            print("Killing all nodes")
            os.system("rosnode kill -a")
        elif "t:" in message:
            if(len(message.split("t:"))>1):
                target = float(message.split("t:")[1])
                pub_target_speed.publish(target)
                print(f"Setting target speed to {target}")
            else:
                print("There was an error parsing the target speed\nExample usage: 't: 0.7'")
        elif "w:" in message:
            if len(message.split("w:"))>1 and len(message.split("w:")[1].split(","))==2:
                real_x, real_y=message.split("w:")[1].split(",")
                sim_pose = for_conversions.real2sim_xyp([float(real_x),float(real_y),0])
                wp  = Waypoint(sim_pose[0],sim_pose[1])
                wp_list = [wp]
                pub_waypoint.publish(True,wp_list)
                print(f"Going to waypoint {wp}")
            else:
                print("There was an error parsing the waypoint\nExample usage: 'w: 1.0, 0.8'")
        elif "demo" in message:
            print(f"following map with name {map_name}")
            f = open(map_name, "r")
            map_data = json.loads(f.read())
            f.close()

            reverse=False
            lane="center"

            if reverse and lane=="right":
                lane_map="left"
            elif reverse and lane=="left":
                lane_map="right"
            else:
                lane_map=lane

            if lane=="center":
                wp=map_data["sim_waypoint_list"]
            else:    
                wp = map_data[f"{lane_map}_lane"]





            
            wp_list = list(map(lambda pair:Waypoint(pair[0], pair[1]), wp))
            if reverse:
                wp_list.reverse()
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_waypoint.publish(True,wp_list)
            pub_current_lane.publish(lane)
            print("waypoints published")
        elif "forward" in message:
            #forward 0.39 1
            #forward 0.34 1
            #forward 0.365 1
            print("starting forward difference experiment")
            parts = message.split(" ")
            throttle = float(parts[1])
            duration = 3.
            # iter = int(parts[2])
            
            
            steering = 0
            print(f"applying throttle {throttle} for {duration} seconds")
            # command = "rosbag record -O t"+"%0.2f"%throttle+"_"+str(iter)+" /donkey/pose /sim/euler /donkey/speed /control/throttle_steering /sim/speed /going /reset"
            # print(command)
            # os.system(command)
            # time.sleep(0.5)
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_throttle_multiplier.publish(1)
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control,queue_size=10)
            
            start = time.time()
            while time.time()-start <duration:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
                
            pub_throttle_steering.publish(Control(0,steering,False,False,False))
            pub_going.publish(False)
        elif "arc" in message:

            # arc 1
            # arc 0.6
            # arc 0.3
            # arc -0.6
            # arc -0.3
            # arc -1

            print("starting arc difference experiment")
            parts = message.split(" ")
            steering = float(parts[1])
            throttle = 0.365
            duration = 6
            #duration = 8
            # duration = 6
            
            print(f"applying steering {steering} an throttle {throttle} fro {duration} seconds")
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control,queue_size=10)
            pub_throttle_multiplier.publish(1)
            sim_mult = rospy.Publisher("throttle_sim/multiplier", Float64, queue_size=10)
            sim_mult.publish(1)
            start = time.time()
            while time.time()-start <duration:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            pub_throttle_steering.publish(Control(0,steering,False,False,False))
            while time.time()-start <duration+10:
                pub_throttle_steering.publish(Control(0,steering,False,False,False))
        elif "braking" in message:
            global speed
            print("starting braking behaviour experiment")
            # parts = message.split(" ")
            steering = 0
            throttle = 1
            target = 1
            rospy.Subscriber("donkey/speed", Float64, new_speed)
            print(f"applying throttle {throttle} until speed is {target}")
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control,queue_size=10)
            while target-speed>0.1:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            print("speed reaches, applying brakes")
            while speed>0.1:
                pub_throttle_steering.publish(Control(0,steering,True,False,True))
            print("braking operation is over")
        elif "pid" in message:
            steering = -0.6
            throttle = 1
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control,queue_size=10)
            time_block = 10
            target = 0.4
            print(f"Setting target speed to {target} for {time_block} seconds")
            pub_target_speed.publish(target)
            start = time.time()
            while time.time() - start <time_block:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            target = 0.8
            print(f"Setting target speed to {target} for {time_block} seconds")
            pub_target_speed.publish(target)
            while time.time() - start<2*time_block:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            target = 0.6
            print(f"Setting target speed to {target} for {time_block} seconds")
            pub_target_speed.publish(target)
            while time.time() - start<3*time_block:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            # target = 0.6
            # print(f"Setting target speed to {target} for {time_block} seconds")
            # pub_target_speed.publish(target)
            # while time.time() - start<4*time_block:
            #     pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            target = 0
            print(f"Setting target speed to {target} for {3} seconds")
            pub_target_speed.publish(target)
            while time.time() - start<3*time_block+3:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            print("EXPERIMENT DONE")
        elif "straight" in message:
            #get sim pose
            rospy.Subscriber("sim/euler", SimPose, new_pose)
            while sim_pose is None:
                pass
            
            print("going to waypoint 15 sim units in front")
            wp = Waypoint(sim_pose[0]+15,sim_pose[1])
            wp_list = [wp]
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_waypoint.publish(True,wp_list)
        elif "close" in message:
            #get sim pose
            rospy.Subscriber("sim/euler", SimPose, new_pose)
            while sim_pose is None:
                pass
            
            print("going to waypoint 10 sim units in front, 5 to the left")
            wp = Waypoint(sim_pose[0]+10,sim_pose[1]+5)
            wp_list = [wp]
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_waypoint.publish(True,wp_list)
        elif "far" in message:
            #get sim pose
            rospy.Subscriber("sim/euler", SimPose, new_pose)
            while sim_pose is None:
                pass
            
            print("going to waypoint 20 sim units in front, 5 to the right")
            wp = Waypoint(sim_pose[0]+20,sim_pose[1]-5)
            wp_list = [wp]
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_waypoint.publish(True,wp_list)

        elif "sharp" in message:
            #get sim pose
            rospy.Subscriber("sim/euler", SimPose, new_pose)
            while sim_pose is None:
                pass
            
            print("sharp turn to the left")
            wp_list = [Waypoint(sim_pose[0]+10,sim_pose[1]),
                       Waypoint(sim_pose[0]+20,sim_pose[1]),
                       Waypoint(sim_pose[0]+20,sim_pose[1]+10)
                       ]
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_waypoint.publish(True,wp_list)

        elif "curve" in message:
            #get sim pose
            rospy.Subscriber("sim/euler", SimPose, new_pose)
            while sim_pose is None:
                pass
            
            print("curve to the right")
            wp_list = [Waypoint(sim_pose[0]+8,sim_pose[1]+2),
                       Waypoint(sim_pose[0]+14,sim_pose[1]+8),
                       Waypoint(sim_pose[0]+20,sim_pose[1]+10)
                       ]
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_waypoint.publish(True,wp_list)

        elif "stest" in message:
            #get sim pose
            rospy.Subscriber("sim/euler", SimPose, new_pose)
            while sim_pose is None:
                pass
            
            print("s shape road")
            wp_list = [Waypoint(sim_pose[0]+6,sim_pose[1]+2),
                       Waypoint(sim_pose[0]+9,sim_pose[1]+6),
                       Waypoint(sim_pose[0]+14,sim_pose[1]+4),
                       Waypoint(sim_pose[0]+16,sim_pose[1]),
                       Waypoint(sim_pose[0]+18,sim_pose[1]-4),
                       Waypoint(sim_pose[0]+24,sim_pose[1]-8),
                       Waypoint(sim_pose[0]+27,sim_pose[1]+2)]
            pub_going = rospy.Publisher("/going", Bool, queue_size=10)
            pub_going.publish(True)
            pub_waypoint.publish(True,wp_list)
            

        else:
            print(f"Command '{message}' not recognized\n")
            print(help)






if __name__ == '__main__':
    try:
        keyboard_node()
    except rospy.ROSInterruptException:
        pass