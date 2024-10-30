#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64, String
from mixed_reality.msg import WaypointList, Waypoint
import json
import os
import time

from mixed_reality.msg import Control




help = "\nKEYBOARD COMMANDS:\n'g' \t to start the car\n's' \t to stop the car\n'r' \t to reset the simulator scenario\n'k' \t to kill all nodes't:<speed>' \t to set a target speed\n'w:<x,y>' \t to set a waypoint\n'i' \t to increase throttle\n'd' \t to decrease throttle\n'exit' \t to quit this node\n'help' \t to show this message again\n"

"""
TODO: brake
TODO: change lanes
"""

throttle_multiplier = 0.3



class For_convertion_utils():
    def __init__(self,size_factor,x_map_shift,y_map_shift,angle_shift):
        self.size_factor=size_factor
        self.x_map_shift=x_map_shift
        self.y_map_shift=y_map_shift
        self.angle_shift=angle_shift
    
    def real2sim_xyzypr(self,pose,orientation):
        x=pose[0]*self.size_factor+self.x_map_shift
        y=pose[1]*self.size_factor+self.y_map_shift
        angle=-orientation[2]+self.angle_shift
        return [x,y,angle]

    def sim2real_xyzypr(self,pose,orientation):
        x=(pose[0]-self.x_map_shift)/self.size_factor
        y=(pose[2]-self.x_map_shift)/self.size_factor
        angle=-(orientation[1]-self.angle_shift)
        return [x,y,angle]
    
    def real2sim_xyp(self,pose):
        x,y,p=pose
        x=x*self.size_factor+self.x_map_shift
        y=y*self.size_factor+self.y_map_shift
        angle=-p+self.angle_shift
        return [x,y,angle]

    def sim2real_xyp(self,pose):
        x,y,p=pose
        x=(x-self.x_map_shift)/self.size_factor
        y=(y-self.y_map_shift)/self.size_factor
        angle=-(p-self.angle_shift)
        return [x,y,angle]

SIZE_FACTOR=7.33
X_MAP_SHIFT=48
Y_MAP_SHIFT=50
ANGLE_SHIFT=0
for_conversions = For_convertion_utils(SIZE_FACTOR,X_MAP_SHIFT,Y_MAP_SHIFT,ANGLE_SHIFT)


speed = 0
def new_speed(msg):
    global speed
    speed = msg.data



def keyboard_node():
    global throttle_multiplier
    global for_conversions


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
                wp = Waypoint(sim_pose[0],sim_pose[1])
                wp_list = [wp]
                pub_waypoint.publish(True,wp_list)
                print(f"Going to waypoint {wp}")
            else:
                print("There was an error parsing the waypoint\nExample usage: 'w: 1.0, 0.8'")
        elif "demo" in message:
            f = open("closed_road_shape.json", "r")
            map_data = json.loads(f.read())
            f.close()

            reverse=True   #TODOSTE: remove this reversal
            lane="left"

            if reverse and lane=="right":
                lane_map="left"
            elif reverse and lane=="left":
                lane_map="right"
            wp = map_data[f"{lane_map}_lane"]
            wp_list = list(map(lambda pair:Waypoint(pair[0], pair[1]), wp))
            if reverse:
                wp_list.reverse()
            pub_waypoint.publish(True,wp_list)
            pub_current_lane.publish(lane)
        elif "forward" in message:
            print("starting forward difference experiment")
            parts = message.split(" ")
            throttle = float(parts[1])
            duration = float(parts[2])
            steering = 0
            print(f"applying throttle {throttle} for {duration} seconds")
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control,queue_size=10)
            pub_throttle_multiplier.publish(1)
            sim_mult = rospy.Publisher("throttle_sim/multiplier", Float64, queue_size=10)
            sim_mult.publish(1)
            start = time.time()
            while time.time()-start <duration:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            pub_throttle_steering.publish(Control(0,steering,False,False,False))
        elif "arc" in message:
            print("starting arc difference experiment")
            parts = message.split(" ")
            steering = float(parts[1])
            throttle = float(parts[2])
            duration = float(parts[3])
            print(f"applying steering {steering} an throttle {throttle} fro {duration} seconds")
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control,queue_size=10)
            pub_throttle_multiplier.publish(1)
            sim_mult = rospy.Publisher("throttle_sim/multiplier", Float64, queue_size=10)
            sim_mult.publish(1)
            start = time.time()
            while time.time()-start <duration:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            pub_throttle_steering.publish(Control(0,steering,True,False,True))
        elif "braking" in message:
            print("starting braking behaviour experiment")
            parts = message.split(" ")
            steering = 0
            throttle = float(parts[1])
            target = float(parts[2])
            rospy.Subscriber("donkey/speed", new_speed)
            print(f"applying throttle {throttle} until speed is {target}")
            pub_throttle_steering = rospy.Publisher("control/throttle_steering", Control,queue_size=10)
            pub_throttle_multiplier.publish(1)
            sim_mult = rospy.Publisher("throttle_sim/multiplier", Float64, queue_size=10)
            sim_mult.publish(1)
            while abs(speed-target)>0.1:
                pub_throttle_steering.publish(Control(throttle,steering,False,False,False))
            print("speed reaches, applying brakes")
            while speed>0.1:
                pub_throttle_steering.publish(Control(0,steering,True,False,True))
            print("braking operation is over")
        else:
            print(f"Command '{message}' not recognized\n")
            print(help)






if __name__ == '__main__':
    try:
        keyboard_node()
    except rospy.ROSInterruptException:
        pass