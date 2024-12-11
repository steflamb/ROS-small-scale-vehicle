#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import Image as SensorImage
from mixed_reality.msg import SimPose, Floats, Control, Obstacles
import math
from datetime import datetime, timedelta
import json

import numpy as np

import cv2

from PIL import Image

import time

import base64
from io import BytesIO

from typing import Union

from gym_donkeycar.core.sim_client import SimClient
from gym_donkeycar.core.message import IMesgHandler

import tf
import tf.transformations

from rospy.numpy_msg import numpy_msg

from mixed_reality.utils.for_conversions import For_convertion_utils



SIZE_FACTOR = rospy.get_param("size_factor")
X_MAP_SHIFT = rospy.get_param("x_map_shift")
Y_MAP_SHIFT = rospy.get_param("y_map_shift")
ANGLE_SHIFT = rospy.get_param("angle_shift")
  
map_name = rospy.get_param("map_name")

simulator_ip = rospy.get_param("simulator_ip")
simulator_port = rospy.get_param("simulator_port")
MAPPING = rospy.get_param("mapping")
TRACKING = rospy.get_param("tracking")

for_conversions = For_convertion_utils(SIZE_FACTOR,X_MAP_SHIFT,Y_MAP_SHIFT,ANGLE_SHIFT)
position_tracked = False

#collection of variables that might be changed through ROS later
simulator_client = None

simulator_pose=[0.,0.,0.]
simulator_orientation=[0.,0.,0.]

simulator_cte=0.0

sim_image=None
sim_image_for_model=None

collision=None

simulated_speed=0.0

tracked_pose_sim=[1.,0.,1.]

throttle = 0
steering = 0
throttle_multiplier = rospy.get_param("default_throttle_multiplier")

counter=0
prev_time=datetime.now()


class DonkeySimMsgHandler(IMesgHandler):
    def __init__(self):
        
        self.fns = {'telemetry' : self.on_telemetry,\
                    'car_loaded' : self.on_car_created,\
                    'on_disconnect' : self.on_disconnect,
                    'aborted' : self.on_aborted}
        
        self.cte=0.0
        self.speed=0.0
        self.pos_x=0.0
        self.pos_y=0.0
        self.pos_z=0.0
        self.angle=None
        self.image_toplot=None
        self.img_arr=None
        self.client = None
        self.rand_seed=0
        self.hit=None
        self.speed=0.0

    def quaternion_to_euler(self, quaternion):
        w, x, y, z = quaternion
    
        # Calculate rotation angle
        rotation_angle_radians = np.arctan2(2 * (w * y + x * z), 1 - 2 * (y ** 2 + z ** 2))
        
        # Convert angle from radians to degrees
        rotation_angle_degrees = np.degrees(rotation_angle_radians)
        
        # Adjust angle to be within 0 to 360 range
        rotation_angle_degrees = rotation_angle_degrees % 360
        
        return rotation_angle_degrees


    def on_recv_message(self, message):
        # self.timer.on_frame()
        if not 'msg_type' in message:
            print('expected msg_type field')
            print("message:", message)
            return

        msg_type = message['msg_type']
        if msg_type in self.fns:
            self.fns[msg_type](message)
        else:
            print('unknown message type', msg_type)

    def reset_scenario(self,road_style, waypoints: Union[str, None]):
        msg = {
            "msg_type": "regen_road",
            'road_style': road_style.__str__(),
            "wayPoints": waypoints.__str__(),

        }

        self.client.queue_message(msg)
        # display waiting screen

    
    def on_connect(self, client):
        self.client = client
        # self.timer.reset()

    def on_car_created(self, data):
        if self.rand_seed != 0:
            self.send_regen_road(0, self.rand_seed, 1.0)

    def on_aborted(self, msg):
        self.stop()

    def on_disconnect(self):
        pass

    def on_telemetry(self, data):
        imgString = data["image"]
        self.cte=data["cte"]
        self.speed=data["speed"]
        self.pos_x=data["pos_x"]
        self.pos_y=data["pos_y"]
        self.pos_z=data["pos_z"]
        self.hit=data["hit"]
        self.speed=data["speed"]
        quaternion=[data["quat_1"],data["quat_2"],data["quat_3"],data["quat_4"]]
        self.angle=self.quaternion_to_euler(quaternion)
        image = Image.open(BytesIO(base64.b64decode(imgString)))
        self.image_toplot=image.copy()
        rgb = np.asarray(image, dtype=np.float32)[:, :, :3]
        alpha = np.asarray(image, dtype=np.float32)[:, :, 3]

        # Convert the image to BGR format for display
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Create a mask from the alpha channel (convert to 3 channels)
        mask = np.repeat(alpha[:, :, np.newaxis], 3, axis=2) / 255.0

        # Perform alpha blending
        blended = (bgr * mask + [0,0,0] * (1 - mask)).astype(np.uint8)
        self.img_arr = np.asarray(blended.copy(), dtype=np.float32)

    def send_pose(self,tracked_pose_sim):
        msg = { 'msg_type' : 'tracking', 'x': tracked_pose_sim[0].__str__(), 'y':tracked_pose_sim[1].__str__(), 'angle': tracked_pose_sim[2].__str__() }
        self.client.send_now(msg)

    def reset_scenario(self,road_style, waypoints: Union[str, None]):
        msg = {
            "msg_type": "regen_road",
            'road_style': road_style.__str__(),
            "wayPoints": waypoints.__str__(),

        }

        self.client.send_now(msg)
        # display waiting screen
    def send_regen_road(self, road_style=0, rand_seed=0, turn_increment=0.0):
        msg = { 'msg_type' : 'regen_road',
            'road_style': road_style.__str__(),
            'rand_seed': rand_seed.__str__(),
            'turn_increment': turn_increment.__str__() }



def new_pose(msg):
    # Update last position and last update time
    global tracked_pose_sim
    global counter
    global prev_time
    global position_tracked
    global for_conversions

    if TRACKING and not MAPPING and position_tracked:
        return

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
    #y = math.degrees(euler[0])
    #p = math.degrees(euler[1])
    r = math.degrees(euler[2])
    #print(y,p,r)

    #orientation = [r, p, y]
    #print(orientation)
   
    #print(tracked_pose_sim)

    time_now=datetime.now()
    counter+=1
    if MAPPING and (time_now-prev_time)>timedelta(0,0,microseconds=20) or not MAPPING and TRACKING and not position_tracked:
    # if MAPPING or not MAPPING and TRACKING and not position_tracked:
        tracked_pose_sim = for_conversions.real2sim_xyp([position[0], position[1], r])
        simulator_client.msg_handler.send_pose(tracked_pose_sim)
        prev_time=time_now
        position_tracked = True
    if counter>100:
        pass
        #print(time_now-prev_time)

def new_obstacles(msg):
    global simulator_client
    for obstacle in msg.data:
        sim_msg = { "msg_type" : "obstacle",
                "name" : obstacle.name.__str__(),
                "x" : obstacle.x.__str__(),
                "y" : obstacle.y.__str__(),
                "z" : obstacle.z.__str__(),
                "angle1" : obstacle.pitch.__str__(),
                "angle2" : obstacle.roll.__str__(),
                "angle3" : obstacle.yaw.__str__() }  
        if simulator_client: 
            simulator_client.queue_message(sim_msg)
            time.sleep(0.1)


def new_reset(msg):
    global simulator_client
    global position_tracked
    print("\nRESETTING SCENARIO")
    #read initial road configuration from json file
    f = open(map_name, "r")
    map_data = json.loads(f.read())
    f.close()
    simulator_client.msg_handler.reset_scenario(0,map_data["road_definition"])
    #send initial car pose
    initial_pose = map_data["initial_pose_sim"]
    print(f"initial pose_sim in map {initial_pose}")
    time.sleep(0.1)
    simulator_client.msg_handler.send_pose(initial_pose) 
    time.sleep(0.1) #CAUTION: if this waiting is excluded we get the wrong reading from the simulator
    print(f"current pose in sim {simulator_client.msg_handler.pos_x,simulator_client.msg_handler.pos_y,simulator_client.msg_handler.pos_z}")
    position_tracked = False
    


def new_throttle_steering(msg):
    #TODO: change behaviour if youre braking to stop
    global throttle
    global steering
    global simulator_client
    global throttle_multiplier
    global going

    throttle = msg.throttle
    steering = msg.steering

    if abs(steering)>=1:
        if steering <0:
            steering = -0.99
        else:
            steering = 0.99

    if MAPPING:
        throttle = 0

    adjusted_throttle = throttle*throttle_multiplier
    if msg.stopping:
        #message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':(throttle).__str__(), 'brake': '1.0' }
        message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':'0.0', 'brake': '1.0' }
    elif msg.brake:
        message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':'0.0', 'brake': '1.0' }
    else:
        message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':adjusted_throttle.__str__(), 'brake': '0.0' }     

    if going:
        simulator_client.queue_message(message)
    else:
        # print("going is set to false, not sending actuation commands")
        pass

def new_multiplier(msg):
    global throttle_multiplier
    throttle_multiplier = msg.data


going = False
def new_going(msg):
    global going
    going = msg.data

    if not going:
        message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':'0.0', 'brake': '1.0' }
        simulator_client.queue_message(message)
        print("Received stopping command")



def simulator_node():
    print("Starting simulator node")
    global simulator_client

    handler = DonkeySimMsgHandler()
    address = (simulator_ip, simulator_port)
    simulator_client = SimClient(address, handler)

    #read initial road configuration from json file
    print("\nINITIALIZING SCENARIO")
    f = open(map_name, "r")
    map_data = json.loads(f.read())
    f.close()
    simulator_client.msg_handler.reset_scenario(0,map_data["road_definition"])
    #load obstacles
    for obstacle in map_data["obstacles"]:
        msg = { "msg_type" : "obstacle",
                "name" : obstacle[0],
                "x" : obstacle[1].__str__(),
                "y" : obstacle[2].__str__(),
                "z" : obstacle[3].__str__(),
                "angle1" : obstacle[4].__str__(),
                "angle2" : obstacle[5].__str__(),
                "angle3" : obstacle[6].__str__()}
        if simulator_client: 
            simulator_client.queue_message(msg)
            time.sleep(0.3)
            print(f"{obstacle[0]} obstacle sent to sim")
        
    #send initial car pose
    initial_pose = map_data["initial_pose_sim"]
    print(f"initial pose_sim in map {initial_pose}")
    time.sleep(0.1)
    simulator_client.msg_handler.send_pose(initial_pose) 
    time.sleep(0.1)
    print(f"current pose in sim {simulator_client.msg_handler.pos_x,simulator_client.msg_handler.pos_y,simulator_client.msg_handler.pos_z}")


    rospy.init_node('simulator_node', anonymous=True)
    rospy.Subscriber('donkey/pose', PoseStamped, new_pose)
    rospy.Subscriber("control/throttle_steering", Control, new_throttle_steering)
    rospy.Subscriber("reset", Bool, new_reset)
    rospy.Subscriber("obstacles", Obstacles, new_obstacles)
    rospy.Subscriber("/going", Bool, new_going)
    if MAPPING:
        rospy.Subscriber("throttle/multiplier", Float64, new_multiplier)
    else:
        rospy.Subscriber("throttle_sim/multiplier", Float64, new_multiplier)
    
    pub_simulator_pose = None
    pub_simulator_cte = None
    pub_sim_image = None
    pub_collision = None
    pub_simulated_speed = None
    pub_euler_position = None
    pub_image_for_model = None
    pub_sim_obstacles = None
    rate = rospy.Rate(50)


    while not rospy.is_shutdown():

        while TRACKING and not position_tracked:
            #initial pose of the car has not been sent to the simulator yet, we'll have to wait for it
            pass



        #publishing the simulator position and angle
        simulator_pose=[simulator_client.msg_handler.pos_x,simulator_client.msg_handler.pos_y,simulator_client.msg_handler.pos_z]
        angle = simulator_client.msg_handler.angle
        #print(f"angle: {angle}")
        #simulator_orientation=[0.,math.radians(angle),0.]


        #SENDING EULER POSE
        euler = SimPose("donkey",simulator_pose[0],simulator_pose[1],simulator_pose[2],
                        angle,0,0)
        if pub_euler_position is None:
            pub_euler_position = rospy.Publisher("sim/euler", SimPose,queue_size=10)
        pub_euler_position.publish(euler)


        #publishing cte
        simulator_cte = simulator_client.msg_handler.cte
        if pub_simulator_cte is None:
            pub_simulator_cte = rospy.Publisher('sim/cte', Float64, queue_size=10)
        pub_simulator_cte.publish(simulator_cte)


        #status.sim_image=client.msg_handler.image_toplot
        sim_image = simulator_client.msg_handler.image_toplot
        msg_sim_image = SensorImage()
        msg_sim_image.header.stamp = rospy.Time.now()
        msg_sim_image.height = sim_image.height
        msg_sim_image.width = sim_image.width
        msg_sim_image.encoding = "rgba8"
        msg_sim_image.is_bigendian = False
        msg_sim_image.step = 4 * sim_image.width
        msg_sim_image.data = np.array(sim_image).tobytes()
        if pub_sim_image is None:
            pub_sim_image = rospy.Publisher("sim/image", SensorImage, queue_size=10)
        pub_sim_image.publish(msg_sim_image)


        image_for_model = simulator_client.msg_handler.img_arr

        if pub_image_for_model is None:
            pub_image_for_model = rospy.Publisher("sim/image_for_model", numpy_msg(Floats), queue_size=10)
        pub_image_for_model.publish(image_for_model.flatten())
    

        #publish collision value
        #print(f"DEBUG: type of collision is {type(simulator_client.msg_handler.hit)} and its value is {simulator_client.msg_handler.hit}\n\n")
        if pub_collision is None:
            pub_collision = rospy.Publisher("collision", String, queue_size=10)
        pub_collision.publish(simulator_client.msg_handler.hit)
        

        #publish simulated speed
        #print(f"DEBUG: type of simulated_speed is {type(simulator_client.msg_handler.speed)}\n\n")
        if pub_simulated_speed is None:
            pub_simulated_speed = rospy.Publisher("sim/speed", Float64, queue_size=10)
        pub_simulated_speed.publish(simulator_client.msg_handler.speed)


        #publish simulation-only objects
        if pub_sim_obstacles is None:
            pub_sim_obstacles = rospy.Publisher("obstacles", Obstacles, queue_size=10)
        pub_sim_obstacles.publish(list(map(lambda o:SimPose(o[0],o[1],o[2],o[3],o[4],o[5],o[6]),map_data["obstacles"])))
        print(map_data["obstacles"])


        rate.sleep()
    print("[SIMULATION CLIENT]: QUIT")




if __name__ == '__main__':
    try:
        simulator_node()
    except rospy.ROSInterruptException:
        pass