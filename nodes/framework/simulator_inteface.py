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

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped

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

going = False

simulated_speed=0.0

tracked_pose_sim=[1.,0.,1.]

throttle = 0
steering = 0
throttle_multiplier = rospy.get_param("default_throttle_multiplier")

counter=0
prev_time=datetime.now()

started=False


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
        self.image_tomix=None
        self.image_depth=None
        self.segmentation_mask=None
        self.image_toobs=None
        self.image_toroad=None
        self.img_arr=None
        self.client = None
        self.rand_seed=0
        self.hit=None
        self.speed=0.0
        self.tb = TransformBroadcaster()

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

    def publish_map_to_base_link_sim(self,x,y,z,q1,q2,q3,q4):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link_sim"
        
        # Translation: x=0, y=0, z=0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        t.transform.rotation.x = q1
        t.transform.rotation.y = q2
        t.transform.rotation.z = q3
        t.transform.rotation.w = q4
        
        # Publish the transform using the given TransformBroadcaster
        self.tb.sendTransform(t)
        
    # pose_pub = None
    def on_telemetry(self, data):
        imgString = data["image"]
        imgString_road = data["road_image"]
        imgString_obstacle = data["obstacle_image"]
        imgString_mixing = data["mixed_image"]
        lidString = data["depth"]
        self.cte=data["cte"]
        self.speed=data["speed"]
        self.pos_x=data["pos_x"]
        self.pos_y=data["pos_y"]
        self.pos_z=data["pos_z"]
        self.hit=data["hit"]
        self.speed=data["speed"]
        quaternion=[data["quat_1"],data["quat_2"],data["quat_3"],data["quat_4"]]
        self.angle=self.quaternion_to_euler(quaternion)

        x_real, y_real, angle_real_deg = for_conversions.sim2real_xyp([
            self.pos_x,
            self.pos_z,
            self.angle
        ])

        # 3) turn that real yaw back into a quaternion
        yaw_rad = math.radians(angle_real_deg+90)
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)

        # if pose_pub is None:
        #     pose_pub = rospy.Publisher('sim/pose', PoseStamped, queue_size=1)

        
        if started:
            self.publish_map_to_base_link_sim(x_real,y_real,data["pos_y"],q[0],q[1],q[2],q[3])
            # print("here")
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

        depth = Image.open(BytesIO(base64.b64decode(lidString)))
        import matplotlib.pyplot as plt

        # If depth is a PIL Image:
        # plt.imshow(depth)
        # plt.axis('off')   # hide the axes
        # plt.show()
        image_road = Image.open(BytesIO(base64.b64decode(imgString_road)))
        image_obstacle = Image.open(BytesIO(base64.b64decode(imgString_obstacle)))
        image_mixing = Image.open(BytesIO(base64.b64decode(imgString_mixing)))

        road_rgb = np.asarray(image_road, dtype=np.float32)[:, :, :3]
        obs_rgb = np.asarray(image_obstacle, dtype=np.float32)[:, :, :3]
        road_alpha = np.asarray(image_road, dtype=np.float32)[:, :, 3]
        obs_alpha = np.asarray(image_obstacle, dtype=np.float32)[:, :, 3]
        road_mask = np.repeat(road_alpha[:, :, np.newaxis], 3, axis=2) / 255.0
        obs_mask = np.repeat(obs_alpha[:, :, np.newaxis], 3, axis=2) / 255.0
        road_blended = (road_rgb * road_mask + [0,0,0] * (1 - road_mask)).astype(np.uint8)
        obs_blended = (obs_rgb * obs_mask + [0,0,0] * (1 - obs_mask)).astype(np.uint8)
        white_mask_road = (np.all(road_blended == [255, 255, 255], axis=-1))
        non_black_mask_road = ~(np.all(road_blended == [0, 0, 0], axis=-1))
        non_black_mask_obs = ~(np.all(obs_blended == [0, 0, 0], axis=-1))
        other_mask_road = non_black_mask_road & ~white_mask_road

        
        road_blended[other_mask_road] = [255, 0, 0]
        road_blended[non_black_mask_road] = [0, 0, 255]
        road_blended[white_mask_road] = [255, 255, 255]
        road_blended[non_black_mask_obs] = [0, 255, 0]
        



        alpha_channel = 255 * np.ones(road_blended.shape[:2], dtype=np.uint8)

        # Stack the RGB channels and the alpha channel to form an RGBA image
        rgba_image = np.dstack((road_blended, alpha_channel))

        # Convert the numpy array back to a PIL Image in RGBA mode
        processed_image = Image.fromarray(rgba_image, 'RGBA')

        

        self.image_tomix=image_mixing.copy()
        self.image_toobs=image_obstacle.copy()
        self.image_toroad=image_road.copy()
        self.segmentation_mask = processed_image.copy()


        self.image_depth=depth.copy()


    def send_pose(self,tracked_pose_sim):
        # if len(tracked_pose_sim)<5:
        if len(tracked_pose_sim)<5:
            msg = { 'msg_type' : 'tracking', 'x': tracked_pose_sim[0].__str__(), 'y':tracked_pose_sim[1].__str__(), 'angle1': (0.0000).__str__(), 'angle2': tracked_pose_sim[2].__str__(), 'angle3': (0.0000).__str__() }
        else:
            # msg = { 'msg_type' : 'tracking', 'x': tracked_pose_sim[0].__str__(), 'y':tracked_pose_sim[1].__str__(), 'angle1': tracked_pose_sim[2].__str__(), 'angle2': tracked_pose_sim[3].__str__(), 'angle3': tracked_pose_sim[4].__str__() }
            msg = { 'msg_type' : 'tracking', 'x': tracked_pose_sim[0].__str__(), 'y':tracked_pose_sim[1].__str__(), 'angle1': (0.0000).__str__(), 'angle2': tracked_pose_sim[3].__str__(), 'angle3': (0.0000).__str__() }
        self.client.send_now(msg)

    def send_obstacle(self, sim_msg):
        self.client.send_now(sim_msg)

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
    r = math.degrees(euler[2])-90
    r1 = math.degrees(euler[0])-90
    r3 = math.degrees(euler[1])-90
    #print(y,p,r)

    #orientation = [r, p, y]
    #print(orientation)
   
    #print(tracked_pose_sim)
    100

    time_now=datetime.now()
    counter+=1
    if MAPPING and (time_now-prev_time)>timedelta(0,0,microseconds=17000)or not MAPPING and TRACKING and not position_tracked:
    # if MAPPING or not MAPPING and TRACKING and not position_tracked:
        # tracked_pose_sim = for_conversions.real2sim_xyp([position[0], position[1], r])
        tracked_pose_sim = for_conversions.real2sim_xyp3([position[0], position[1], r1,r,r3])
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
            # simulator_client.queue_message(sim_msg)
            simulator_client.msg_handler.send_obstacle(sim_msg)
            # time.sleep(0.1)


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
    global going

    #TODO: change behaviour if youre braking to stop
    global throttle
    global steering
    global simulator_client
    global throttle_multiplier
    

    #throttle = msg.throttle*throttle_multiplier
    throttle = msg.throttle
    steering = msg.steering

    #throttle = 0.3
    #TODO: add throttle multiplier factor

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
    elif going:
        message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':adjusted_throttle.__str__(), 'brake': '0.0' }     
    else:
        message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':'0.0', 'brake': '0.0' }     
    
    simulator_client.queue_message(message)

def new_multiplier(msg):
    global throttle_multiplier
    throttle_multiplier = msg.data
    print(f"Sim recieved a multiplier change {throttle_multiplier}")

def new_going(msg):
    global going
    going = msg.data
    print(msg.data)
    print(f"received new going command: {going}")




def simulator_node():
    print("Starting simulator node")
    global simulator_client
    global started
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
    rospy.Subscriber("/obstacles", Obstacles, new_obstacles)
    rospy.Subscriber("/going", Bool, new_going)
    rospy.Subscriber("throttle/multiplier", Float64, new_multiplier)
    
    pub_simulator_pose = None
    pub_simulator_cte = None
    pub_sim_image = None
    pub_mix_image = None
    pub_d_image = None
    pub_road_image = None
    pub_obs_image = None
    pub_seg_image = None
    pub_collision = None
    pub_simulated_speed = None
    pub_euler_position = None
    pub_image_for_model = None
    pub_sim_obstacles = None
    rate = rospy.Rate(20)
    started=True


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


        if simulator_client.msg_handler.image_toplot is not None:
            sim_image = simulator_client.msg_handler.image_toplot
            msg_sim_image = SensorImage()
            msg_sim_image.header.stamp = rospy.Time.now()
            msg_sim_image.height = sim_image.height
            msg_sim_image.width = sim_image.width
            msg_sim_image.encoding = "rgba8"
            msg_sim_image.is_bigendian = False
            msg_sim_image.step = 4 * sim_image.width
            msg_sim_image.data = np.array(sim_image).tobytes()

        if simulator_client.msg_handler.image_tomix is not None:
            mix_image = simulator_client.msg_handler.image_tomix
            msg_mix_image = SensorImage()
            msg_mix_image.header.stamp = rospy.Time.now()
            msg_mix_image.height = mix_image.height
            msg_mix_image.width = mix_image.width
            msg_mix_image.encoding = "rgba8"
            msg_mix_image.is_bigendian = False
            msg_mix_image.step = 4 * mix_image.width
            msg_mix_image.data = np.array(mix_image).tobytes()

        if simulator_client.msg_handler.image_depth is not None:
            d_image = simulator_client.msg_handler.image_depth
            msg_d_image = SensorImage()
            msg_d_image.header.stamp = rospy.Time.now()
            msg_d_image.height = d_image.height
            msg_d_image.width = d_image.width
            msg_d_image.encoding = "rgba8"
            msg_d_image.is_bigendian = False
            msg_d_image.step = 4 * d_image.width
            msg_d_image.data = np.array(d_image).tobytes()

        if simulator_client.msg_handler.segmentation_mask is not None:
            seg_image = simulator_client.msg_handler.segmentation_mask
            msg_seg_image = SensorImage()
            msg_seg_image.header.stamp = rospy.Time.now()
            msg_seg_image.height = seg_image.height
            msg_seg_image.width = seg_image.width
            msg_seg_image.encoding = "rgba8"
            msg_seg_image.is_bigendian = False
            msg_seg_image.step = 4 * seg_image.width
            msg_seg_image.data = np.array(seg_image).tobytes()
        if simulator_client.msg_handler.image_toobs is not None:
            obs_image = simulator_client.msg_handler.image_toobs
            msg_obs_image = SensorImage()
            msg_obs_image.header.stamp = rospy.Time.now()
            msg_obs_image.height = obs_image.height
            msg_obs_image.width = obs_image.width
            msg_obs_image.encoding = "rgba8"
            msg_obs_image.is_bigendian = False
            msg_obs_image.step = 4 * obs_image.width
            msg_obs_image.data = np.array(obs_image).tobytes()
        if simulator_client.msg_handler.image_toroad is not None:
            road_image = simulator_client.msg_handler.image_toroad
            msg_road_image = SensorImage()
            msg_road_image.header.stamp = rospy.Time.now()
            msg_road_image.height = road_image.height
            msg_road_image.width = road_image.width
            msg_road_image.encoding = "rgba8"
            msg_road_image.is_bigendian = False
            msg_road_image.step = 4 * road_image.width
            msg_road_image.data = np.array(road_image).tobytes()





            if pub_sim_image is None:
                pub_sim_image = rospy.Publisher("sim/image", SensorImage, queue_size=1)
            pub_sim_image.publish(msg_sim_image)
            if pub_mix_image is None:
                pub_mix_image = rospy.Publisher("sim/image_for_mixing", SensorImage, queue_size=1)
            pub_mix_image.publish(msg_mix_image)

            if pub_d_image is None:
                pub_d_image = rospy.Publisher("sim/image_depth", SensorImage, queue_size=1)
            pub_d_image.publish(msg_d_image)
            if pub_road_image is None:
                pub_road_image = rospy.Publisher("sim/image_road", SensorImage, queue_size=1)
            pub_road_image.publish(msg_road_image)
            if pub_obs_image is None:
                pub_obs_image = rospy.Publisher("sim/image_obs", SensorImage, queue_size=1)
            pub_obs_image.publish(msg_obs_image)
            if pub_seg_image is None:
                pub_seg_image = rospy.Publisher("sim/image_seg", SensorImage, queue_size=1)
            pub_seg_image.publish(msg_seg_image)
        



            image_for_model = simulator_client.msg_handler.img_arr

            if pub_image_for_model is None:
                pub_image_for_model = rospy.Publisher("sim/image_for_model", numpy_msg(Floats), queue_size=10)
            pub_image_for_model.publish(image_for_model.flatten())
        

            #publish collision value
            # print(f"DEBUG: type of collision is {type(simulator_client.msg_handler.hit)} and its value is {simulator_client.msg_handler.hit}\n\n")
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

            


        rate.sleep()
    print("[SIMULATION CLIENT]: QUIT")




if __name__ == '__main__':
    try:
        simulator_node()
    except rospy.ROSInterruptException:
        pass
