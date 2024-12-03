import os
import json
import time
from io import BytesIO
import base64

from typing import Any, Union, List, Dict
import cv2

from PIL import Image
import numpy as np
from gym_donkeycar.core.fps import FPSTimer
from gym_donkeycar.core.message import IMesgHandler
import torchvision.transforms as T

from gym_donkeycar.core.sim_client import SimClient
import math
import numpy as np
import struct
import math
import warnings
import numpy as np
from scipy.interpolate import CubicSpline
import json
import cv2
import warnings
from PIL import Image
import numpy as np
import time
import base64
import datetime
from io import BytesIO
from typing import Any, Union, List, Dict
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.interpolate import CubicSpline

# DO NOT CHANGE
STEERING = 0
THROTTLE = 1
SIZE_FACTOR=7.33
X_MAP_SHIFT=48
Y_MAP_SHIFT=50
ANGLE_SHIFT=0
SCALE_1=2.08
SCALE_2=3.2
WAYPOINTS_TRESHOLD = 0.4 * SIZE_FACTOR
ANGLE_TRESHOLD=5
OFFSET=3

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
class Road_shapes_utils():
    def __init__(self,status,scale_1, scale_2, x_map_shift, y_map_shift, size_factor,angle_shift):
        self.status, self.scale_1, self.scale_2, self.x_map_shift, self.y_map_shift, self.size_factor,self.angle_shift=status, scale_1, scale_2, x_map_shift, y_map_shift, size_factor,angle_shift


    def generate_road_margins(self,road_points, offset):
        left_margins = []
        right_margins = []

        num_points = len(road_points)

        # Calculate the direction vectors for each road segment
        direction_vectors = []
        for i in range(num_points - 1):
            dx = road_points[i + 1][0] - road_points[i][0]
            dy = road_points[i + 1][1] - road_points[i][1]
            mag = np.sqrt(dx ** 2 + dy ** 2)
            direction_vectors.append((dx / mag, dy / mag))

        # Average neighboring direction vectors to get smoother normals
        averaged_directions = []
        for i in range(num_points - 1):
            if i == 0:
                averaged_directions.append(direction_vectors[0])
            elif i == num_points - 2:
                averaged_directions.append(direction_vectors[-1])
            else:
                averaged_directions.append(((direction_vectors[i][0] + direction_vectors[i - 1][0]) / 2,
                                            (direction_vectors[i][1] + direction_vectors[i - 1][1]) / 2))

        # Calculate normals and generate margins
        for i in range(num_points - 1):
            dx, dy = averaged_directions[i]
            nx = -dy
            ny = dx

            left_x = road_points[i][0] + offset * nx
            left_y = road_points[i][1] + offset * ny
            right_x = road_points[i][0] - offset * nx
            right_y = road_points[i][1] - offset * ny

            left_margins.append([left_x, left_y])
            right_margins.append([right_x, right_y])

        return left_margins, right_margins

    def create_non_closed_road(self,intermediate_points,start_angle,start_offset):
            for_conversions=For_convertion_utils(self.size_factor,self.x_map_shift,self.y_map_shift,self.angle_shift)
            starting_x_real, starting_y_real, starting_angle_real = intermediate_points[0][0],intermediate_points[0][1], start_angle
            if start_offset:
                initial_pose_real=[starting_x_real+start_offset[0], starting_y_real+start_offset[1], starting_angle_real]
            else:
                initial_pose_real=[starting_x_real, starting_y_real, starting_angle_real]
            initial_pose_sim=for_conversions.real2sim_xyp(initial_pose_real)
            scaled_vertices = []
            for point in intermediate_points:
                x,y,_= for_conversions.real2sim_xyp([point[0],point[1],0])
                scaled_vertices.append([x, y])

            # Interpolate a smooth road using cubic spline
            scaled_vertices = np.array(scaled_vertices)
            cs = CubicSpline(np.arange(len(scaled_vertices)), scaled_vertices)

            # Generate interpolated points along the smooth road
            num_interpolation_points = len(scaled_vertices) * 10  # Adjust as needed
            interpolated_points = cs(np.linspace(0, len(scaled_vertices) - 1, num_interpolation_points))

            # Format the road segments
            road_segments = []
            road_points =[]
            for point in interpolated_points:
                x, z = point
                road_points.append([x,z])
                y = 0.6  # Assuming constant y value
                road_segment = f"{x:.2f},{y:.2f},{z:.2f}"
                road_segments.append(road_segment)
            self.status.real_initial_waypoint=[initial_pose_real[0],initial_pose_real[1]]
            self.status.real_initial_angle=initial_pose_real[2]
            self.status.sim_initial_waypoint=[initial_pose_sim[0],initial_pose_sim[1]]
            self.status.sim_initial_angle=initial_pose_sim[2]
            return initial_pose_sim,initial_pose_real,road_points, '@'.join(road_segments)

    def create_closed_road(self,intermediate_points,start_angle,start_offset):
            for_conversions=For_convertion_utils(self.size_factor,self.x_map_shift,self.y_map_shift,self.angle_shift)
            starting_x_real, starting_y_real, starting_angle_real = intermediate_points[0][0],intermediate_points[0][1], start_angle
            if start_offset:
                initial_pose_real=[starting_x_real+start_offset[0], starting_y_real+start_offset[1], starting_angle_real]
            else:
                initial_pose_real=[starting_x_real, starting_y_real, starting_angle_real]
            initial_pose_sim=for_conversions.real2sim_xyp(initial_pose_real)
            scaled_vertices = []
            for point in intermediate_points:
                x,y,_= for_conversions.real2sim_xyp([point[0],point[1],0])
                scaled_vertices.append([x, y])
            scaled_vertices.append(scaled_vertices[0])
            # Interpolate a smooth road using cubic spline
            scaled_vertices = np.array(scaled_vertices)
            cs = CubicSpline(np.arange(len(scaled_vertices)), scaled_vertices)

            # Generate interpolated points along the smooth road
            num_interpolation_points = len(scaled_vertices) * 10  # Adjust as needed
            interpolated_points = cs(np.linspace(0, len(scaled_vertices) - 1, num_interpolation_points))


            # Format the road segments
            road_segments = []
            road_points =[]
            for point in interpolated_points:
                x, z = point
                road_points.append([x,z])
                y = 0.6  # Assuming constant y value
                road_segment = f"{x:.2f},{y:.2f},{z:.2f}"
                road_segments.append(road_segment)
            self.status.real_initial_waypoint=[initial_pose_real[0],initial_pose_real[1]]
            self.status.real_initial_angle=initial_pose_real[2]
            self.status.sim_initial_waypoint=[initial_pose_sim[0],initial_pose_sim[1]]
            self.status.sim_initial_angle=initial_pose_sim[2]
            return initial_pose_sim,initial_pose_real,road_points, '@'.join(road_segments)

    def create_circular_road(self,radius_real):
        radius_sim=radius_real*self.size_factor

        center_x_sim=self.x_map_shift
        center_y_sim=self.y_map_shift

        road_definition,road_points = self.create_circle(center_x_sim,center_y_sim,radius_sim, 100)
        for_conversions=For_convertion_utils(self.size_factor,self.x_map_shift,self.y_map_shift,self.angle_shift)

        initial_pose_sim=self.x_map_shift+radius_sim,self.y_map_shift,0
        initial_pose_real=for_conversions.sim2real_xyp(initial_pose_sim)
        self.status.real_initial_waypoint=[initial_pose_real[0],initial_pose_real[1]]
        self.status.real_initial_angle=initial_pose_real[2]

        self.status.sim_initial_waypoint=[initial_pose_sim[0],initial_pose_sim[1]]
        self.status.sim_initial_angle=initial_pose_sim[2]
        return initial_pose_sim,initial_pose_real,road_points, road_definition


    def create_circle(self,start_x, start_z, radius, num_segments):
        road_segments = []
        road_segments_complete = []
        interpolated_points = []
        theta_values = np.linspace(0, 2*np.pi, num_segments)
        for theta in theta_values:
            x = start_x + radius * np.cos(theta)
            z = start_z + radius * np.sin(theta)
            interpolated_points.append([x,z])
            y = 0.6
            road_segment = f"{x:.2f},{y:.2f},{z:.2f}"
            road_segments.append(road_segment)
        # road_segments_complete.append(road_segments[-1])
        for road_segment in road_segments:
            road_segments_complete.append(road_segment)
        return '@'.join(road_segments_complete),interpolated_points


class System_status_utils():
    def __init__(self):
        self.simulator_pose=[0.,0.,0.]
        self.simulator_orientation=[0.,0.,0.]

        self.tracked_pose_sim=[0.,0.,0.]

        self.real_pose=[0.,0.,0.]
        self.real_orientation=[0.,0.,0.]

        self.real_initial_waypoint=[0.,0.]
        self.real_initial_angle=0.

        self.sim_initial_waypoint=[0.,0.]
        self.sim_initial_angle=0.

        self.current_waypoint_index=0
        self.waypoint_size=0

        self.real_target_waypoint=None
        self.sim_target_waypoint=None

        self.real_waypoint_list=None
        self.sim_waypoint_list=None
        self.sim_waypoint_list_center=None
        self.sim_waypoint_list_right=None
        self.sim_waypoint_list_left=None


        self.real_image=None
        self.sim_image=None
        self.blended_image=None

        self.simulator_cte=0.0

        self.sim_image_for_model=None

        self.real_throttle_multiplier=0.3
        self.real_steering_multiplier=1

        self.state = "stopped"


        self.is_running=True

        self.keyboard_client=False
        self.real_camera_client=False
        self.tracking_client=False
        self.simulator_client=False
        self.real_car_controls=False
        self.feed_server=False
        self.simulator_client_object=None

        self.for_conversions=None

        self.collision=None

        self.waypoint_state="going_to_waypoint"
        self.waypoint_distance=0.0

        self.car_controls=[0.0,0.0,False]


        self.current_lane="left"


        self.status_obstacle_list=[]

        self.left_margins, self.right_margins = None,None

        self.real_speed=0.0
        self.simulated_speed=0.0

        self.speed_threshold_min=0.4

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


class Waypoint_control_utils():
        def __init__(self,treshold,angle_treshold):
            self.waypoint_treshold=treshold
            self.angle_treshold=angle_treshold


        def angle_difference(self,a1, a2):
            diff = a2 - a1
            while diff <= -180:
                diff += 360
            while diff > 180:
                diff -= 360

            return diff

        def angle_extraction(self, x1, y1, z1, x2, y2, z2):

            # Calculate the distances between points
            dx = x2 - x1
            dy = y2 - y1
            dz = z2 - z1

            # Calculate the angles on each axis
            angle_x_axis = math.atan2(dy, dz)
            angle_y_axis = math.atan2(dx, dz)
            angle_z_axis = math.atan2(dy, dx)

            # Convert angles from radians to degrees
            angle_x_axis_degrees = math.degrees(angle_x_axis)
            angle_y_axis_degrees = math.degrees(angle_y_axis)
            angle_z_axis_degrees = math.degrees(angle_z_axis)
            return angle_x_axis_degrees, angle_y_axis_degrees, angle_z_axis_degrees


        def calculate_control(self, x_target, y_target, simulator_pose, simulator_orientation):
            x_cur, _, y_cur = simulator_pose
            angle_cur = simulator_orientation
            distance = math.sqrt((x_target - x_cur)**2 + (y_target - y_cur)**2)
            _, angle_y_axis_degrees, _=self.angle_extraction(x_cur, 0.0, y_cur, x_target, 0.0, y_target)
            angle_difference=self.angle_difference(angle_cur,angle_y_axis_degrees)
            steering = (math.radians(angle_difference) / math.pi)*5
            steering=min(1,max(-1,steering))
            throttle=1

            return steering, throttle, distance, angle_difference

        def calculate_distance(self, x_target, y_target, simulator_pose):
            x_cur, _, y_cur = simulator_pose
            distance = math.sqrt((x_target - x_cur)**2 + (y_target - y_cur)**2)
            return distance

        def calculate_angle(self, x_target, y_target, simulator_pose, simulator_orientation):
            x_cur, _, y_cur = simulator_pose
            _, angle_cur, _ = simulator_orientation
            _, angle_y_axis_degrees, _=self.angle_extraction(x_cur, 0.0, y_cur, x_target, 0.0, y_target)
            angle_difference=self.angle_difference(angle_cur,angle_y_axis_degrees)
            return angle_difference

        def go_to_waypoint_step(self, simulator_pose, simulator_orientation):
            x,y=self.status.sim_target_waypoint
            steering, throttle, distance, _ = self.calculate_control(x,y, simulator_pose, simulator_orientation)
            if distance <= self.waypoint_treshold and self.status.waypoint_state!="reached_waypoint":
                self.status.car_controls=[throttle,steering,True]
                self.status.set_waypoint_state("reached_waypoint")
                self.status.set_state("stopping")
                print(f"[CAR WAYPOINT CONTROL] Reached waypont (SIM): {x}, {y}")
            elif self.status.waypoint_state!="reached_waypoint":
                self.status.car_controls=[throttle,steering,False]
                time.sleep(0.1)
            else:
                self.status.car_controls=[0,0,False]
                time.sleep(0.1)


class DonkeySimMsgHandler(IMesgHandler):
    STEERING = 0
    THROTTLE = 1
    SIZE_FACTOR=7.33
    WAYPOINT_THRESHOLD = 0.3*SIZE_FACTOR
    OBS_THRESHOLD = 10
    def __init__(self,model,translate_model,waypoints,driving_lane,obstacles,speed_threshold_min,repeat=False,real_model=False, show_preview=False, use_masks_sim=False):
        self.last_display_time = 0
        self.last_image = None
        self.car_loaded = False
        self.pos_x = 0.
        self.pos_y = 0.
        self.pos_z = 0.
        self.cte=0.
        self.xte=0.
        self.time=0.
        self.lap=0.
        self.speed=0.
        self.show_preview = show_preview
        self.hit='none'
        self.sector=0.
        self.angle=0.0
        self.timer = FPSTimer()
        self.model=model
        self.translate_model=translate_model
        self.img_arr=None
        self.waypoints=waypoints
        self.current_lane=driving_lane
        self.waypoint_controller = Waypoint_control_utils(self.WAYPOINT_THRESHOLD, 0)
        self.state="driving"
        self.prev_steering=0.0
        self.current_waypoint_index=1
        self.obstacles=obstacles
        self.pid_controller = PIDController(kp=0.7, ki=0.000, kd=0.00)
        self.speed_threshold_min=speed_threshold_min
        self.repeat=repeat
        self.controls=[]
        self.changing=None
        self.real_model=real_model
        self.use_masks_sim=use_masks_sim
        self.segmentation_mask=None

    def reset_scenario(self,road_style, waypoints: Union[str, None]):
        msg = {
            "msg_type": "regen_road",
            'road_style': road_style.__str__(),
            "wayPoints": waypoints.__str__(),

        }
        self.client.queue_message(msg)

    def quaternion_to_euler(self, quaternion):
        w, x, y, z = quaternion
        rotation_angle_radians = np.arctan2(2 * (w * y + x * z), 1 - 2 * (y ** 2 + z ** 2))
        rotation_angle_degrees = np.degrees(rotation_angle_radians)
        rotation_angle_degrees = rotation_angle_degrees % 360

        return rotation_angle_degrees


    def on_recv_message(self, json_packet):
        if json_packet['msg_type'] == "need_car_config":
            self.send_config()

        if json_packet['msg_type'] == "car_loaded":
            self.car_loaded = True

        if json_packet['msg_type'] == "telemetry":
            imgString = json_packet["image"]
            image = Image.open(BytesIO(base64.b64decode(imgString)))
            rgb = np.asarray(image, dtype=np.float32)[:, :, :3]
            alpha = np.asarray(image, dtype=np.float32)[:, :, 3]
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            mask = np.repeat(alpha[:, :, np.newaxis], 3, axis=2) / 255.0
            blended = (rgb * mask + [0,0,0] * (1 - mask)).astype(np.uint8)
            self.image_toplot=image.copy()
            self.img_arr = np.asarray(blended.copy(), dtype=np.float32)

            if self.use_masks_sim:
                imgStringRoad = json_packet["road"]
                imgStringObstacles = json_packet["obstacles"]
                road_image = Image.open(BytesIO(base64.b64decode(imgStringRoad)))
                obs_image = Image.open(BytesIO(base64.b64decode(imgStringObstacles)))
                road_rgb = np.asarray(road_image, dtype=np.float32)[:, :, :3]
                obs_rgb = np.asarray(obs_image, dtype=np.float32)[:, :, :3]
                road_alpha = np.asarray(road_image, dtype=np.float32)[:, :, 3]
                obs_alpha = np.asarray(obs_image, dtype=np.float32)[:, :, 3]
                road_mask = np.repeat(road_alpha[:, :, np.newaxis], 3, axis=2) / 255.0
                obs_mask = np.repeat(obs_alpha[:, :, np.newaxis], 3, axis=2) / 255.0
                road_blended = (road_rgb * road_mask + [0,0,0] * (1 - road_mask)).astype(np.uint8)
                obs_blended = (obs_rgb * obs_mask + [0,0,0] * (1 - obs_mask)).astype(np.uint8)
                white_mask_road = (np.all(road_blended == [255, 255, 255], axis=-1))
                non_black_mask_road = ~(np.all(road_blended == [0, 0, 0], axis=-1))
                non_black_mask_obs = ~(np.all(obs_blended == [0, 0, 0], axis=-1))
                other_mask_road = non_black_mask_road & ~white_mask_road

                road_blended[white_mask_road] = [255, 255, 255]
                road_blended[other_mask_road] = [255, 0, 0]
                road_blended[non_black_mask_obs] = [0, 255, 0]

                # Convert the array back to an image
                processed_image = Image.fromarray(road_blended)
                self.segmentation_mask = processed_image.copy()


                



            self.pos_x=json_packet["pos_x"]
            self.pos_y=json_packet["pos_y"]
            self.pos_z=json_packet["pos_z"]
            self.cte=json_packet["cte"]
            self.time=json_packet["time"]
            self.lap=json_packet["lap"]
            self.speed=json_packet["speed"]
            self.hit=json_packet["hit"]
            self.sector=json_packet["sector"]

            quaternion=[json_packet["quat_1"],json_packet["quat_2"],json_packet["quat_3"],json_packet["quat_4"]]
            self.angle=self.quaternion_to_euler(quaternion)

            del json_packet["image"]

    def predict(self, image_array):
        outputs = self.model.predict(image_array)
        self.parse_outputs(outputs)

    def parse_outputs(self, outputs):
        res = []
        # Expects the model with final Dense(2) with steering and throttle
        for i in range(outputs.shape[1]):
            res.append(outputs[0][i])

        self.on_parsed_outputs(res)

    def on_parsed_outputs(self, outputs):
        self.outputs = outputs
        self.steering_angle = 0.0
        if self.real_model:
            outputs[self.THROTTLE]=outputs[self.THROTTLE]/3
            outputs[self.STEERING]=outputs[self.STEERING]
        self.send_controls(outputs[self.STEERING], outputs[self.THROTTLE])

    def send_config(self):
        '''
        send three config messages to setup car, racer, and camera
        '''
        racer_name = "Your Name"
        car_name = "Car"
        bio = "I race robots."
        country = "Neverland"
        guid = "some random constant string"

        msg = {'msg_type': 'racer_info',
            'racer_name': racer_name,
            'car_name' : car_name,
            'bio' : bio,
            'country' : country,
            'guid' : guid }
        self.client.send_now(json.dumps(msg))
        time.sleep(0.2)

        msg = '{ "msg_type" : "car_config", "body_style" : "car01", "body_r" : "255", "body_g" : "0", "body_b" : "255", "car_name" : "%s", "font_size" : "100" }' % (car_name)
        self.client.send_now(msg)

        time.sleep(0.2)
        msg = '{ "msg_type" : "cam_config", "fov" : "150", "fish_eye_x" : "1.0", "fish_eye_y" : "1.0", "img_w" : "255", "img_h" : "255", "img_d" : "1", "img_enc" : "JPG", "offset_x" : "0.0", "offset_y" : "3.0", "offset_z" : "0.0", "rot_x" : "90.0" }'
        self.client.send_now(msg)
        time.sleep(0.2)

    def send_controls(self, steering, throttle,brake=0.0):
        msg = { "msg_type" : "control",
                "steering" : steering.__str__(),
                "throttle" : throttle.__str__(),
                "brake" : brake.__str__() }
        self.client.send(json.dumps(msg))

    def send_obstacle(self, name, position,rotation):
        x,y,z=position
        pitch,yaw,roll=rotation
        msg = { "msg_type" : "obstacle",
                "name" : name.__str__(),
                "x" : x.__str__(),
                "y" : y.__str__(),
                "z" : z.__str__(),
                "angle1" : yaw.__str__(),
                "angle2" : pitch.__str__(),
                "angle3" : roll.__str__() }
        self.client.send(json.dumps(msg))
        time.sleep(0.01)

    def update_with_model(self):
        if self.img_arr is not None:
            original_size = (self.img_arr.shape[1], self.img_arr.shape[0])

            plottedimg = np.array(self.image_toplot)
            #plottedimg = cv2.resize(plottedimg, (256,256))  # Resize to 256x256 pixels
            plottedimg = cv2.cvtColor(plottedimg, cv2.COLOR_BGR2RGB)
            plottedimg = torch.from_numpy(plottedimg).float().permute(2, 0, 1) / 255.0  # Normalize to [0, 1] and convert to tensor

            transform = T.Compose([
                T.ToPILImage(),
                T.Resize((256, 256)),  # Resize to the model’s expected input size
                T.ToTensor(),
                T.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))  # Normalize to [-1, 1] if trained this way
            ])

            plottedimg = transform(plottedimg).unsqueeze(0).to(self.translate_model.device)  # Add batch dimension and move to model's device

            input_dict = {
                'A': plottedimg,
                'B': None,
                'A_paths': '',
                'B_paths': ''
            }

            # Set the input and perform translation
            self.translate_model.set_input(input_dict)
            self.translate_model.test()

            output_image = self.translate_model.fake_B.detach().cpu().squeeze(0).permute(1, 2, 0).numpy()
            output_image = (output_image + 1) / 2.0
            output_image = (output_image * 255).astype(np.uint8)
            output_image = Image.fromarray(output_image)
            output_image = output_image.resize(original_size, Image.BICUBIC)
            output_image = np.array(output_image)

            image = output_image

            if self.show_preview:
                current_time = time.time()
                if current_time - self.last_display_time >= 1:
                    self.last_display_time = current_time

                    plt.figure('Current View-Window')
                    plt.imshow(self.image_toplot)
                    plt.axis('off')
                    plt.draw()
                    plt.pause(0.001)

                    plt.figure('Current Transform-Window')
                    plt.imshow(output_image)
                    plt.axis('off')
                    plt.draw()
                    plt.pause(0.001)

                    if self.use_masks_sim:
                        plt.figure('Current Road-mask')
                        plt.imshow(self.segmentation_mask)
                        plt.axis('off')
                        plt.draw()
                        plt.pause(0.001)

            image=image[None, ...]

            self.predict(image)
            self.img_arr = None

    def update_with_waypoints(self):
        if self.state!='stopped':
            sim_pose=self.pos_x,self.pos_y,self.pos_z
            if self.obstacles!=None and self.changing==None:
                for i,name in enumerate(self.obstacles["obstacle_names"]):
                    obstacle_pose=self.obstacles['obstacle_poses'][i]
                    closest_lane = None
                    min_distance = float('inf')
                    left_waypoints = self.waypoints["left"]
                    right_waypoints = self.waypoints["right"]
                    for lane, waypoints in [("left", left_waypoints), ("right", right_waypoints)]:
                        for waypoint in waypoints:
                            distance = math.sqrt((obstacle_pose[0] - waypoint[0]) ** 2 + (obstacle_pose[1] - waypoint[1]) ** 2)
                            if distance < min_distance:
                                min_distance = distance
                                closest_lane = lane
                    distance = self.waypoint_controller.calculate_distance(obstacle_pose[0],obstacle_pose[1],sim_pose)
                    if distance < self.OBS_THRESHOLD and closest_lane and closest_lane == self.current_lane:
                        print(f"\nFound {name} in pose: {obstacle_pose}, Lane: {closest_lane} at {min_distance}, i am in {self.current_lane}")
                        if closest_lane == "right":
                            self.current_lane = "center"
                            self.changing=['left',0]
                        elif closest_lane == "left":
                            self.current_lane = "center"
                            self.changing=['right',0]
            elif self.changing!=None:
                if self.changing[1]==12:
                    self.current_lane = self.changing[0]
                    self.changing=None
                    print("changed")
                else:
                    self.changing[1]=self.changing[1]+1

            current_waypoint = self.waypoints[self.current_lane][self.current_waypoint_index]
            x, y = current_waypoint
            sim_orientation= self.angle
            distance = self.waypoint_controller.calculate_distance(x,y,sim_pose)
            if distance <= self.WAYPOINT_THRESHOLD:
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints[self.current_lane]):
                    current_waypoint = self.waypoints[self.current_lane][self.current_waypoint_index]
                    # print("Going to: ", current_waypoint)
                elif self.repeat:
                    self.current_waypoint_index = 1
                    print("Reached final waypoint")
                else:
                    self.state="stopping"
                    print("Reached final waypoint")
                    self.current_waypoint_index = 1

            x, y = current_waypoint
            steering, throttle, _, _ = self.waypoint_controller.calculate_control(x, y, sim_pose, sim_orientation)

            if self.hit!='none':
                print("COLLISION!", self.hit)
                self.state = "stopping"
            elif self.state=="stopping":
                self.send_controls(steering, 0.0,1.0)
                self.state = "stopped"
            elif self.state=="driving":
                throttle_correction = self.pid_controller.calculate(self.speed_threshold_min, self.speed/self.SIZE_FACTOR)
                throttle=0.3+throttle_correction
                throttle=min(max(0,throttle),1)
                self.controls=[steering,throttle]
                self.send_controls(steering,throttle)
            else:
                self.send_controls(0, 0.0,1.0)
            
            if self.show_preview:
                current_time = time.time()
                if current_time - self.last_display_time >= 1:
                    self.last_display_time = current_time

                    plt.figure('Current View-Window')
                    plt.imshow(self.image_toplot)
                    plt.axis('off')
                    plt.draw()
                    plt.pause(0.001)
                    if self.use_masks_sim:
                        plt.figure('Current Road-mask')
                        plt.imshow(self.segmentation_mask)
                        plt.axis('off')
                        plt.draw()
                        plt.pause(0.001)


    def check_with_waypoints(self):
        sim_pose=self.pos_x,self.pos_y,self.pos_z
        for lane in ["left","center","right"]:
            current_waypoint = self.waypoints[lane][self.current_waypoint_index]
            x, y = current_waypoint
            distance = self.waypoint_controller.calculate_distance(x,y,sim_pose)
            if distance <= self.WAYPOINT_THRESHOLD:
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints[lane]):
                    print(f"Progress: {self.current_waypoint_index}/{len(self.waypoints[lane])}")
                    return False
                else:
                    print("Reached final waypoint")
                    return True




    def send_pose(self,tracked_pose_sim):
        msg = { 'msg_type' : 'tracking', 'x': tracked_pose_sim[0].__str__(), 'y':tracked_pose_sim[1].__str__(), 'angle': tracked_pose_sim[2].__str__() }
        self.client.send_now(msg)

    def on_connect(self, client):
        self.client = client
        self.timer.reset()

    def on_aborted(self, msg):
        self.stop()

    def on_disconnect(self):
        pass


def generate_road(road_generation,road_points,start_angle,start_offset_real,closed_road,output_filename):
    if closed_road:
        initial_pose_sim,initial_pose_real,sim_waypoint_list,road_definition=road_generation.create_closed_road(road_points,start_angle,start_offset_real)
    else:
        initial_pose_sim,initial_pose_real,sim_waypoint_list,road_definition=road_generation.create_non_closed_road(road_points,start_angle,start_offset_real)
    left_lane, right_lane = road_generation.generate_road_margins(sim_waypoint_list, 1.5)
    left_margins, right_margins = road_generation.generate_road_margins(sim_waypoint_list, OFFSET)
    if closed_road:
        left_lane.append(left_lane[0])
        right_lane.append(right_lane[0])
        left_margins.append(left_margins[0])
        right_margins.append(right_margins[0])

    sim_x, sim_y = zip(*[((x-X_MAP_SHIFT) / SIZE_FACTOR, (y-Y_MAP_SHIFT) / SIZE_FACTOR) for x, y in sim_waypoint_list])
    left_lane_x, left_lane_y = zip(*[((x-X_MAP_SHIFT) / SIZE_FACTOR, (y-Y_MAP_SHIFT) / SIZE_FACTOR) for x, y in left_lane])
    right_lane_x, right_lane_y = zip(*[((x-X_MAP_SHIFT) / SIZE_FACTOR, (y-Y_MAP_SHIFT) / SIZE_FACTOR) for x, y in right_lane])
    left_margin_x, left_margin_y = zip(*[((x-X_MAP_SHIFT) / SIZE_FACTOR, (y-Y_MAP_SHIFT) / SIZE_FACTOR) for x, y in left_margins])
    right_margin_x, right_margin_y = zip(*[((x-X_MAP_SHIFT) / SIZE_FACTOR, (y-Y_MAP_SHIFT) / SIZE_FACTOR) for x, y in right_margins])

    data = {
        "initial_pose_sim": initial_pose_sim,
        "initial_pose_real": initial_pose_real,
        "sim_waypoint_list": sim_waypoint_list,
        "road_definition": road_definition,
        "left_lane": left_lane,
        "right_lane": right_lane,
        "left_margin": left_margins,
        "right_margin": right_margins,
        "obstacles": []
    }

    x_marks=[x for x,y in road_points]
    y_marks=[y for x,y in road_points]

    with open(output_filename, 'w') as json_file:
        json.dump(data, json_file, indent=4)
    print(f"JSON file '{output_filename}' has been created successfully.")

    plt.figure(figsize=(10, 8))
    plt.plot(sim_x, sim_y, 'b-', label="Simulated Waypoints")
    plt.plot(left_lane_x, left_lane_y, 'g-', label="Left Lane")
    plt.plot(right_lane_x, right_lane_y, 'r-', label="Right Lane")
    plt.plot(left_margin_x, left_margin_y, 'c--', label="Left Margin")
    plt.plot(right_margin_x, right_margin_y, 'm--', label="Right Margin")
    plt.plot((initial_pose_sim[0]-X_MAP_SHIFT) / SIZE_FACTOR, (initial_pose_sim[1]-Y_MAP_SHIFT)/ SIZE_FACTOR, 'ko', label="Pose", markersize=10)
    plt.plot(x_marks,y_marks, 'gx', label="waypoints", markersize=7)

    plt.title("Path and Lane Visualizations")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    min_val=min(min(min(left_margin_x),min(right_margin_x)),min(min(left_margin_y),min(right_margin_y)))
    max_val=max(max(max(left_margin_x),max(right_margin_x)),max(max(left_margin_y),max(right_margin_y)))
    plt.xlim([min_val, max_val])
    plt.ylim([min_val, max_val])
    plt.grid(True)
    plt.show()


def run_client(map_data,model,translate_model,time_to_drive,obstacles,speed_target=0.4,initial_lane="left",repeat=False,collect=None,invert=False,start_offset=None,real_model=False,show_preview=False,use_masks_sim=False):

    # SETUP SIMULATION CLIENT
    num_clients = 1
    clients = []
    address = ("127.0.0.1", 9091)

    # process = DonkeyProcess()
    # process.start(port=9091)
    # time.sleep(20)


    if invert and initial_lane=="left":
        initial_lane="right"
    elif invert and initial_lane=="right":
        initial_lane="left"
    print(f"initial lane: {initial_lane}")
    waypoints={}

    if start_offset!=None:
        typestr="margin"
    else:
        typestr="lane"

    if invert:
            waypoints["right"]=map_data["left_lane"][::-1]
            waypoints["left"]=map_data["right_lane"][::-1]
            waypoints["center"]=map_data["sim_waypoint_list"][::-1]
            if initial_lane=="left":
                initial_pose = map_data[f"right_{typestr}"][-1].copy()
            elif initial_lane=="right":
                initial_pose = map_data[f"left_{typestr}"][-1].copy()
            else:
                if start_offset!=None:
                    initial_pose = map_data[start_offset][-1].copy()
                else:
                    initial_pose = map_data["sim_waypoint_list"][-1].copy()
    else:
        waypoints["left"]=map_data["left_lane"]
        waypoints["right"]=map_data["right_lane"]
        waypoints["center"]=map_data["sim_waypoint_list"]
        if initial_lane=="left" or initial_lane=="right":
            initial_pose = map_data[f"{initial_lane}_{typestr}"][0].copy()
        else:
            if start_offset!=None:
                initial_pose = map_data[start_offset][0].copy()
            else:
                initial_pose = map_data["sim_waypoint_list"][0].copy()
        print(initial_pose)


    for _ in range(0, num_clients):
        # setup the clients
        handler = DonkeySimMsgHandler(model,translate_model,waypoints,initial_lane,obstacles,speed_target,repeat,real_model=real_model,show_preview=show_preview,use_masks_sim=use_masks_sim)
        client = SimClient(address, handler)
        clients.append(client)
    time.sleep(1)

    # SETUP MAP
    clients[0].msg_handler.reset_scenario(0,map_data["road_definition"])
    time.sleep(0.1)
    if invert:
        initial_pose.append(map_data["initial_pose_sim"][2]+180)
    else:
        initial_pose.append(map_data["initial_pose_sim"][2])
    print(f"initial pose_sim in map {initial_pose}")
    # SETUP OBSTACLES
    if obstacles!=None:
        for i,name in enumerate(obstacles["obstacle_names"]):
            clients[0].msg_handler.send_obstacle(name,obstacles['obstacle_poses'][i] ,obstacles['obstacle_orientations'][i])
        time.sleep(0.1)

    # SETUP VEHICLE POSE
    clients[0].msg_handler.send_pose(initial_pose)
    time.sleep(0.1)
    print(f"current pose in sim {clients[0].msg_handler.pos_x,clients[0].msg_handler.pos_y,clients[0].msg_handler.pos_z}")
    time.sleep(3)


    # MAIN LOOP
    start = time.time()
    quality_log=[]
    do_drive = True
    if collect is not None:
        json_files = [file for file in os.listdir(collect) if file.endswith('.json')]
        if len(json_files)>0:
            print(f"There already are {len(json_files)} files!, will append")
            iterations=len(json_files)
        else:
            iterations=0
    while time.time() - start < time_to_drive and do_drive:
        for i,c in enumerate(clients):
            if c.msg_handler.hit!='none':
                c.stop()
                print("Obstacle collision!!!")
                do_drive=False
                collision=True
            elif c.aborted:
                print("Client socket closed, stopping driving.")
                do_drive=False
            else:
                if model!=None:
                    c.msg_handler.update_with_model()
                    quality_log.append(c.msg_handler.cte)
                    if c.msg_handler.check_with_waypoints():
                        do_drive=False
                else:
                    c.msg_handler.update_with_waypoints()
                    if c.msg_handler.state=="stopped":
                        do_drive=False
                    if collect is not None and c.msg_handler.image_toplot is not None:
                        image=c.msg_handler.img_arr
                        # image = np.full((32, 24, 3), 128, dtype=np.uint8) #TODO remove
                        image_uint8 = (image).astype(np.uint8)
                        pil_image = Image.fromarray(image_uint8)
                        pil_image.save(f'{collect}/{iterations}_cam-image_array_.png')
                        json_data = {
                            "cam/image_array": f"{iterations}_cam-image_array_.png",
                            "user/throttle": c.msg_handler.controls[1],
                            "user/angle": c.msg_handler.controls[0],
                            "user/mode": "user",
                            "track/lap": c.msg_handler.lap,
                            "track/sector": c.msg_handler.sector,
                            "speed": c.msg_handler.speed,
                            "loc": 0,
                            "track/x": c.msg_handler.pos_x,
                            "track/y": c.msg_handler.pos_y,
                            "track/z": c.msg_handler.pos_z,
                            "track/cte": c.msg_handler.cte,
                            "track/oot": False,
                            "time": c.msg_handler.time
                        }
                        with open(f'{collect}/record_{iterations}.json', 'w') as json_file:
                            json.dump(json_data, json_file, indent=4)
                        iterations+=1
                time.sleep(0.01)
    for i,c in enumerate(clients):
            c.msg_handler.status="stopping"
    # process.quit()


    time.sleep(3.0)
    if model!=None:
        min_cte = min(quality_log)
        max_cte = max(quality_log)
        print(f"The min cte of this run was: {min_cte} (maximum error of {abs(min(0.0,min_cte+3.0))})")
        print(f"The max cte of this run was: {max_cte} (maximum error of {abs(max(0.0,max_cte-3.0))})")

    print("waiting for msg loop to stop")
    for c in clients:
        c.stop()

