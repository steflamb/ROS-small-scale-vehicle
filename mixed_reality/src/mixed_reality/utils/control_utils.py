import math
import time
import websockets
import json



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
            #print(f"Position\nx:{x_cur}, y:{y_cur}")
            _, angle_cur, _ = simulator_orientation
            #print(f"Orientation: {round(angle_cur, 3)}, {round(math.radians(angle_cur), 3)} rad")
            distance = math.sqrt((x_target - x_cur)**2 + (y_target - y_cur)**2)
            #print(f"dist {distance}")
            _, angle_y_axis_degrees, _=self.angle_extraction(x_cur, 0.0, y_cur, x_target, 0.0, y_target)
            #print(f"Angle to goal: {angle_y_axis_degrees}")
            angle_difference=self.angle_difference(angle_cur,angle_y_axis_degrees)
            #print(f"angle diff {angle_difference}")
            steering = (math.radians(angle_difference) / math.pi)*10
            throttle=1
            # print(f"from {[x_cur, y_target]} to {[x_target,y_target]}: steering={steering}")

            return steering, throttle, distance, angle_difference
        
        def calculate_distance(self, x_target, y_target, simulator_pose):
            x_cur, _, y_cur = simulator_pose
            distance = math.sqrt((x_target - x_cur)**2 + (y_target - y_cur)**2)
            # print(f"distance between {[round(x_cur,3),round(y_cur,3)]} and {[round(x_target,3),round(y_target,3)]} is {round(distance,3)}")
            return distance
        
        def calculate_angle(self, x_target, y_target, simulator_pose, simulator_orientation):
            x_cur, _, y_cur = simulator_pose
            _, angle_cur, _ = simulator_orientation
            _, angle_y_axis_degrees, _=self.angle_extraction(x_cur, 0.0, y_cur, x_target, 0.0, y_target)
            angle_difference=self.angle_difference(angle_cur,angle_y_axis_degrees)
            return angle_difference
        
        
        def rotate_to_point_start(self,point):
            print("[CAR WAYPOINT CONTROL] Rotating to point")
            i=0
            x,y=point
            angle_difference = self.calculate_angle(x,y)
            list_commands_thr=[1,-1]
            if angle_difference>0:
                    list_commands_str=[0.9,-0.9]
            else:
                    list_commands_str=[-0.9,0.9]
            while(abs(angle_difference)>self.angle_treshold and self.status.is_running and self.status.waypoint_state=="going_to_start"):
                print("Rotation angle remaining: ",int(angle_difference))
                time.sleep(.1)
                controls=(list_commands_thr[i],list_commands_str[i])
                self.status.car_controls=[controls[0],controls[1],False]
                time.sleep(0.5)
                if not self.status.real_car_controls:
                    self.status.car_controls=[controls[0],controls[1],True]
                    time.sleep(0.3)
                self.status.car_controls=[0,0,False]
                angle_difference = self.calculate_angle(x,y)
                time.sleep(0.3)
                if i==0:
                    i=1
                else:
                    i=0
            print("[CAR WAYPOINT CONTROL] Rotation complete")

        def rotate_to_angle_start(self,angle):
            print("[CAR WAYPOINT CONTROL] Rotating to angle")
            i=0
            _, angle_cur, _ = self.status.simulator_orientation
            angle_difference=self.angle_difference(angle_cur,angle)
            list_commands_thr=[1,-1]
            print(angle_difference)
            if angle_difference>0:
                    list_commands_str=[0.9,-0.9]
            else:
                    list_commands_str=[-0.9,0.9]
            while(abs(angle_difference)>self.angle_treshold and self.status.is_running):
                print("Rotation angle remaining: ",int(angle_difference))
                time.sleep(.1)
                controls=(list_commands_thr[i],list_commands_str[i])
                self.status.car_controls=[controls[0],controls[1],False]
                time.sleep(0.5)
                if not self.status.real_car_controls:
                    self.status.car_controls=[controls[0],controls[1],True]
                    time.sleep(0.3)
                self.status.car_controls=[0,0,False]
                _, angle_cur, _ = self.status.simulator_orientation
                angle_difference=self.angle_difference(angle_cur,angle)
                time.sleep(0.3)
                if i==0:
                    i=1
                else:
                    i=0
            print("[CAR WAYPOINT CONTROL] Rotation complete")
        
        def reach_start_loop(self, simulator_pose, simulator_orientation):    
            x,y=self.status.sim_initial_waypoint
            steering, throttle, distance, _ = self.calculate_control(x,y, simulator_pose, simulator_orientation)
            print("[CAR WAYPOINT CONTROL] Reaching start")
            while self.status.waypoint_state=="going_to_start" and self.status.is_running:
                if distance <= self.waypoint_treshold and self.status.waypoint_state!="reached_start":
                    self.status.car_controls=[throttle,steering,True]
                    self.status.set_waypoint_state("reached_start")
                    # self.status.set_state("stopping")
                    print("[CAR WAYPOINT CONTROL] REACHED START")
                elif self.status.waypoint_state!="reached_start":
                    self.status.car_controls=[throttle,steering,False]
                else:
                    self.status.car_controls=[0,steering,False]
                steering, throttle, distance, _ = self.calculate_control(x,y, simulator_pose, simulator_orientation)
                time.sleep(0.01)
            print("[CAR WAYPOINT CONTROL] Reached start")

        def go_to_waypoint_step(self, simulator_pose, simulator_orientation):    
            x,y=self.status.sim_target_waypoint
            steering, throttle, distance, _ = self.calculate_control(x,y, simulator_pose, simulator_orientation)
            if distance <= self.waypoint_treshold and self.status.waypoint_state!="reached_waypoint":
                self.status.car_controls=[throttle,steering,True]
                self.status.set_waypoint_state("reached_waypoint")
                self.status.set_state("stopping")
                real_waypoint=self.status.for_conversions.sim2real_xyp([x,y,0])
                print(f"[CAR WAYPOINT CONTROL] Reached waypont (SIM): {x}, {y} (REAL): {real_waypoint[0]}, {real_waypoint[1]}")
            elif self.status.waypoint_state!="reached_waypoint":
                self.status.car_controls=[throttle,steering,False]
                time.sleep(0.1)
            else:
                self.status.car_controls=[0,0,False]
                time.sleep(0.1)


        def go_to_start_logic(self, simulator_pose, simulator_orientation):
            print("[CAR WAYPOINT CONTROL]: Recieved a starting sequence request")
            initial_x,initial_y=self.status.sim_initial_waypoint[0],self.status.sim_initial_waypoint[1]
            _, angle_cur, _ = self.status.simulator_orientation
            if self.calculate_distance(initial_x,initial_y)<=self.waypoint_treshold:
                if self.angle_difference(angle_cur,self.status.sim_initial_angle)<=self.angle_treshold:
                    print("[CAR WAYPOINT CONTROL]: Already in start pose")
                    self.status.set_waypoint_state("reached_start")
                    self.status.set_state("driving")
                else:
                    print("[CAR WAYPOINT CONTROL]: Already in start pose but wrong angle")
                    self.rotate_to_angle_start(self.status.sim_initial_angle)
                    self.status.set_waypoint_state("reached_start")
                    time.sleep(1)
                    self.status.set_state("driving")
            else:
                self.rotate_to_point_start(self.status.sim_initial_waypoint)
                self.reach_start_loop(simulator_pose, simulator_orientation)
                time.sleep(2)
                self.status.car_controls=[0.,0.,True]
                self.rotate_to_angle_start(self.status.sim_initial_angle)
                self.status.set_state("stopping")


#TODO: check if Car_control_utils is used anywhere at all
class Car_control_utils():
    def __init__(self,uri,fixed_throttle,real_controls):
        self.websocket=None
        self.uri=uri
        self.real_controls=real_controls
        #self.sim_client=status.simulator_client_object
        self.fixed_throttle=fixed_throttle



    async def send_command_to_car(self, throttle, steering):
        command = {
            "throttle": throttle,
            "angle": steering
        }
        message = json.dumps(command)
        await self.websocket.send(message)

    def send_command_to_sim(self,throttle,steering,brake):
        if brake:
            message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':'0.0', 'brake': '1.0' }
        else:
            message = { 'msg_type' : 'control', 'steering': steering.__str__(), 'throttle':throttle.__str__(), 'brake': '0.0' }    
        
        self.sim_client.queue_message(message)

    async def connect_and_control_real_car(self):
        if self.real_controls:
            async with websockets.connect(self.uri) as websocket:
                self.websocket=websocket
                await self.car_control_loop()
        else:
            await self.car_control_loop()

    async def connect_and_control_obstacle(self):
        
        await self.obstacle_control_loop()

    async def car_send_commands(self,throttle,steering):
        throttle=throttle*self.status.real_throttle_multiplier
        steering=steering*self.status.real_steering_multiplier
        
        if steering>1:
            steering=0.99
        if steering<-1:
            steering=-0.99
        if self.status.state=='stopping':
            self.status.set_state('stopped')
            if self.real_controls:
                await self.send_command_to_car(-throttle, steering)
                # self.send_command_to_sim(0, steering, False)
                self.send_command_to_sim(-throttle, steering,True)
                time.sleep(0.3)
                await self.send_command_to_car(0., steering)
            else:
                self.send_command_to_sim(-throttle, steering,True)
        elif self.status.state=='stopped':
            if self.real_controls:
                await self.send_command_to_car(0., steering)
                # self.send_command_to_sim(0., 0.,False)
                self.send_command_to_sim(0., 0.,True)
            else:
                self.send_command_to_sim(0., 0.,True)
        else:
            if self.real_controls:
                real_throttle=throttle
                if throttle<0:
                    real_throttle=throttle*3
                await self.send_command_to_car(real_throttle, steering)
                self.send_command_to_sim(throttle, steering, False)
                # self.send_command_to_sim(0.0, steering, False)
            else:
                self.send_command_to_sim(throttle, steering, False)

    async def car_send_brake_command(self,throttle,steering):
        if self.status.state!='stopping' and self.status.state!='stopped':
            if self.real_controls:
                await self.send_command_to_car(-throttle, steering)
                self.send_command_to_sim(0, steering,False)
                time.sleep(0.3)
                await self.send_command_to_car(0., steering)
            else:
                self.send_command_to_sim(-throttle, steering,True)

    
    async def car_control_loop(self):
        while self.status.is_running:
            try:
                throttle=self.status.car_controls[0]
                steering=self.status.car_controls[1]
                brake=self.status.car_controls[2]
                if self.status.collision!='none' and self.status.state!="stopped":
                    print("COLLISION!",self.status.collision)
                    self.status.set_state("stopping")
                if brake:
                    await self.car_send_brake_command(throttle,steering)
                else:
                    await self.car_send_commands(throttle,steering)
                time.sleep(0.05)
            except Exception as e:
                print(f"Error: {e}")

    async def obstacle_control_loop(self):
        while self.status.is_running:
            try:
                throttle=0.
                steering=0.
                brake=0.
                if self.status.collision!='none':
                    print("COLLISION!",self.status.collision)
                await self.car_send_commands(throttle,steering)
                time.sleep(0.02)
            except Exception as e:
                print(f"Error: {e}")