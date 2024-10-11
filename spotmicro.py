import time
import json
import socket
import numpy as np
import matplotlib.pyplot as plt
import threading
from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk
from spot_micro_kinematics import SpotMicroKinematics
from config_loader import load_config

import socket
import json

from read_mpu6050 import MPU6050SensorKalman

# Globale Variablen für die Sensordaten
pitch_data = 0.0
roll_data = 0.0
thread_running = False

# Lock für den Thread-Schutz

sensor_lock = threading.Lock()

def sensor_thread():
    global pitch_data, roll_data
    sensor = MPU6050SensorKalman() 

    while True:
        roll_abs, pitch_abs = sensor.get_absolute_angles()

        with sensor_lock:
            pitch_data = pitch_abs
            roll_data = roll_abs

        time.sleep(0.1)  # Warte kurz, bevor neue Daten erfasst werden

plt.ion()
config = load_config()

class SpotMicro:

    sensor_thread_instance = threading.Thread(target=sensor_thread)
    sensor_thread_instance.daemon = True  
    sensor_thread_instance.start()
    
    def __init__(self, mode, ax):
        self.kinematics = SpotMicroKinematics()
        self.mode = mode
        self.stopwalk=True
        self.ax = ax
        self.kinematics.sm.set_body_angles(theta=5 * self.kinematics.d2r)
        coords = self.kinematics.sm.get_leg_coordinates()
        print ("X", self.mode)
        if self.mode != "real":
            self.ax.set_xlim3d([-0.2, 0.2])
            self.ax.set_zlim3d([0, 0.4])
            self.ax.set_ylim3d([0.2, -0.2])
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Z')
            self.ax.set_zlabel('Y')
            self.lines = self.init_lines(coords)
        else:
            from adafruit_servokit import ServoKit
            self.kit = ServoKit(channels=16)

        self.stop = True
        self.total_time = 1
        self.height = 0.18
        
    def init_lines(self, coords):
        lines = []
        for i in range(4):
            ind = -1 if i == 3 else i
            x_vals = [coords[ind][0][0], coords[ind + 1][0][0]]
            y_vals = [coords[ind][0][1], coords[ind + 1][0][1]]
            z_vals = [coords[ind][0][2], coords[ind + 1][0][2]]
            line, = self.ax.plot(x_vals, z_vals, y_vals, color='k')
            lines.append(line)

        plt_colors = ['r', 'c', 'b']
        for leg in coords:
            for i in range(3):
                x_vals = [leg[i][0], leg[i + 1][0]]
                y_vals = [leg[i][1], leg[i + 1][1]]
                z_vals = [leg[i][2], leg[i + 1][2]]
                line, = self.ax.plot(x_vals, z_vals, y_vals, color=plt_colors[i])
                lines.append(line)
        return lines


    ####
    def release_servos(self):
        '''Releases all servos by setting their angles to None.'''
        for i in range(16):
            self.kit.servo[i].angle = None

    def control_servos(self, angles):
        
    
        '''Controls the servos based on the received angles.'''
        if len(angles) != 4:
            logging.error("Invalid number of angle sets received. Expected 4 sets of angles.")
            return

        angle = [
            [angles[0][0], angles[0][1], angles[0][2]],  # RR
            [angles[1][0], angles[1][1], angles[1][2]],  # RF 
            [angles[2][0], -angles[2][1], -angles[2][2]],  # LF
            [angles[3][0], -angles[3][1], -angles[3][2]]  # LR
        ]
        
        corrections = [
            [+10, +0, +0],  # RR
            [-10, +0, +0],  # RF 
            [+0, 0, 0],  # LF
            [-4, +0, +15]  # LR
        ]
        
        rotations = [
            [1, +1, -1],  # RR
            [-1, 1, -1],  # RF 
            [1,  -1, 1],  # LF
            [-1, 1, +1]  # LR
        ]
        
        rr_coxa_pin = 0
        rr_tibia_pin = 1
        rr_femur_pin = 2
        
        rf_coxa_pin = 5
        rf_tibia_pin = 6
        rf_femur_pin = 7
        
        lf_coxa_pin = 9
        lf_tibia_pin = 10
        lf_femur_pin = 11
        
        lr_coxa_pin = 13
        lr_tibia_pin = 14
        lr_femur_pin = 15
        
        servo_angles = [
            (rr_coxa_pin, 90 + rotations[0][0] * (angle[0][0] + corrections[0][0])),  # RR coxa
            (rr_tibia_pin, 90 + rotations[0][1] * (angle[0][1] + corrections[0][1])),  # RR tibia
            (rr_femur_pin, 180 - (angle[0][2] - corrections[0][2])),  # RR femur
            
            (rf_coxa_pin, 90 + rotations[1][0] * (angle[1][0] + corrections[1][0])),  # RF coxa
            (rf_tibia_pin, 90 + rotations[1][1] * (angle[1][1] + corrections[1][1])),  # RF tibia
            (rf_femur_pin, 180 - (angle[1][2] - corrections[1][2])),  # RF femur
            
            (lf_coxa_pin, 90 + rotations[2][0] * (angle[2][0] + corrections[2][0])),  # LF coxa
            (lf_tibia_pin, 90 + rotations[2][1] * (angle[2][1] + corrections[2][1])),  # LF tibia
            (lf_femur_pin, 180 - (angle[2][2] - corrections[2][2])),  # LF femur
            
            (lr_coxa_pin, 90 + rotations[3][0] * (angle[3][0] + corrections[3][0])),  # LR coxa
            (lr_tibia_pin, 90 + rotations[3][1] * (angle[3][1] + corrections[3][1])),  # LR tibia
            (lr_femur_pin, angle[3][2] + corrections[3][2])  # LR femur
        ]

        for servo, angle in servo_angles:
            self.kit.servo[servo].angle = angle
        
        if (thread_running==False):
            global pitch_data, pitch_data, sensor_lock
            with sensor_lock:
                current_pitch = pitch_data
                current_roll = roll_data
                thread_running==True
                print(f"Aktueller Pitch: {current_pitch:.2f}°, Aktueller Roll: {current_roll:.2f}°")
                self.kinematics.sm.set_body_angles(phi=current_roll* self.kinematics.d2r, theta=current_pitch * self.kinematics.d2r)
                #coords = self.get_current_coords()
                #self.update_lines(coords)
                #### hier self.kinematics.sm.set_body_angles(theta=5 * self.kinematics.d2r)
                '''
                def set_body_angles(self,phi=0,theta=0,psi=0):
                Set a body angles without translation of the body

                Args:
                phi: roll angle in radians
                theta: pitch angle in radians
                psi: yaw angle in radians
                '''
    
    ####
    
    def update_lines(self, coords):
 
        if self.mode == "real":
            angles = self.get_current_angles()
            deg_arr = tuple(tuple(round(value * self.kinematics.r2d) for value in inner_array) for inner_array in angles)
            #print("deg_arr", deg_arr)

            # here comes the real robot

            #works, way to slow and laggy over WiFi using a Pi 3
            
            ###data_to_send = json.dumps(deg_arr).encode()
            #print("data_to_send:", deg_arr)
            
            self.control_servos(deg_arr)
            
            '''
            host = config['host']
            port = config['port']

            connected = False
            while not connected:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.connect((host, port))
                        s.sendall(data_to_send)
                        connected = True  
                except socket.error as e:
                    print(f"Connection failed: {e}. Retrying in 5 seconds...")
                    time.sleep(5)  
           '''         
           
        else:
            for i in range(4):
                ind = -1 if i == 3 else i
                x_vals = [coords[ind][0][0], coords[ind + 1][0][0]]
                y_vals = [coords[ind][0][1], coords[ind + 1][0][1]]
                z_vals = [coords[ind][0][2], coords[ind + 1][0][2]]
                self.lines[i].set_data(x_vals, z_vals)
                self.lines[i].set_3d_properties(y_vals)

            plt_colors = ['r', 'c', 'b']
            for leg_index, leg in enumerate(coords):
                for joint_index in range(3):
                    x_vals = [leg[joint_index][0], leg[joint_index + 1][0]]
                    y_vals = [leg[joint_index][1], leg[joint_index + 1][1]]
                    z_vals = [leg[joint_index][2], leg[joint_index + 1][2]]
                    line_index = 4 + leg_index * 3 + joint_index
                    self.lines[line_index].set_data(x_vals, z_vals)
                    self.lines[line_index].set_3d_properties(y_vals)
            
            plt.draw()
            self.ax.figure.canvas.flush_events()

    def print_angles(self, arr):
        deg_arr = tuple(tuple(round(value * self.kinematics.r2d) for value in inner_array) for inner_array in arr)
        print("-------------------------------")
        print("leg_rightback:", deg_arr[0])
        print("leg_rightfront:", deg_arr[1])
        print("leg_leftfront:", deg_arr[2])
        print("leg_leftback:", deg_arr[3])
        
    def set_current_angles(self, moving_time, steps, angles_arr, pitch):
        global mode
        current_angles = self.get_current_angles()
        target_angles = [
            [angles_arr[0][0] * self.kinematics.d2r, -angles_arr[0][1] * self.kinematics.d2r,  angles_arr[0][2] * self.kinematics.d2r],
            [angles_arr[1][0] * self.kinematics.d2r, -angles_arr[1][1] * self.kinematics.d2r,  angles_arr[1][2] * self.kinematics.d2r],
            [angles_arr[2][0] * self.kinematics.d2r,  angles_arr[2][1] * self.kinematics.d2r, -angles_arr[2][2] * self.kinematics.d2r],
            [angles_arr[3][0] * self.kinematics.d2r,  angles_arr[3][1] * self.kinematics.d2r, -angles_arr[3][2] * self.kinematics.d2r]
        ]

        #step_delay = moving_time / steps

        for step in range(steps):
            t = (1 - np.cos(np.pi * step / steps)) / 2
            #print ("t:", t)
            interpolated_angles = [
                [
                    current_angles[leg][joint] * (1 - t) + target_angles[leg][joint] * t
                    for joint in range(3)
                ]
                for leg in range(4)
            ]
            self.kinematics.sm.set_leg_angles(interpolated_angles)
            coords = self.get_current_coords()
            mypoints = [coords[0][3], coords[1][3], coords[2][3], coords[3][3]]
            self.anim(mypoints, pitch)
            
    def get_current_angles(self):
        return self.kinematics.sm.get_leg_angles()

    def get_current_coords(self):
        return self.kinematics.sm.get_leg_coordinates()
        
    def anim(self, mypoints, pitch):
        self.kinematics.sm.set_absolute_foot_coordinates(mypoints)
        self.kinematics.sm.set_body_angles(theta=pitch * self.kinematics.d2r)
        coords = self.get_current_coords()
        self.update_lines(coords)

    def initplot(self):
        angles = config['stand_angles']
        moving_time = config['total_time']
        steps = config['steps']
        pitch = 5
        self.pose(moving_time, steps, angles, pitch)

    def calib(self):
        angles = config['calib_angles']
        moving_time = config['total_time']
        steps = config['steps']
        pitch = 5
        self.pose(moving_time, steps, angles, pitch)
        plt.pause(30)

    def pose(self, moving_time, steps, leg_angles, pitch):
        self.set_current_angles(moving_time, steps, leg_angles, pitch)
        
    def twist(self, roll, pitch, yaw):
        self.kinematics.sm.set_body_angles(
            theta=pitch * self.kinematics.d2r,
            phi=roll * self.kinematics.d2r,
            psi=yaw * self.kinematics.d2r
        )
        coords = self.get_current_coords()
        self.update_lines(coords)
    
    def calculate_start_time_offsets(self, gait):
        if gait == "wave":
            phase_time = 1 / 2
            offsets = [2 * phase_time, 1 * phase_time, 3 * phase_time, 0 * phase_time]
        elif gait == "trot":
            phase_time = 1
            offsets = [0 * phase_time, 1 * phase_time, 2 * phase_time, 3 * phase_time]
        elif gait == "pace":
            phase_time = 1
            offsets = [0, 0, phase_time, phase_time]
        elif gait == "gallop":
            phase_time = 1 / 2
            offsets = [0, 0, phase_time, phase_time]
        elif gait in ["rotate_left", "rotate_right"]:
            phase_time = 1 / 2
            offsets = [2 * phase_time, 1 * phase_time, 3 * phase_time, 0 * phase_time]
        return offsets
    
    def getpositions(self, positions, name, desired_p4_points, steps, radius, start_time_offset, swing_height, swing_time_ratio, angle=0):
        swing_time = 2 * swing_time_ratio
        stance_time = 2 * (1 - swing_time_ratio)
        swing_steps = int(steps * swing_time_ratio)
        stance_steps = steps - swing_steps

        for t in np.linspace(0, 2, steps):
            t_mod = (t + start_time_offset) % 2
            if t_mod < stance_time / 2:
                x, z = self.linear_motion(t_mod, stance_time / 2, 0, -radius / 2)
                y = 0
            elif t_mod < stance_time / 2 + swing_time:
                x, y = self.point_on_ellipse(t_mod - stance_time / 2, swing_time, radius, swing_height, reverse=True)
                z = 0
            else:
                x, z = self.linear_motion(t_mod - stance_time / 2 - swing_time, stance_time / 2, radius / 2, 0)
                y = 0

            x_rot = x * np.cos(angle) - z * np.sin(angle)
            z_rot = x * np.sin(angle) + z * np.cos(angle)

            if name not in positions:
                positions[name] = []
            positions[name].append((x_rot, y, z_rot))
            
        return positions
    
    def point_on_ellipse(self, t, T, radius, swing_height, reverse=False):
        angle = np.pi * t / T
        if reverse:
            angle = np.pi - angle
        x = (radius / 2) * np.cos(angle)
        y = swing_height * np.sin(angle)
        return x, y

    def linear_motion(self, t, T, start_pos, end_pos):
        return start_pos + (end_pos - start_pos) * (t / T), 0
    
    def walk(self, total_time, repetitions, radii, steps, gait_pattern, overlap_times, swing_heights, swing_time_ratios, angle=0):
        leg_names = ["front_right", "back_right", "back_left", "front_left"]
        
        self.total_time = total_time
        self.stepwidth = radii 
        self.angle = angle
        
        start_time_offsets = self.calculate_start_time_offsets(gait_pattern)
       
        for n in range(repetitions):
            positions = {}
            for i, leg_name in enumerate(leg_names):
                if gait_pattern == "rotate_left":
                    radius = self.stepwidth[i] if leg_name in ["front_right", "back_right"] else -self.stepwidth[i]
                elif gait_pattern == "rotate_right":
                    radius = -self.stepwidth[i] if leg_name in ["front_right", "back_right"] else self.stepwidth[i]
                else:
                    radius = self.stepwidth[i]
                    
                print("walk.... angle:", self.angle)    
                positions = self.getpositions(positions, leg_name, self.kinematics.desired_p4_points[i], steps, radius, start_time_offsets[i], swing_heights[i], swing_time_ratios[i], self.angle)
                    
            if self.stopwalk:
                return
            start_time = time.time()
            for step in range(steps):
                if self.stopwalk:
                    return
                mypoints = []
                for i, leg_name in enumerate(leg_names):
                    p = positions[leg_name][step]
                    new_point = [
                        self.kinematics.desired_p4_points[i][0] + p[0],
                        self.kinematics.desired_p4_points[i][1] + p[1],
                        self.kinematics.desired_p4_points[i][2] + p[2]
                    ]
                    mypoints.append(new_point)
                
                self.anim(mypoints, pitch=10.0)
                elapsed = time.time() - start_time
                time.sleep(max(0, self.total_time / steps - elapsed))
                start_time = time.time()

    def update_speed(self, adjustment):
        self.max = 0.1
        self.min = 4
        self.total_time += adjustment
        self.total_time = max(min(self.total_time, 3.9), 0.11)
        print(f"Updated total_time: {self.total_time}")
        
    def update_stepwidth(self, adjustment):
        self.max = 0.10
        self.stepwidth = [x + adjustment for x in self.stepwidth]
        
    def update_angle(self, adjustment):
        self.angle = adjustment
        
    def update_pose(self, adjustment):
        self.angle = adjustment
        
    def update_height(self, adjustment):
        self.height += adjustment
        print(f"Updated height {self.height}")
        self.kinematics.sm.set_body_height(self.height)
        coords = self.get_current_coords()
        self.update_lines(coords)
                

    def demo(self):

        repetitions = 5
        total_time = config['total_time']
        steps = config['steps']
        radii = [0.04, 0.04, 0.04, 0.04] 
        overlap_times = config['overlap_times']
        swing_heights = config['swing_heights']
        swing_time_ratios = config['swing_time_ratios']

        print("walk straight")
        self.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, np.deg2rad(0))
        plt.pause(0.01)
        
        print("walk diagonal +45")
        self.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, np.deg2rad(45))
        plt.pause(0.01)
        
        print("walk diagonal -45")
        self.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, np.deg2rad(-45))
        plt.pause(0.01)
        
        print("walk sideways 90")
        self.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, np.deg2rad(90))
        plt.pause(0.01)
        
        print("walk sideways -90")
        self.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, np.deg2rad(-90))
        plt.pause(0.01)
        
        print("walk backward")
        self.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, np.deg2rad(180))
        plt.pause(0.01)

        moving_time = config['total_time']
        steps = config['steps']
        repetitions = 2
        radii = [0.045, 0.045, 0.045, 0.045] 
        overlap_times = [0.0, 0.0, 0.0, 0.0]
        swing_heights = [0.03, 0.03, 0.03, 0.03]
        swing_time_ratios = [0.25, 0.25, 0.25, 0.25]

        print("Wave Gait - most stable")
        self.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios)
        plt.pause(1)
        
        print("Trot Gait - second stable")
        swing_time_ratios = [0.5, 0.5, 0.5, 0.5]
        self.walk(total_time, repetitions, radii, steps, "trot", overlap_times, swing_heights, swing_time_ratios)
        plt.pause(1)
        
        print("Pace Gait")
        swing_time_ratios = [0.5, 0.5, 0.5, 0.5]
        self.walk(total_time, repetitions, radii, steps, "pace", overlap_times, swing_heights, swing_time_ratios)
        plt.pause(1)
        
        print("Gallop Gait")
        swing_time_ratios = [0.5, 0.5, 0.5, 0.5]
        self.walk(total_time, repetitions, radii, steps, "gallop", overlap_times, swing_heights, swing_time_ratios)
        plt.pause(1)

        print("roll, pitch, yaw")
        roll = 12
        pitch = 12
        yaw = 12
        self.twist(roll, pitch, yaw)
        plt.pause(1)

        print("rotate - left")
        self.walk(total_time, repetitions, radii, steps, "rotate_left", overlap_times, swing_heights, swing_time_ratios)
        plt.pause(1)

        print("rotate - right")
        self.walk(total_time, repetitions, radii, steps, "rotate_right", overlap_times, swing_heights, swing_time_ratios)
        plt.pause(1)

        print("Stand")
        angles = config['stand_angles']
        self.pose(0.5, 20, angles, 5)
        plt.pause(1)

        print("Rest")
        angles = config['rest_angles']
        self.pose(1, 20, angles, 5)
        plt.pause(1)

        print("Sit")
        angles = config['sit_angles']
        self.pose(1, 20, angles, 5)
        plt.pause(1)

        print("Pee")
        angles = config['pee_angles']
        self.pose(1, 20, angles, 5)
        plt.pause(1)
        
        print("Pushups")
        def pushup(cnt):
            poses = [
                [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]],
                [[0, 80, 135], [0, 80, 135], [0, 80, 135], [0, 80, 135]]
            ]
            for _ in range(cnt):
                for angles in poses:
                    self.pose(1, 20, angles, 5)
                    plt.pause(0.25)
        pushup(3)
        plt.pause(1)
        
        print("Greet")
        def wave(wavecnt):
            poses = [
                [[0, 80, 135], [0, 35, 60], [0, -45, 60], [0, 80, 135]],
                [[0, 80, 135], [0, 35, 60], [0, -45, 50], [0, 80, 135]]
            ]
            for _ in range(wavecnt):
                for angles in poses:
                    self.pose(1, 20, angles, 5)
        wave(3)
