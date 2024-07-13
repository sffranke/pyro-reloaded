import os
import sys
import numpy as np
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import time
from math import pi
from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure

plt.ion()

class SpotMicroKinematics:
    def __init__(self):
        self.d2r = pi / 180
        self.r2d = 180 / pi
        self.sm = SpotMicroStickFigure(x=0, y=0.18, z=0, theta=5 * self.d2r)
        self.desired_p4_points = np.array([
            [-self.sm.body_length / 2, 0, self.sm.body_width / 2 + self.sm.hip_length],
            [self.sm.body_length / 2, 0, self.sm.body_width / 2 + self.sm.hip_length],
            [self.sm.body_length / 2, 0, -self.sm.body_width / 2 - self.sm.hip_length],
            [-self.sm.body_length / 2, 0, -self.sm.body_width / 2 - self.sm.hip_length]
        ])
        self.sm.set_absolute_foot_coordinates(self.desired_p4_points)

class SpotMicro:
    def __init__(self, ax):
        self.kinematics = SpotMicroKinematics()
        self.ax = ax
        self.kinematics.sm.set_body_angles(theta=5 * self.kinematics.d2r)
        coords = self.kinematics.sm.get_leg_coordinates()
        self.ax.set_xlim3d([-0.2, 0.2])
        self.ax.set_zlim3d([0, 0.4])
        self.ax.set_ylim3d([0.2, -0.2])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Z')
        self.ax.set_zlabel('Y')
        self.lines = self.init_lines(coords)
        self.stopwalk = False
        self.total_time = 1

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

    def update_lines(self, coords):
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
        '''
        print("leg_rightback:  ", deg_arr[0])
        print("leg_rightfront: ", deg_arr[1])
        print("leg_leftfront:  ", deg_arr[2])
        print("leg_leftback:   ", deg_arr[3])
        '''
    def set_current_angles(self, moving_time, steps, angles_arr, pitch):
        current_angles = self.get_current_angles()
        #print("Vorher")
        self.print_angles(current_angles)
        target_angles = [
            [angles_arr[0][0] * self.kinematics.d2r, -angles_arr[0][1] * self.kinematics.d2r,  angles_arr[0][2] * self.kinematics.d2r],
            [angles_arr[1][0] * self.kinematics.d2r, -angles_arr[1][1] * self.kinematics.d2r,  angles_arr[1][2] * self.kinematics.d2r],
            [angles_arr[2][0] * self.kinematics.d2r,  angles_arr[2][1] * self.kinematics.d2r, -angles_arr[2][2] * self.kinematics.d2r],
            [angles_arr[3][0] * self.kinematics.d2r,  angles_arr[3][1] * self.kinematics.d2r, -angles_arr[3][2] * self.kinematics.d2r]
        ]

        step_delay = moving_time / steps

        for step in range(steps):
            t = (1 - np.cos(np.pi * step / steps)) / 2  # Sinusoidal interpolation
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

        #print("Nachher")
        self.print_angles(self.get_current_angles())
        #print("########")

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
        angles = [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]]
        moving_time = 0.5
        steps = 20
        pitch = 5
        self.pose(moving_time, steps, angles, pitch)

    def calculate_start_time_offsets(self, gait):
        if gait == "wave":
            phase_time =  1 / 2
            offsets = [2 * phase_time, 1 * phase_time, 3 * phase_time, 0 * phase_time]
            return offsets
        elif gait == "trot":
            phase_time = 1
            offsets = [0 * phase_time, 1 * phase_time, 2 * phase_time, 3 * phase_time]
            return offsets
        elif gait == "pace":
            phase_time = 1
            offsets = [0, 0, phase_time, phase_time]
            return offsets
        elif gait == "gallop":
            phase_time = 1 / 2
            offsets = [0, 0, phase_time, phase_time]
            return offsets
        elif gait == "rotate_left" or gait == "rotate_right":
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
                y = 0  # Ground level
            elif t_mod < stance_time / 2 + swing_time:
                x, y = self.point_on_ellipse(t_mod - stance_time / 2, swing_time, radius, swing_height, reverse=True)
                z = 0  # Maintain the same z position
            else:
                x, z = self.linear_motion(t_mod - stance_time / 2 - swing_time, stance_time / 2, radius / 2, 0)
                y = 0  # Ground level

            # Rotate the points by the specified angle
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
        
        start_time_offsets = self.calculate_start_time_offsets(gait_pattern)
       
        for n in range(repetitions):
            positions = {}
            for i, leg_name in enumerate(leg_names):
                if gait_pattern == "rotate_left":
                    if leg_name in ["front_right", "back_right"]:
                        # These legs move forward
                        radius = self.stepwidth[i]
                    else:
                        # These legs move backward
                        radius = -self.stepwidth[i]
                elif gait_pattern == "rotate_right":
                    if leg_name in ["front_right", "back_right"]:
                        # These legs move backward
                        radius = -self.stepwidth[i]
                    else:
                        # These legs move forward
                        radius = self.stepwidth[i]
                else:
                    radius = self.stepwidth[i]
                    
                positions = self.getpositions(positions, leg_name, self.kinematics.desired_p4_points[i], steps, radius, start_time_offsets[i], swing_heights[i], swing_time_ratios[i], angle)
                    
            if self.stopwalk:
                break
            start_time = time.time()
            for step in range(steps):
                mypoints = []
                for i, leg_name in enumerate(leg_names):
                    p = positions[leg_name][step]
                    new_point = [self.kinematics.desired_p4_points[i][0] + p[0],
                                 self.kinematics.desired_p4_points[i][1] + p[1],
                                 self.kinematics.desired_p4_points[i][2] + p[2]]
                    mypoints.append(new_point)
                
                self.anim(mypoints, pitch=10.0)
                elapsed = time.time() - start_time
                time.sleep(max(0, self.total_time / steps - elapsed))
                start_time = time.time()

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    walker = SpotMicro(ax)
    total_time = 2 
    repetitions = 3
    steps = 90
    radii = [0.04, 0.04, 0.04, 0.04] 
    overlap_times = [0.0, 0.0, 0.0, 0.0]
    swing_heights = [0.03, 0.03, 0.03, 0.03]
    swing_time_ratios = [0.25, 0.25, 0.25, 0.25]
    angle = np.deg2rad(0)  # example angle in radians
    walker.walk(total_time, repetitions, radii, steps, "rotate_left", overlap_times, swing_heights, swing_time_ratios, angle)
    plt.pause(0.01)
    
    angle = np.deg2rad(0)  # example angle in radians
    walker.walk(total_time, repetitions, radii, steps, "rotate_right", overlap_times, swing_heights, swing_time_ratios, angle)
    plt.pause(0.01)
    
    angle = np.deg2rad(0)  
    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, angle)
    plt.pause(0.01)
    angle = np.deg2rad(45)  
    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, angle)
    plt.pause(0.01)
    angle = np.deg2rad(-45)  
    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, angle)
    plt.pause(0.01)
    angle = np.deg2rad(90)  
    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, angle)
    plt.pause(0.01)
    plt.show()
    angle = np.deg2rad(-90)  
    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, angle)
    plt.pause(0.01)
    plt.show()
    angle = np.deg2rad(180) 
    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, angle)
    plt.pause(0.01)
    plt.show()

if __name__ == "__main__":
    main()
