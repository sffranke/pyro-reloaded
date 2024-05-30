import numpy as np
import matplotlib.pyplot as plt
import time
from math import pi
from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure

plt.ion()
degree2rad = pi/180
rad2degree = 180/pi

class SpotMicroKinematics:
    def __init__(self):
        self.d2r = pi / 180
        self.sm = SpotMicroStickFigure(x=0, y=0.18, z=0, theta=5 * degree2rad)
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
        self.kinematics.sm.set_body_angles(theta=5 * degree2rad)
        coords = self.kinematics.sm.get_leg_coordinates()
        self.ax.set_xlim3d([-0.2, 0.2])
        self.ax.set_zlim3d([0, 0.4])
        self.ax.set_ylim3d([0.2, -0.2])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Z')
        self.ax.set_zlabel('Y')
        self.lines = self.init_lines(coords)

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
        deg_arr = tuple(tuple(round(value * rad2degree) for value in inner_array) for inner_array in arr)
        print("leg_rightback:  ", deg_arr[0])
        print("leg_rightfront: ", deg_arr[1])
        print("leg_leftfront:  ", deg_arr[2])
        print("leg_leftback:   ", deg_arr[3])

    def set_current_angles(self, moving_time, steps, angles_arr, pitch):
        current_angles = self.get_current_angles()
        print("Vorher")
        self.print_angles(current_angles)
        target_angles = [
            [angles_arr[0][0] * degree2rad, -angles_arr[0][1] * degree2rad, angles_arr[0][2] * degree2rad],
            [angles_arr[1][0] * degree2rad, -angles_arr[1][1] * degree2rad, angles_arr[1][2] * degree2rad],
            [angles_arr[2][0] * degree2rad,  angles_arr[2][1] * degree2rad, -angles_arr[2][2] * degree2rad],
            [angles_arr[3][0] * degree2rad,  angles_arr[3][1] * degree2rad, -angles_arr[3][2] * degree2rad]
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

        print("Nachher")
        self.print_angles(self.get_current_angles())
        print("########")

    def get_current_angles(self):
        return self.kinematics.sm.get_leg_angles()

    def get_current_coords(self):
        return self.kinematics.sm.get_leg_coordinates()
        
    def anim(self, mypoints, pitch):
        self.kinematics.sm.set_absolute_foot_coordinates(mypoints)
        self.kinematics.sm.set_body_angles(theta=pitch * self.kinematics.d2r)
        coords = self.get_current_coords()
        self.update_lines(coords)

    def pose(self, moving_time, steps, leg_angles, pitch):
        self.set_current_angles(moving_time, steps, leg_angles, pitch)
        
    def twist(self, roll, pitch, yaw):
        self.kinematics.sm.set_body_angles(
                                theta=pitch * self.kinematics.d2r,
                                phi=roll * self.kinematics.d2r,
                                psi=yaw * self.kinematics.d2r)
                                
        coords = self.get_current_coords()
        self.update_lines(coords)
    

if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.set_zlabel('Y')

    walker = SpotMicro(ax)
    
    roll=17
    pitch=17
    yaw=17
    walker.twist(roll, pitch, yaw)
    plt.pause(11)
    
    # Example poses
    # stand
    # rear-right, front-right, front-left, rear-left
    angles = [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]]
    moving_time = 0.5
    steps = 20
    pitch = 5
    walker.pose(moving_time, steps, angles, pitch)
    plt.pause(1)

    # rest
    angles = [[0, 80, 135], [0, 80, 135], [0, 80, 135], [0, 80, 135]]
    moving_time = 1
    steps = 20
    pitch = 5
    walker.pose(moving_time, steps, angles, pitch)
    plt.pause(1)

    # sit
    angles = [[0, 80, 135], [0, 35, 60], [0, 35, 60], [0, 80, 135]]
    moving_time = 1
    steps = 20
    pitch = 5
    walker.pose(moving_time, steps, angles, pitch)
    plt.pause(1)

    # pee
    angles = [[0, 35, 60], [0, 35, 60], [0, 35, 60], [-50, 80, 135]]
    moving_time = 1
    steps = 20
    pitch = 5
    walker.pose(moving_time, steps, angles, pitch)
    plt.pause(1)
    
    def pushup(cnt):
        moving_time = 1
        steps = 20
        pitch = 5
        poses = [
            [[0, 35, 60],  [0, 35, 60],  [0, 35, 60],  [0, 35, 60]],
            [[0, 80, 135], [0, 80, 135], [0, 80, 135], [0, 80, 135]]
        ]
        # Iterate through poses
        for _ in range(cnt):
            for angles in poses:
                walker.pose(moving_time, steps, angles, pitch)
                plt.pause(0.25)
    pushup(7)
    
    def wave(wavecnt):
        moving_time = 1
        steps = 20
        pitch = 5
        poses = [
            # hi0
            [[0, 80, 135], [0, 35, 60], [0, -45, 60], [0, 80, 135]],
            # hi1
            [[0, 80, 135], [0, 35, 60], [0, -45, 50], [0, 80, 135]]
        ]
        # Iterate through poses
        for _ in range(wavecnt):
            for angles in poses:
                walker.pose(moving_time, steps, angles, pitch)
    wave(7)            
    
    plt.pause(10)

