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
from pyPS4Controller.controller import Controller
import threading
import queue
event_queue = queue.Queue()
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

    def pose(self, moving_time, steps, leg_angles, pitch):
        self.stopwalk = not self.stopwalk
        time.sleep(1)
        self.set_current_angles(moving_time, steps, leg_angles, pitch)
        
    def twist(self, roll, pitch, yaw):
        self.kinematics.sm.set_body_angles(
                                theta=pitch * self.kinematics.d2r,
                                phi=roll * self.kinematics.d2r,
                                psi=yaw * self.kinematics.d2r)
                                
        coords = self.get_current_coords()
        self.update_lines(coords)
        
          
    # --------- walk ---------
    
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
            # just dreaming ..self.
            phase_time = 1 / 2
            offsets = [0, 0, phase_time, phase_time]
            return offsets
    
    def getpositions(self, positions, name, desired_p4_points, steps, radius, start_time_offset, swing_height, swing_time_ratio):
        swing_time = 2 * swing_time_ratio
        stance_time = 2 * (1 - swing_time_ratio)
        swing_steps = int(steps * swing_time_ratio)
        stance_steps = steps - swing_steps

        for t in np.linspace(0, 2, steps):
            t_mod = (t + start_time_offset) % 2
            if t_mod < stance_time / 2:
                x, y = self.linear_motion(t_mod, stance_time / 2, 0, -radius / 2)
            elif t_mod < stance_time / 2 + swing_time:
                x, y = self.point_on_ellipse(t_mod - stance_time / 2, swing_time, radius, swing_height, reverse=True)  # Reverse direction
            else:
                x, y = self.linear_motion(t_mod - stance_time / 2 - swing_time, stance_time / 2, radius / 2, 0)

            if name not in positions:
                positions[name] = []
            positions[name].append((x, y))
            
        return positions
    
 
    def point_on_ellipse(self, t, T, radius, swing_height, reverse=False):
        angle = np.pi * t / T
        if reverse:
            angle = np.pi - angle  # Reverse direction
        x = (radius / 2) * np.cos(angle)
        y = swing_height * np.sin(angle)
        return x, y

    def linear_motion(self, t, T, start_pos, end_pos):
        return start_pos + (end_pos - start_pos) * (t / T), 0

    def walk(self, total_time, repetitions, radii, steps, gait_pattern, overlap_times, swing_heights, swing_time_ratios):
        leg_names = ["front_right", "back_right", "back_left", "front_left"]
        
        self.total_time = total_time
        self.stepwidth = radii
        
        start_time_offsets = self.calculate_start_time_offsets (gait_pattern)
       
        for n in range(repetitions):
            positions = {}
            for i, leg_name in enumerate(leg_names):
                print (steps,"self.total_time: ",self.total_time)       
                if self.stepwidth[i] < 0.08:  #limit
                    positions = self.getpositions(positions, leg_name, self.kinematics.desired_p4_points[i], steps, self.stepwidth[i], start_time_offsets[i], swing_heights[i], swing_time_ratios[i])
                else:
                    self.stepwidth[i] = 0.08
                    positions = self.getpositions(positions, leg_name, self.kinematics.desired_p4_points[i], steps, self.stepwidth[i], start_time_offsets[i], swing_heights[i], swing_time_ratios[i])
                    
            if self.stopwalk:
                break
            print (steps,"self.stepwidth: ",self.stepwidth[i])            
            start_time = time.time()
            for step in range(steps):
                mypoints = []
                for i, leg_name in enumerate(leg_names):
                    p = positions[leg_name][step]
                    new_point = [self.kinematics.desired_p4_points[i][0] + p[0],
                                 self.kinematics.desired_p4_points[i][1] + p[1],
                                 self.kinematics.desired_p4_points[i][2]]
                    mypoints.append(new_point)
                
                self.anim(mypoints, pitch=10.0)
                elapsed = time.time() - start_time
                time.sleep(max(0, self.total_time / steps - elapsed))
                start_time = time.time()

    def update_speed(self, adjustment):
        self.max = 0.1  # short time leads to more speed
        self.min = 4
        self.total_time += adjustment
        if self.total_time > self.min:
             self.total_time = 3.9
        if self.total_time < self.max:
             self.total_time = 0.11
             
       
        if self.total_time < self.max:
             self.total_time = 0.11
        if self.total_time > self.min:
             self.total_time = 3.9
            
        print(f"Updated total_time: {self.total_time}")
        
        
    def update_stepwidth(self, adjustment):
        self.max = 0.10
        
        tmp_array = []
        for x in self.stepwidth:
            tmp_array.append(x + adjustment)
            self.stepwidth = tmp_array
        print(f"Updated update_stepwidth: {self.stepwidth}")
                
    def demo(self):
        total_time = 1 
        repetitions = 2
        steps = 60  # Increase the number of steps for smoother animation
        
        # rear-right, front-right, front-left, rear-left
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
        roll = 17
        pitch = 17
        yaw = 17
        self.twist(roll, pitch, yaw)
        plt.pause(1)
        
        # Example poses
        # Stand
        print("Stand")
        angles = [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]]
        moving_time = 0.5
        steps = 20
        pitch = 5
        self.pose(moving_time, steps, angles, pitch)
        plt.pause(1)

        # Rest
        print("Rest")
        angles = [[0, 80, 135], [0, 80, 135], [0, 80, 135], [0, 80, 135]]
        moving_time = 1
        steps = 20
        pitch = 5
        self.pose(moving_time, steps, angles, pitch)
        plt.pause(1)

        # Sit
        print("Sit")
        angles = [[0, 80, 135], [0, 35, 60], [0, 35, 60], [0, 80, 135]]
        moving_time = 1
        steps = 20
        pitch = 5
        self.pose(moving_time, steps, angles, pitch)
        plt.pause(1)

        # Pee
        print("Pee")
        angles = [[0, 35, 60], [0, 35, 60], [0, 35, 60], [-50, 80, 135]]
        moving_time = 1
        steps = 20
        pitch = 5
        self.pose(moving_time, steps, angles, pitch)
        plt.pause(1)
        
        print("Pushups")
        def pushup(cnt):
            moving_time = 1
            steps = 20
            pitch = 5
            poses = [
                [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]],
                [[0, 80, 135], [0, 80, 135], [0, 80, 135], [0, 80, 135]]
            ]
            # Iterate through poses
            for _ in range(cnt):
                for angles in poses:
                    self.pose(moving_time, steps, angles, pitch)
                    plt.pause(0.25)
        pushup(3)
        plt.pause(1)
        
        print("Greet")
        def wave(wavecnt):
            moving_time = 1
            steps = 20
            pitch = 5
            poses = [
                [[0, 80, 135], [0, 35, 60], [0, -45, 60], [0, 80, 135]],
                [[0, 80, 135], [0, 35, 60], [0, -45, 50], [0, 80, 135]]
            ]
            # Iterate through poses
            for _ in range(wavecnt):
                for angles in poses:
                    self.pose(moving_time, steps, angles, pitch)
        wave(3)            
        
        #plt.pause(10)


class MyController(Controller):
    def __init__(self, walker, interface, connecting_using_ds4drv):
        self.walker = walker
        super().__init__(interface=interface, connecting_using_ds4drv=connecting_using_ds4drv)
    
    
        self.pitch = 0
        self.yaw = 0
        self.roll = 0
        
        self.state = {
            'stand': True,
            'sit': False,
            'rest': False,
            'walk': False,
            'greet': False,
            'pee': False,
            'pushups': False,
            'twist': False,
        }

    '''    
    def on_square_press(self):
        self.pitch += 1
        self.walker.twist(self.roll, self.pitch, self.yaw)
        #self.walker.demo()
    '''
    '''
    def adjust_speed(self, total_time):
        repetitions = 111
        steps = 15
        radii = [0.045, 0.045, 0.045, 0.045] 
        overlap_times = [0.0, 0.0, 0.0, 0.0]
        swing_heights = [0.03, 0.03, 0.03, 0.03]
        swing_time_ratios = [0.25, 0.25, 0.25, 0.25]
        self.walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios)
    '''
    
    def on_triangle_press(self):
        print ("on_triangle_press")
        
        if not self.state['walk']:
            
            event_queue.put("triangle")
            self.state['walk'] = True
            self.walker.stopwalk = not self.walker.stopwalk
           
        else:
        
            self.walker.pose(0.5, 20, [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]], 5)
            self.state['walk'] = False
            
    def on_L1_press(self):
        print ("on_L1_press: steppwith")
        stepwidthdelta = 0.005
        # speed up
        if self.state['walk']:
            self.walker.update_stepwidth(stepwidthdelta)
            print("increase_stepwidth")
            
    def on_R1_press(self):
        print ("on_R1_press: steppwith")
        stepwidthdelta = 0.005
        # speed up
        if self.state['walk']:
            self.walker.update_stepwidth(-stepwidthdelta)
            print("decrease_stepwidth")
    
    
    '''
    on_up_arrow_press
    on_up_down_arrow_release
    on_down_arrow_press
    on_up_down_arrow_release
    on_left_arrow_press
    on_left_right_arrow_release
    on_right_arrow_press
    on_left_right_arrow_release
    '''
    def on_up_down_arrow_release(self):
        pass

    def on_left_right_arrow_release(self):
        pass   
     
     
    def on_up_arrow_press(self):
        delta = 0.5
        # speed up
        if self.state['walk']:
            print ("on_up_arrow_press: ", self.state['walk'])
            self.walker.update_speed(-delta)
            print("increase_speed")
             
    def on_down_arrow_press(self):
        delta = 0.5
        # speed down
        if self.state['walk']:
            print ("on_down_arrow_press: ", self.state['walk'])
            self.walker.update_speed(+delta)
            print("decrease_speed")
        
    
    
    def on_right_arrow_press(self):
        # turn right
        pass
        
    def on_left_arrow_press(self):
        # turn left
        pass
    
    def on_triangle_release(self):
        pass
        
        
    def on_circle_press(self):
        # Stand
        if not self.state['stand']:
            angles = [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]]
            
            moving_time = 0.5
            steps = 20
            pitch = 5
            self.walker.pose(moving_time, steps, angles, pitch)
            self.state['walk'] = False
        else:
            self.walker.pose(0.5, 20, [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]], 5)
        self.state['stand'] = not self.state['stand']  
        
    def on_square_press(self):
        #sit
        if not self.state['sit']:
            angles = [[0, 80, 135], [0, 35, 60], [0, 35, 60], [0, 80, 135]]
            moving_time = 1
            steps = 20
            pitch = 5
            self.walker.pose(moving_time, steps, angles, pitch)
            self.state['walk'] = False
        else:
            self.walker.pose(0.5, 20, [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]], 5)
        self.state['sit'] = not self.state['sit']
        
    def on_x_press(self):
        #rest
        if not self.state['rest']:
            angles = [[0, 80, 135], [0, 80, 135], [0, 80, 135], [0, 80, 135]]
            moving_time = 2
            steps = 20
            pitch = 5
            self.state['walk'] = False
            self.walker.pose(moving_time, steps, angles, pitch)
        else:
            self.walker.pose(0.5, 20, [[0, 35, 60], [0, 35, 60], [0, 35, 60], [0, 35, 60]], 5)
        self.state['rest'] = not self.state['rest']
        
    
    
    def on_L3_up(self, value):
        pass
    
    def on_R3_up(self, value):
        pass
        
    def on_R3_down(self, value):
        pass
        
    def on_L3_down(self, value):
        pass
        
    def on_L3_right(self, value):
        pass
        
    def on_R3_right(self, value):
        pass
    
    def on_L3_left(self, value):
        pass
    
    def on_R3_y_at_rest(self):
        pass
        
    def on_L3_x_at_rest(self):
        pass
           
    def on_L3_y_at_rest(self):
        pass
        
    def on_R3_y_at_rest(self):
        pass
    
    def on_R3_left(self, value):
        pass
        
    def on_R3_x_at_rest(self):
        pass
        
    def on_playstation_button_release(self):
        os._exit(1)
 
 
def start_controller(walker, c):
    controller = c
    controller.listen()
    
def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    walker = SpotMicro(ax)
    
    if len(sys.argv) == 2:
        arg = sys.argv[1]
        
        if arg == "d":
            walker.demo()
    else:
        controller = MyController(walker=walker, interface="/dev/input/js0", connecting_using_ds4drv=False)
        controller_thread = threading.Thread(target=start_controller, args=(walker,controller))
        controller_thread.start()
        walker.initplot()
        
        print ("          A: walk")
        print ("[]: sit              O: stand")
        print ("          X: rest")
        print ("-----------------------------")
        print ("          up: faster")
        print ("<-: left            ->: right")
        print ("        down: slower")
        print ("-----------------------------")
        print ("        P: exit")
         
        print ("     Arrow up: step faster")
        print ("Arrow left: tbd  Arrow right: tbd")
        print ("    Arrow down: step slower")
        
        print ("Trigger Left: step wider  Trigger Right: step narrower")

        while True:
            if not event_queue.empty():
                event = event_queue.get()
                print("increase_speed", event)
                if event == "triangle":
                    print("Processing triangle button event", controller.state['walk'])
                    
                    print("self.walker.stopwalk:",walker.stopwalk)
                    print("state:",'walk',controller.state['walk'])
                    
                    total_time = 2 
                    repetitions = 111
                    steps = 30
                    radii = [0.00, 0.00, 0.00, 0.00] 
                    overlap_times = [0.0, 0.0, 0.0, 0.0]
                    swing_heights = [0.03, 0.03, 0.03, 0.03]
                    swing_time_ratios = [0.25, 0.25, 0.25, 0.25]
                    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios)          
            plt.pause(0.01)        

if __name__ == "__main__":
    main()
