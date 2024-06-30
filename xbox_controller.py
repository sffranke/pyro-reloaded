import os

from pyPS4Controller.controller import Controller
#import matplotlib
#matplotlib.use('tkagg')
import time
import numpy as np
#import queue
from shared_event_queue import event_queue
import json

def load_config():
    with open('config.json', 'r') as config_file:
        return json.load(config_file)

config = load_config()

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
            'walksideways': False,
        }
        
        self.left_joystick_x = 0
        self.left_joystick_y = 0
        self.last_update_time = time.time()
        self.dead_zone = 0.2

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
        stepwidthdelta = 0.015
        # speed up
        if self.state['walk']:
            self.walker.update_stepwidth(stepwidthdelta)
            print("increase_stepwidth")
            
    def on_R1_press(self):
        print ("on_R1_press: steppwith")
        stepwidthdelta = 0.015
        # speed up
        if self.state['walk']:
            self.walker.update_stepwidth(-stepwidthdelta)
            print("decrease_stepwidth")
    
    def on_up_down_arrow_release(self):
        pass

    def on_left_right_arrow_release(self):
        pass   
     
    def on_L3_press(self):
        if not self.state['walk']:
            delta = 0.01
            # increase height
            self.walker.update_height(delta)
      
    def on_R3_press(self):
        if not self.state['walk']:
            delta = 0.01
            # decrease height
            self.walker.update_height(-delta)  
    
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
        # walk walk_sideways right
        if not self.state['walk']:
            
            event_queue.put("on_right_arrow_press")
            self.state['walk'] = True
            self.walker.stopwalk = not self.walker.stopwalk
           
        else:
        
            self.walker.pose(0.5, 20, config['stand_angles'], 5)
            self.state['walk'] = False
        
    def on_left_arrow_press(self):
        if not self.state['walk']:
            
            event_queue.put("on_left_arrow_press")
            self.state['walk'] = True
            self.walker.stopwalk = not self.walker.stopwalk
           
        else:
        
            self.walker.pose(0.5, 20, config['stand_angles'], 5)
            self.state['walk'] = False
        
    
    def on_triangle_release(self):
        pass
        
        
    def on_circle_press(self):
        # Stand
        angles = config['stand_angles']
        if not self.state['stand']:
            moving_time = 0.5
            steps = 20
            pitch = 5
            self.walker.pose(moving_time, steps, angles, pitch)
            self.state['walk'] = False
        else:
            self.walker.pose(0.5, 20, angles, 5)
        self.state['stand'] = not self.state['stand']  
        
    def on_square_press(self):
        #sit
        if not self.state['sit']:
            angles = config['sit_angles']
            moving_time = 1
            steps = 20
            pitch = 5
            self.walker.pose(moving_time, steps, angles, pitch)
            self.state['walk'] = False
        else:
            self.walker.pose(0.5, 20, config['stand_angles'], 5)
        self.state['sit'] = not self.state['sit']
        
    def on_x_press(self):
        #rest
        if not self.state['rest']:
            angles = config['rest_angles']
            moving_time = 2
            steps = 20
            pitch = 5
            self.state['walk'] = False
            self.walker.pose(moving_time, steps, angles, pitch)
        else:
            self.walker.pose(0.5, 20, config['stand_angles'], 5)
        self.state['rest'] = not self.state['rest']
        
    
    def on_L3_up(self, value):
        self.left_joystick_y = value
        self.update_angle()

    def on_L3_down(self, value):
        self.left_joystick_y = value
        self.update_angle()

    def on_L3_left(self, value):
        self.left_joystick_x = value
        self.update_angle()

    def on_L3_right(self, value):
        self.left_joystick_x = value
        self.update_angle()

    def on_R3_up(self, value):
        pass
        
    def on_R3_left(self, value):
        pass
        
    def on_R3_right(self, value):
        pass
        
    def on_R3_down(self, value):
        pass
        
    def on_share_press(self):
        print ("rotate right")
        if not self.state['walk']:
            
            event_queue.put("rotate_left")
            self.state['walk'] = True
            self.walker.stopwalk = not self.walker.stopwalk
        
    def on_options_press(self):
        print ("rotate right") 
        if not self.state['walk']:
            
            event_queue.put("rotate_right")
            self.state['walk'] = True
            self.walker.stopwalk = not self.walker.stopwalk

    def update_angle(self):
        current_time = time.time()
        if current_time - self.last_update_time >= 1:
            angle = self.get_joystick_angle(self.left_joystick_x, self.left_joystick_y)
            print(f"The angle is {angle:.2f} degrees")

            self.walker.update_angle(angle)
            self.last_update_time = current_time

    def get_joystick_angle(self, x, y):
        # Normalize x and y to the range [-1, 1]
        x = x / 32767.0
        y = y / 32767.0

        # Calculate the magnitude of the joystick position
        magnitude = np.sqrt(x**2 + y**2)

        # Apply dead zone: if the magnitude is within the dead zone, consider the angle as 0°
        if magnitude < self.dead_zone:
            return 0.0

        # Calculate the angle in radians using numpy
        angle_radians = np.arctan2(y, x)

        # Convert the angle to degrees using numpy
        angle_degrees = np.degrees(angle_radians)

        # Adjust the angle to match the desired reference frame
        # Up is 0°
        # Left is -90°
        # Right is +90°
        # Down is 180°
        angle_degrees = (angle_degrees + 90) % 360  # Rotate 90 degrees clockwise

        # Ensure the angle is in the range [-180, 180)
        if angle_degrees > 180:
            angle_degrees -= 360

        return angle_degrees

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
 
 
