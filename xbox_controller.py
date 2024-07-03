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
        
        self.left_joystick_x = 0
        self.left_joystick_y = 0
        self.last_update_time = time.time()
        self.dead_zone = 0.2
        self.updateangle = False
    
    def on_triangle_press(self):
        print ("on_triangle_press")
        self.updateangle = True
        event_queue.put("walk")
        self.walker.stopwalk = False
            
    def on_L1_press(self):
        print ("on_L1_press: steppwith")
        stepwidthdelta = 0.015
        # speed up
        if self.walker.stopwalk == False:
            self.walker.update_stepwidth(stepwidthdelta)
            print("increase_stepwidth")
            
    def on_R1_press(self):
        print ("on_R1_press: steppwith")
        stepwidthdelta = 0.015
        # speed up
        if self.walker.stopwalk == False:
            self.walker.update_stepwidth(-stepwidthdelta)
            print("decrease_stepwidth")
    
    def on_up_down_arrow_release(self):
        pass

    def on_left_right_arrow_release(self):
        pass   
     
    def on_L3_press(self):
        if self.walker.stopwalk == True:
            delta = 0.01
            # increase height
            self.walker.update_height(delta)
      
    def on_R3_press(self):
        if self.walker.stopwalk == True:
            delta = 0.01
            # decrease height
            self.walker.update_height(-delta)  
    
    def on_up_arrow_press(self):
        delta = 0.5
        # speed upself.walker.stopwalk
        if self.walker.stopwalk == False:
            print ("on_up_arrow_press: ")
            self.walker.update_speed(-delta)
            print("increase_speed")    

    def on_down_arrow_press(self):
        delta = 0.5
        # speed down
        if self.walker.stopwalk == False:
            print ("on_down_arrow_press: ")
            self.walker.update_speed(+delta)
            print("decrease_speed")

    def on_right_arrow_press(self):
        return
        # walk walk_sideways right
        if self.walker.stopwalk == False:
            event_queue.put("on_right_arrow_press")
            self.updateangle = False
            #self.walker.stopwalk = not self.walker.stopwalk
           
    def on_left_arrow_press(self):
        self.walker.stopwalk = True
        print("Poop press")
        event_queue.put("poop")
    
    def on_triangle_release(self):
        pass
        
    def on_circle_press(self):
        # Stand
        self.walker.stopwalk = True
        print("Stand press")
        event_queue.put("stand")
    
    def on_square_press(self):
        #sit
        self.walker.stopwalk = True
        print("event sit")
        event_queue.put("sit")
        
    def on_x_press(self):
        #rest
        self.walker.stopwalk = True
        print("rest press")
        event_queue.put("rest")
        
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
        #print("on_R3_up")
        self.right_joystick_y = value
        self.update_pose()

    def on_R3_left(self, value):
        #print("on_R3_left")
        self.right_joystick_x = value
        self.update_pose()
        
    def on_R3_right(self, value):
        #print("on_R3_right")
        self.right_joystick_x = value
        self.update_pose()
        
    def on_R3_down(self, value):
        #print("on_R3_down")
        self.right_joystick_y = value
        self.update_pose()
        
    def on_share_press(self):
        self.walker.stopwalk = True
        print ("rotate left")
        event_queue.put("rotate_left")
        #self.walker.stopwalk = not self.walker.stopwalk
        
    def on_options_press(self):
        self.walker.stopwalk = True
        print ("rotate right") 
        event_queue.put("rotate_right")
        #self.walker.stopwalk = not self.walker.stopwalk

    def update_pose(self):
        
        current_time = time.time()
        if current_time - self.last_update_time >= 0.5:
            #print("update_pose")
            angle = self.get_joystick_angle(self.right_joystick_x, self.right_joystick_y)
            self.walker.update_pose(angle)
            self.last_update_time = current_time

    def update_angle(self):
        current_time = time.time()
        if current_time - self.last_update_time >= 1:
            angle = self.get_joystick_angle(self.left_joystick_x, self.left_joystick_y)
            ##print(f"The angle is {angle:.2f} degrees")
            #if(self.updateangle):
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
 
 