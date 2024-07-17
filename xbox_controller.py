import time
import numpy as np
import json
from pyPS4Controller.controller import Controller
from shared_event_queue import event_queue

def load_config():
    with open('config.json', 'r') as config_file:
        return json.load(config_file)

config = load_config()

class MyController(Controller):
    def __init__(self, stateobj, walker, interface, connecting_using_ds4drv):
        super().__init__(interface=interface, connecting_using_ds4drv=connecting_using_ds4drv)
        self.stateobj = stateobj
        self.walker = walker
        self.pitch = 0
        self.yaw = 0
        self.roll = 0
        self.left_joystick_x = 0
        self.left_joystick_y = 0
        self.right_joystick_x = 0
        self.right_joystick_y = 0
        self.last_update_time = time.time()
        self.dead_zone = 0.2

    def on_triangle_press(self):
        print("on_triangle_press")
        event_queue.put("walk")
        self.walker.stopwalk = False

    def on_L3_press(self):
        print ("L3 press", self.walker.stopwalk)
        if self.walker.stopwalk == True:
            delta = 0.01
            # increase height
            self.walker.update_height(delta)
      
    def on_R3_press(self):
        print ("L3 press", self.walker.stopwalk)
        if self.walker.stopwalk == True:
            delta = 0.01
            # decrease height
            self.walker.update_height(-delta)  

    def on_L1_press(self):
        if  self.walker.stopwalk == False:
            self.walker.update_stepwidth(0.015)
            print("increase_stepwidth")

    def on_R1_press(self):
        if  self.walker.stopwalk == False:
            self.walker.update_stepwidth(-0.015)
            print("decrease_stepwidth")

    def on_up_arrow_press(self):
        if  self.walker.stopwalk == False:
            self.walker.update_speed(-0.5)
            print("increase_speed")

    def on_down_arrow_press(self):
        if  self.walker.stopwalk == False:
            self.walker.update_speed(0.5)
            print("decrease_speed")
    
    def on_right_arrow_press(self):
        self.walker.stopwalk = True
        event_queue.put("pose")
   
    def on_left_arrow_press(self):
        self.walker.stopwalk = True
        print("Poop press")
        event_queue.put("poop")

    def on_circle_press(self):
        self.walker.stopwalk = True
        print("Stand press")
        event_queue.put("stand")

    def on_square_press(self):
        self.walker.stopwalk = True
        print("event sit")
        event_queue.put("sit")

    def on_x_press(self):
        self.walker.stopwalk = True
        print("rest press")
        event_queue.put("rest")

    def on_share_press(self):
        self.walker.stopwalk = True
        print("rotate left")
        event_queue.put("rotate_left")

    def on_options_press(self):
        self.walker.stopwalk = True
        print("rotate right")
        event_queue.put("rotate_right")

    def on_playstation_button_release(self):
        event_queue.put("release")
        

    def update_pose(self):
        
        #print ("Stopwalk: ",self.walker.stopwalk,self.stateobj.get_state())
        #if self.walker.stopwalk == False or self.stateobj.get_state() != self.stateobj.State.POSE:
        if self.stateobj.get_state() != self.stateobj.State.POSE:
            #print ("Stopwalk false: ",self.stateobj.get_state())
            return
        
        if self.stateobj.get_state() == self.stateobj.State.POSE:
            current_time = time.time()
            if current_time - self.last_update_time >= config['update_pose_time']:
            
                print(f"Pitch: {self.pitch:.2f}, Roll: {self.roll:.2f}, Yaw: {self.yaw:.2f}")
                self.walker.kinematics.sm.set_body_angles(phi=self.roll*self.walker.kinematics.d2r, 
                                                          theta=(5+self.pitch)*self.walker.kinematics.d2r, 
                                                          psi=self.yaw*self.walker.kinematics.d2r)
                coords = self.walker.get_current_coords()
                self.walker.update_lines(coords)
                #time.sleep(0.002)
                self.last_update_time = current_time
        
    
    # control walking directions
    def update_angle(self):
        if self.stateobj.get_state() == self.stateobj.State.WALK:
            current_time = time.time()
            if current_time - self.last_update_time >= config['update_walkangle_time']:
                angle = self.get_joystick_angle(self.left_joystick_x, self.left_joystick_y)
                #print(f"The angle is {angle:.2f} degrees")
                self.walker.update_angle(angle)
                self.last_update_time = current_time

    def get_joystick_angle(self, x, y):
        x = x / 32767.0
        y = y / 32767.0
        magnitude = np.sqrt(x**2 + y**2)
        if magnitude < self.dead_zone:
            return 0.0
        angle_radians = np.arctan2(y, x)
        angle_degrees = np.degrees(angle_radians) + 90
        if angle_degrees > 180:
            angle_degrees -= 360
        return angle_degrees

    def on_L3_up(self, value): self.left_joystick_y = value; self.update_angle()
    def on_L3_down(self, value): self.left_joystick_y = value; self.update_angle()
    def on_L3_left(self, value): self.left_joystick_x = value; self.update_angle()
    def on_L3_right(self, value): self.left_joystick_x = value; self.update_angle()
    def on_L3_x_at_rest(self): pass
    def on_L3_y_at_rest(self): pass

    def on_R3_up(self, value): 
        #event_queue.put("update_pose")
        self.pitch = config['pose_factor']*value / 32767.0
        self.update_pose()

    def on_R3_down(self, value):
        #event_queue.put("update_pose")
        self.pitch = value*config['pose_factor'] / 32767.0
        self.update_pose()

    def on_R3_left(self, value):
         #event_queue.put("update_pose")
         self.roll = value*config['pose_factor'] / 32767.0
         self.update_pose()

    def on_R3_right(self, value): 
        #event_queue.put("update_pose")
        self.roll = value*config['pose_factor'] / 32767.0
        self.update_pose()

    def on_L2_press(self, value):
        self.yaw = 31079+value*config['pose_factor'] / 32767.0/2
        self.update_pose()
   
    def on_R2_press(self, value):
        #print(value)
        self.yaw = 31079-value*config['pose_factor'] / 32767.0/2
        self.update_pose()

    def on_R3_y_at_rest(self): self.pitch = 0; self.update_pose()
    def on_R3_x_at_rest(self): self.roll = 0; self.update_pose()

    def on_up_down_arrow_release(self): pass
    def on_left_right_arrow_release(self): pass
    def on_triangle_release(self): pass
  
