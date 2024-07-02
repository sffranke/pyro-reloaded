import os
import sys
import json
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
from shared_event_queue import event_queue

from spot_micro_kinematics import SpotMicroKinematics

plt.ion()

from spotmicro import SpotMicro
from xbox_controller import MyController

import enum

class State(enum.Enum):
    WALK = "walk"
    WALKSIDEWAYS = "walksideways"
    SIT = "sit"
    STAND = "stand"
    REST = "rest"

current_state = State.STAND

def load_config():
    with open('config.json', 'r') as config_file:
        return json.load(config_file)

config = load_config()


def transition_to(walker, target_state):
    global current_state
    print("Transitioning to", target_state, "from", current_state)
    if target_state == current_state:
        return 
        
    if current_state != State.WALK and target_state == State.WALK:
        current_state = State.WALK
        angles = config['stand_angles']
        walker.pose(0.5, 20, angles, 5)
        #time.sleep(0.5)
        walker.walk(config['total_time'], config['repetitions'], config['radii'], config['steps'], "wave", config['overlap_times'], config['swing_heights'], config['swing_time_ratios'], np.deg2rad(0))
        return

    if current_state == State.WALK and target_state != State.STAND:
        current_state = target_state
        angles = config['stand_angles']
        walker.pose(0.5, 20, angles, 5)
        if target_state == State.STAND:
            angles = config['stand_angles']
            walker.pose(0.5, 20, angles, 5)
        elif target_state == State.REST:
            angles = config['rest_angles']
            walker.pose(0.5, 20, angles, 5)
        elif target_state == State.SIT:
            angles = config['sit_angles']
            walker.pose(0.5, 20, angles, 5)
        return

    # Direct transitions for all states except from WALK to another state without going through STAND
    if target_state == State.STAND:
        angles = config['stand_angles']
        walker.pose(0.5, 20, angles, 5)
    elif target_state == State.REST:
        angles = config['rest_angles']
        walker.pose(0.5, 20, angles, 5)
    elif target_state == State.SIT:
        angles = config['sit_angles']
        walker.pose(0.5, 20, angles, 5)

def start_controller(walker, c):
    controller = c
    controller.listen()
    
def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    walker = SpotMicro(ax)
    global current_state
    
    if len(sys.argv) == 2:
        arg = sys.argv[1]
        
        if arg == "d":
            walker.demo()
    else:
        controller = MyController(walker=walker, interface="/dev/input/js0", connecting_using_ds4drv=False)
        controller_thread = threading.Thread(target=start_controller, args=(walker,controller))
        controller_thread.start()
        walker.initplot()
        
        print ("           A: walk")
        print ("[]: sit              O: stand")
        print ("           X: rest")
        print ("-----------------------------")
        print ("          up: faster")
        print ("        down: slower")
        print ("-----------------------------")
        print ("           P: exit")
        print ("    Arrow up: step faster")
        print ("  Arrow down: step slower")
        print ("Trigger Left: step wider  Trigger Right: step narrower")
        print ("L3/R3 press: increase/decrease height")
        print ("share/options press: rotate left/right")
        
        while True:
            if not event_queue.empty():
                event = event_queue.get()
                print("EVENT: ",event)
                
                if event == "rest":
                    print("event rest")
                    transition_to(walker, State.REST)

                if event == "walk":
                    print("event walk")
                    current_state = State.STAND
                    transition_to(walker, State.WALK)

                if event == "sit":
                    print("event sit")
                    transition_to(walker, State.SIT)

                if event == "stand":
                    print("event stand")
                    transition_to(walker, State.STAND)
                    
                if event == "rotate_left":
                    print("Processing rotate_left event")
                    
                    print("self.walker.stopwalk:",walker.stopwalk)
                    
                    total_time = 2 
                    repetitions = 111
                    steps = 30
                    radii = [0.02, 0.02, 0.02, 0.02] 
                    overlap_times = [0.0, 0.0, 0.0, 0.0]
                    swing_heights = [0.03, 0.03, 0.03, 0.03]
                    swing_time_ratios = [0.25, 0.25, 0.25, 0.25]
                    angle = np.deg2rad(0) 
                    walker.walk(total_time, repetitions, radii, steps, "rotate_left", overlap_times, swing_heights, swing_time_ratios,angle)

                if event == "rotate_right":
                    print("Processing rotate_right event")
                    
                    print("self.walker.stopwalk:",walker.stopwalk)
                    
                    total_time = 2 
                    repetitions = 111
                    steps = 30
                    radii = [0.02, 0.02, 0.02, 0.02] 
                    overlap_times = [0.0, 0.0, 0.0, 0.0]
                    swing_heights = [0.03, 0.03, 0.03, 0.03]
                    swing_time_ratios = [0.25, 0.25, 0.25, 0.25]
                    angle = np.deg2rad(0) 
                    walker.walk(total_time, repetitions, radii, steps, "rotate_right", overlap_times, swing_heights, swing_time_ratios,angle)

            plt.pause(0.01)    

if __name__ == "__main__":
    main()
