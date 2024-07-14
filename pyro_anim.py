global mode 
import argparse
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

from spotmicro_anim import SpotMicro
from xbox_controller import MyController
import enum

parser = argparse.ArgumentParser(description='Control the SpotMicro robot.')
parser.add_argument('-m', '--mode', type=str, choices=['calib', 'demo', 'animation', 'angles'], default='animation',
                    help='Select the mode of operation: demo, animation or angles')

args = parser.parse_args()
mode = args.mode

if mode != 'angles':
    plt.ion()


class StateClass:
    from enum import Enum

    class State(Enum):
        WALK = "walk"
        ROTATELEFT = "rotateleft"
        ROTATERIGHT = "rotateright"
        SIT = "sit"
        POOP = "poop"
        STAND = "stand"
        REST = "rest"
        POSE = "pose"

    def __init__(self, state):
        if isinstance(state, self.State):
            self._state = state
        else:
            raise ValueError("Invalid initial state")

    def get_state(self):
        return self._state

    def set_state(self, new_state):
        if isinstance(new_state, self.State):
            self._state = new_state
        else:
            raise ValueError("Invalid state")

##Instantiate and initialize the state
stateobj = StateClass(StateClass.State.STAND)
currentstate = stateobj.get_state()

def load_config():
    with open('config.json', 'r') as config_file:
        return json.load(config_file)

config = load_config()

def transition_to(walker, target_state):
    current_state=stateobj.get_state()
    print("Transitioning to", target_state, "from", current_state)

    walker.stopwalk = True

    if target_state == current_state:
        target_state = stateobj.set_state(stateobj.State.STAND)
        
    if current_state != stateobj.State.WALK and target_state == stateobj.State.WALK:
    
        walker.stopwalk = True
        stateobj.set_state(stateobj.State.WALK)
        angles = config['stand_angles']
        walker.pose(0.5, 20, angles, 5)
        plt.pause(1)
        walker.stopwalk = False
        walker.walk(config['total_time'], config['repetitions'], config['radii'], config['steps'], "wave", config['overlap_times'], config['swing_heights'], config['swing_time_ratios'], np.deg2rad(0))
        

    elif target_state == stateobj.State.ROTATELEFT:
        stateobj.set_state(stateobj.State.ROTATELEFT)
        angles = config['stand_angles']
        walker.pose(0.5, 20, angles, 5)
        walker.stopwalk = False
        walker.walk(config['total_time'], config['repetitions'], config['radii'], config['steps'], "rotate_left", config['overlap_times'], config['swing_heights'], config['swing_time_ratios'], np.deg2rad(0))
    
    elif target_state == stateobj.State.ROTATERIGHT:
        stateobj.set_state(stateobj.State.ROTATERIGHT)
        angles = config['stand_angles']
        walker.pose(0.5, 20, angles, 5)
        walker.stopwalk = False
        walker.walk(config['total_time'], config['repetitions'], config['radii'], config['steps'], "rotate_right", config['overlap_times'], config['swing_heights'], config['swing_time_ratios'], np.deg2rad(0))

    elif target_state == stateobj.State.REST:
        stateobj.set_state(stateobj.State.REST)
        angles = config['rest_angles']
        walker.pose(0.5, 20, angles, 5)

    elif target_state == stateobj.State.SIT:
        stateobj.set_state(stateobj.State.SIT)
        angles = config['sit_angles']
        walker.pose(0.5, 20, angles, 5)

    elif target_state == stateobj.State.POSE:
    
        if walker.stopwalk == False:
            walker.stopwalk = True
            time.sleep(0.3)

        angles = config['stand_angles']
        walker.pose(0.5, 20, angles, 5)

        time.sleep(0.5)
        stateobj.set_state(stateobj.State.POSE)
        
    elif target_state == stateobj.State.POOP:
        walker.stopwalk = True
        time.sleep(1)
        stateobj.set_state(stateobj.State.POOP)
        angles = config['poop_angles']
        walker.pose(0.5, 20, angles, 5)

    elif target_state == stateobj.State.STAND:
        stateobj.set_state(stateobj.State.STAND)
        angles = config['stand_angles']
        walker.pose(0.5, 20, angles, 5)

def start_controller(walker, c):
    controller = c
    controller.listen()

def main():
    global mode
    if mode != 'angles':
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    else:
        ax = None

    walker = SpotMicro(mode, ax)
    current_state = stateobj.get_state()
    print("current_state: ", current_state)
    rundemo = False
    runangles = False

    if mode == 'angles':
        print("Outputting angles instead of running animation...")
        mode = 'angles'
    elif mode == 'demo':
        mode = 'demo'
    elif mode == 'calib':
        mode = 'calib'
    elif mode == 'animation':
        mode = 'animation'
        pass

    if mode == 'demo':
            walker.demo()
            return
    if mode == 'calib':
            walker.calib()
            return
    else:
        controller = MyController(stateobj, walker=walker, interface="/dev/input/js0", connecting_using_ds4drv=False)
        controller_thread = threading.Thread(target=start_controller, args=(walker,controller))
        controller_thread.start()
        walker.initplot()
        
        print ("           A: walk")
        print ("[]: sit              O: stand")
        print ("           X: rest")
        print ("-----------------------------")
        print ("          up: faster")
        print ("Left: poop         Right: pose")
        print ("        down: slower")
        print ("-----------------------------")
        print ("Trigger Left: step wider  Trigger Right: step narrower")
        print ("L3/R3 press: increase/decrease height")
        print ("share/options press: rotate left/right")
        print ("           P: exit")
        
        while True:
            if not event_queue.empty():
                event = event_queue.get()
                current_state = stateobj.get_state()
                #print("EVENT: ",event, "State:", current_state)
                
                if event == "rest":
                    #print("event rest")
                    time.sleep (0.1)
                    transition_to(walker, stateobj.State.REST)
                    stateobj.set_state(stateobj.State.REST)

                if event == "walk":
                    time.sleep (0.1)
                    transition_to(walker, stateobj.State.WALK)
                    stateobj.set_state(stateobj.State.WALK)

                if event == "sit":
                    time.sleep (0.1)
                    transition_to(walker, stateobj.State.SIT)
                    stateobj.set_state(stateobj.State.SIT)

                if event == "poop":
                    time.sleep (0.1)
                    transition_to(walker, stateobj.State.POOP)
                    stateobj.set_state(stateobj.State.POOP)

                if event == "stand":
                    time.sleep (0.1)
                    transition_to(walker, stateobj.State.STAND)
                    stateobj.set_state(stateobj.State.STAND)
                    
                if event == "rotate_left":
                    time.sleep (0.1)
                    transition_to(walker, stateobj.State.ROTATELEFT)
                    stateobj.set_state(stateobj.State.ROTATELEFT)
                    
                if event == "rotate_right":
                    time.sleep (0.1)
                    transition_to(walker, stateobj.State.ROTATERIGHT)
                    stateobj.set_state(stateobj.State.ROTATERIGHT)

                if event == "pose":
                    time.sleep (0.1)
                    transition_to(walker, stateobj.State.POSE)

            plt.pause(0.01)   

if __name__ == "__main__":
    main()
