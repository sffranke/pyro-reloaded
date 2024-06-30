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

def load_config():
    with open('config.json', 'r') as config_file:
        return json.load(config_file)

config = load_config()

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
        print ("L3/R3 press: increase/decrease height")
        print ("share/options press: rotate left/right")
        
        while True:
            if not event_queue.empty():
                event = event_queue.get()
                print("increase_speed", event)
                if event == "triangle":
                    print("Processing triangle button event", controller.state['walk'])
                    
                    print("self.walker.stopwalk:",walker.stopwalk)
                    print("state:",'walk',controller.state['walk'])
                    '''
                    total_time = 2 
                    repetitions = 50
                    steps = 30
                    radii = [0.00, 0.00, 0.00, 0.00] 
                    overlap_times = [0.0, 0.0, 0.0, 0.0]
                    swing_heights = [0.03, 0.03, 0.03, 0.03]
                    swing_time_ratios = [0.25, 0.25, 0.25, 0.25]
                    angle = np.deg2rad(0)  # example angle in radians
                    '''
                    total_time = config['total_time']
                    steps = config['steps']
                    radii = config['radii']
                    overlap_times = config['overlap_times']
                    repetitions = config['repetitions']
                    swing_heights =config['swing_heights']
                    swing_time_ratios =config['swing_time_ratios']
                    angle = np.deg2rad(0) 
                    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios, angle)
                    plt.pause(0.01)
                    
                    plt.show()
                if event == "on_right_arrow_press":
                    print("Processing right_arrow button event", controller.state['walk'])
                    
                    print("self.walker.stopwalk:",walker.stopwalk)
                    print("state:",'walksideways',controller.state['walk'])
                    
                    total_time = 2 
                    repetitions = 111
                    steps = 30
                    radii = [0.02, 0.02, 0.02, 0.02] 
                    overlap_times = [0.0, 0.0, 0.0, 0.0]
                    swing_heights = [0.03, 0.03, 0.03, 0.03]
                    swing_time_ratios = [0.25, 0.25, 0.25, 0.25]
                    angle = np.deg2rad(90) 
                    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios,angle)
                    
                if event == "on_left_arrow_press":
                    print("Processing left button event", controller.state['walk'])
                    
                    print("self.walker.stopwalk:",walker.stopwalk)
                    print("state:",'walksideways',controller.state['walk'])
                    
                    total_time = 2 
                    repetitions = 111
                    steps = 30
                    radii = [0.02, 0.02, 0.02, 0.02] 
                    overlap_times = [0.0, 0.0, 0.0, 0.0]
                    swing_heights = [0.03, 0.03, 0.03, 0.03]
                    swing_time_ratios = [0.25, 0.25, 0.25, 0.25]
                    angle = np.deg2rad(-90) 
                    walker.walk(total_time, repetitions, radii, steps, "wave", overlap_times, swing_heights, swing_time_ratios,angle)
                
                if event == "rotate_left":
                    print("Processing rotate_left event", controller.state['walk'])
                    
                    print("self.walker.stopwalk:",walker.stopwalk)
                    print("state:",'walksideways',controller.state['walk'])
                    
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
                    print("Processing rotate_right event", controller.state['walk'])
                    
                    print("self.walker.stopwalk:",walker.stopwalk)
                    print("state:",'walksideways',controller.state['walk'])
                    
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
