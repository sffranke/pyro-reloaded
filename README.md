# pyro-reloaded
yet another robodog  
Pure python, without ROS.

pyro.py:  
Controllable with Xbox Controller for basic poses and different walk gaits  
Demo mode without Xbox Controller: python3 pyro.py --mode demo  
Real mode with 12 DOF quadruped robot: python3 pyro.py --mode real
```
           A: walk
[]: sit              O: stand
           X: rest

         Arrow up: faster
Left: poop         Right: pose
       Arrow  down: slower
    
Trigger Left: step wider  Trigger Right: step narrower  
L3/R3 press: increase/decrease height  
share/options press: rotate left/right  
P: exit  
```
<img src="pyro.gif" width="400" height="300" />

