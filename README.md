# pyro-reloaded
yet another robodog  
Pure python, without ROS.

1-sim-poses-without-walk.py:    
Simulation using matplotlib, basic tests posing only

2-sim-poses-with-walk:  
Simulation using matplotlib, with different walk gaits  
Demo mode: python 2-sim-poses-with-walk.py d  

3-sim-with-controller:  
Controllable with Xbox-Controller for basic poses, no walk yet, includes demo mode

pyro_anim:  
Controllable with Xbox Controller for basic poses and different walk gaits  
Demo mode without Xbox Controller: python pyro_anim.py --mode demo  

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

<img src="pyro.gif" width="400" height="300" />

pyro_real:
includes servo motor control - tbd -
