# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

## Driving States

Although a FSM has been implemented but I am not happy with the way it has been defined. A lot of code had to be replicated in different states e.g. in "lane change" and "prepare for lane change" had to keep checking if the car in front had moved out

### Keep Lane State

Code defined in: ```void CarDriver::KeepLaneState()```

If there is no car in front, keep going at desired_speed. In case there is a car then change state to PrepareLaneChange, while mainting distance from the car in front. 

### Prepare Lane Change State


### Shortcomings

1) Each car's behaviour should be predicted and then a decision should be made (or changed accordingly) as right when we are about to change lanes, car front can also decide to change lanes
2) Rather than using an overall finite state machine, would have liked to implement a rule based system where each finite state's outcome should have been checked against some rule system e.g. not too close to other cars, not over speeding etc. Right now had to put these checks inside a state and had to replicate code in different states for the same thing.
3) Not at all happy with the way overall code has been written. It seems more like an if-then-else statements in each of the state functions. I would have rather followed the idea presented in the lectures that is to generate trajectories and then rate them on the basis of cost. But didn't know how to check if other cars lie on the path of the trajectory or not.

## Code Implementation


## Shortcomings

### Simulator won't allow to run scenarios

### Won't show a predicted path

Would have loved it, if the simulator would allow different lines or paths to be drawn e.g. to check safe distance to move or distance behind / front of the car