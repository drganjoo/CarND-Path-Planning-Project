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




## Code Implementation


## Shortcomings

### Simulator won't allow to run scenarios

### Won't show a predicted path

Would have loved it, if the simulator would allow different lines or paths to be drawn e.g. to check safe distance to move or distance behind / front of the car