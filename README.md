[//]: # (Image References)
[debugging]: ./writeup/debugging.png
[car_video]: ./writeup/path_planning.mov


# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## Simulator.
Term3 Simulator which contains the Path Planning Project can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

## Point Generation

```CarDriver::GenerateTrajectory```

Car's current speed is estimated by either using cur_speed_mph sent by the simulator or in case previous points are sent, then the difference between the last two points is considered as the speed of the car.

All previous points sent by the simulator are used and more are added to complete 50 points. A spline is generated using the last two points of the previous set and then adding three more points, which are 30, 60, and 90 meters away from the car's current position. The spline is generated in the body frame of reference.

A function ```GetPerPointSpeed``` is called to figure out at each of the 50 points, what should be the speed of the car. This function takes into account maximum acceleration allowed.

In body frame of reference, 50 points are generated, which basically tell where the car would be at each 20ms interval starting from current x,y coordinate. The current x,y is considered as 0,0 and then each consecutive point is set at a distance, which is proportional to the speed of the car for that section. Speed is converted into meters per sec and then multiplied by 0.02 (20ms Δt) to figure out the distance car will travel in 20ms. Y coordinate is figured out by passing the X coordinate to the generated spline. 

The points are eventually converted from body frame of reference to global x,y coordinate space.

```
    for (int i = 0; i < 50 - prev_size; i++) {

        const auto distance_travelled = speed_mph_to_mtr_per_sec(speed_between_points) * 0.02;

        x_from_origin += distance_travelled;
        double y_from_origin = path_spline(x_from_origin);

        // translate from car system to world
        auto x_point = x_from_origin * cos(model_.ref_yaw) - y_from_origin * sin(model_.ref_yaw);
        auto y_point = x_from_origin * sin(model_.ref_yaw) + y_from_origin * cos(model_.ref_yaw);
        x_point += model_.ref_prev.x;
        y_point += model_.ref_prev.y;

        next_x_vals_.push_back(x_point);
        next_y_vals_.push_back(y_point);
    }
```

## Jerk Avoidance

Each timestep, previous points sent by the simulator are kept and some new points are generated. Spline is calculated by adding three more points on top of the last two points from the previous set. If there is no previous set then the car's current point and yaw angle is used for calculating two reference points on top of, which three more points are added. 

```
tk::spline CarDriver::GetPathToFollow(int start_distance, int increment) {

    spline_pts_x.push_back(model_.ref_prev_prev.x);
    spline_pts_x.push_back(model_.ref_prev.x);
    spline_pts_y.push_back(model_.ref_prev_prev.y);
    spline_pts_y.push_back(model_.ref_prev.y);

    auto lane_no_d = GetDesiredFrenetLaneNo(desired_lane_no_);

    FrenetPoint wp[3] = {{model_.car_s + start_distance, lane_no_d},
                         {model_.car_s + start_distance + increment, lane_no_d},
                         {model_.car_s + start_distance + increment * 2, lane_no_d}};
```

From the previous points, last speed is calculated and then new points are generated using the spline. For each new point, the X coordinate passed to the spline is generated by taking into account the speed at which the car would be going at that particular point. Then the speed is increased in a way to make sure that maximum acceleration is never exceeded. Same formula is used for deceleration.

Jerk in changing lanes is avoided by keeping the spline points at a farther distance and by never changing two lanes at one time.

```
const auto lane_change_speed_mph = min(ideal_speed_mph_, MAX_LANE_CHANGE_SPEED_MPH);
DriveAtSpeed(lane_change_speed_mph, 50, 40);

// 50 is the first point generated
// 40 is increment
```

## Collision Avoidance

When a decision has been made to change lanes, distance to car in front (in the target lane) is considered and if it is thought to be very close then the car is slowed down and we wait for the distance to get better. Similarly, for any vehicle behind our car (in the target lane) its speed is calculated and an estimate of where it would be after 7 seconds is calculated. If that point happens to be ahead of our estimated position after 7 seconds then we wait for the car to pass us.

7 seconds was chosen by experiment.

*Shortcoming*: Idealling would have liked to implement MPC to correctly figure out where we would be and where the other car would exactly be and then see if a collision would occur or not.

## Overview of classes

**CarDriver** - Most of the code of driving is in here   
**CostCalculator** - Cost calculation is in this class   
**CarModel** - Structures defined to keeping Cartesian, Frenet points, Vehicle Sensed and our own car's estimated mathematical model.

## Cost Functions

**Catchup Speed:** The main cost function uses the idea of how soon will we be able to catch up with the car in front. This takes into account other car's speed, our speed and the distance between the two cars. 

```
    auto distance_meters = GetDistaneToCarMeters(closest_car);
    auto our_speed_mps = speed_mph_to_mtr_per_sec(*speed_mph);
    auto other_car_speed_mps = closest_car->speed_mps();
    auto delta_speed = our_speed_mps - other_car_speed_mps;

    auto catchup_seconds = distance_meters / delta_speed;
    const auto SECS_TO_ZERO_COST = 6.0;
    auto catchup_seconds_percent = catchup_seconds / SECS_TO_ZERO_COST;

    auto cost = exp(-abs(catchup_seconds_percent));
```


**Lane Speed**: This uses the speed of the lane versus our speed to calculate the cost. This was mostly used to figure out the relative cost of staying in the lane:

```
    double cost = (*speed_mph - lane_speed) / *speed_mph;
```

Although, not directly related but another factor that was used for slowing down was based on **dead stop** calculation, whereby the car slows down keeping in mind the meters it will require to come to halt at the given speed it is travelling at.


## Driving States

Although a FSM has been implemented but I am not happy with the way it has been defined. A lot of code had to be replicated in different states e.g. in "lane change" and "prepare for lane change" had to keep checking if the car in front had moved out

### Keep Lane State

Code defined in: ```void CarDriver::KeepLaneState()```

If there is no car in front, keep going at desired_speed. In case there is a car then change state to PrepareLaneChange, while mainting distance from the car in front. 

### Prepare Lane Change State

Code defined in: ```void CarDriver::PrepareLaneChangeState()```

Uses distance versus speed to the car in front to figure out, which lane would be best to stay in. If it decides to stay in the current lane then the speed of the car in front is compared with how long it will take us to come to dead stop and then speed is decreased if the distance to the car infront is less.

In case the car in front in the current lane moves out, state is changed back to Keep In Lane.

### Change Lane State

Code defined in: ```void CarDriver::ChangeLaneState(int target_lane_no, int best_lane_no)```

Checks are made to make sure lane can be switched safely. There should be enough distance (twice the distance it takes us to come to dead stop) from the car in front in the target lane. To the car behind in the target lane, its speed is calculated and its position is projected into future and then our estimated position in future is compared to make sure that the cars won't collide.

*Note: Since distance calculation is not accurate, there are times when visually one can see ample space between us and the car behind in the target lane but still the car wouldn't change lanes*

### Shortcomings

1) Distance calcualtion is not very accurate. Earlier I had tried using XY body frame to calculate distance from the car to other vehicles but that wasn't accurate either:

```
CartesianPoint car_in_body_frame = model_.TranslateXYToBodyFrame(car->x, car->y);
const auto distance_to_car = sqrt(car_in_body_frame.x * car_in_body_frame.x + car_in_body_frame.y * car_in_body_frame.y);
return distance_to_car;

```

I've settled on car_s sent by the simulator. However on curves this is not giving correct values.

```
double CostCalculator::GetDistaneToCarMeters(const VehicleSensed *car) {
    return car->s - model_.car_s;
}
```

2) Not at all happy with the way overall code has been written. It seems more like an if-then-else statements in each of the state functions. I would have rather followed the idea presented in the lectures that is to generate trajectories and then rate them on the basis of cost. e.g. instead of putting in checks for speeding up or slowing down for cars that are ahead / behind in the target lane, I would have liked to generate different trajectories, one for speeding up, one for slowing down, one for changing 2 lanes together etc. and then find out, which is more appropriate. However, I couldn't come up with a suitable formula to find out intersections between trajectories and other cars present on the road. Specially when the distance to other vehicle is not at all accurate.

3) Emergency brakes have not been implemented. The driver always gives preference to non-jerks but some times car in front just stops in which case the car should also try to stop immediately.

4) Rather than just considering the x,y (or s/d coordinate) of each car, its behaviour should be predicted and then it should be compared to see whether it is following the prediction or not. If it is not following the prediction, whatever decision we have taken should be re-evaluated.

5) Rather than using an overall finite state machine, would have liked to implement a rule based system on top of FSM, where each finite state's outcome should have been checked against some rule  e.g. not too close to other cars, not over speeding etc.

## Debugging

Since the simualator makes it really hard to visualize the decisions, such as where other car(s) are, their distances etc., I wrote a separeate program to view all cars on the track. However, due to shortage of time, it still requires some more effort to finish off.

Code has #ifdef _DEBUG_DATA to send debugging infromation over websockets to connected client on 4568 port

![debugging]