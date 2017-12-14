//
// Created by fahad on 11/29/17.
//

#include "CarDriver.h"
#include "utils.h"
#include <memory>

using namespace std;

static double deg2rad(double x) { return x * M_PI / 180.0; }
static double rad2deg(double x) { return x * 180 / M_PI; }
double distance(double x1, double y1, double x2, double y2);

vector<double> getFrenet(double x, double y, double theta);

CarDriver::CarDriver() {
    end_path_s_ = 0.0;
    end_path_d_ = 0.0;
//    state_ = bind(&CarDriver::KeepLaneState, this);
    state_ = bind(&CarDriver::InitState, this);
}


std::array<std::vector<double>, 2> CarDriver::GetPath() {
    return {next_x_vals_, next_y_vals_};
}


void CarDriver::UpdateModel(json &j) {
    model_ = CarModel(j);
    last_debug_ = unique_ptr<DebugValues>(new DebugValues(model_));

    previous_path_x_ = j["previous_path_x"];
    previous_path_y_ = j["previous_path_y"];
    // Previous path's end s and d values
    end_path_s_ = j["end_path_s"];
    end_path_d_ = j["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_data = j["sensor_fusion"];

    sensor_fusion_.clear();
    for (auto &sensor : sensor_data) {
        VehicleSensed v;
        v.car_id = sensor[0];
        v.x = sensor[1];
        v.y = sensor[2];
        v.x_dot = sensor[3];
        v.y_dot =sensor[4];
        v.s = sensor[5];
        v.d = sensor[6];

//        cout << v.car_id << "," << v.d << "," << v.s << endl;

        sensor_fusion_.push_back(v);
    }

    FigureOutCarOrigin(&model_.ref_prev, &model_.ref_prev_prev, &model_.ref_yaw);
    FigureOutSpeedFromPrevious();

    last_debug_->sensor_fusion = sensor_fusion_;
    last_debug_->debug_message.clear();

    cost_ = unique_ptr<CostCalculator>(new CostCalculator(sensor_fusion_, model_));
    state_();

//    cout << "******************************" << endl;
//
//    for (int i = 0; i < 3; i++) {
//        auto car = cost_->GetClosestCarInFront(i);
//        cout << "Lane: " << i;
//
//        if (car) {
//            cout << " id:" << car->car_id;
//        }
//        else {
//            cout << " NO CAR!!!";
//        }
//
//        cout << endl;
//    }
}


void CarDriver::PrepareLaneChangeState() {
    // possible states from here:
    //  stay here and match traffic speed_mps
    //  change left
    //  change right

    int cur_lane_no = (int) (model_.car_d / 4);
    int best_lane_no = 0;
    double lane_costs[3];
    double lane_speeds[3] = { ideal_speed_mph_, ideal_speed_mph_, ideal_speed_mph_ };

    SpeedDistanceCostCalculator sd_calc(sensor_fusion_, model_);
    SpeedCostCalculator s_calc(sensor_fusion_, model_);

    cout << "Current Lane: " << cur_lane_no << endl;
    cout << "Lane, FC Distance, Our Speed, FC Speed, CU Distance, CU Secs %, Cost" << endl;

    for (int i = 0; i < 3; i++) {
        if (i == cur_lane_no) {
            lane_costs[i] = s_calc.CalculateCost(cur_lane_no, &lane_speeds[i]);
        }
        else {
            lane_costs[i] = sd_calc.CalculateCost(i, &lane_speeds[i]);
        }

        if (lane_costs[i] < lane_costs[best_lane_no])
            best_lane_no = i;
    }

    // TEMP
//    best_lane_no = desired_lane_no_;

    if (desired_lane_no_ == best_lane_no) {
        bool car_close = DriveWhileKeepingDistance();
        if (!car_close)
            state_ = bind(&CarDriver::KeepLaneState, this);

        last_debug_->desired_speed_mph = lane_speeds[best_lane_no];
        last_debug_->desired_lane_no = desired_lane_no_;
        last_debug_->state = __FUNCTION__;
        last_debug_->debug_message = "Already in best lane to go in";

        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    }
    else {
        cout << "Best is to switch to lane #: " << best_lane_no << " with cost: " << lane_costs[best_lane_no]
             << endl;

        // we don't want 2 lane change, goto intermediate lane and then go to the other one

        int target_lane_no;
        auto diff = abs(best_lane_no - cur_lane_no);
        if (diff == 1) {
            target_lane_no = best_lane_no;
        }
        else {
            // This is only possible in case we are at 0th lane and want to go to 2nd
            // or we are at 2nd lane and want to go to 0th. In both cases the intermediate
            // target lane would be 1
            target_lane_no = 1;
        }

        cout << "Changing state to ChangeLane" << endl;
        state_ = bind(&CarDriver::ChangeLaneState, this, target_lane_no, best_lane_no);
        state_();
    }
}

void CarDriver::ChangeLaneState(int target_lane_no, int best_lane_no) {
    // TODO: keep on checking in case, best_lane_no is no longer the best one
    // we will switch back to the state of driving in lane

    last_debug_->state = __FUNCTION__;

    // 1) Enough distance from the car in front in current lane
    // 2) Enough distance from the closest car in front of us in the target lane
    // 3) closest car behind us in the target lane should not be going so fast that it hits us, when we change

    auto cur_lane_no = (int) (model_.car_d / 4);

    cout << "Change Lane State. Current Lane #: " << cur_lane_no << endl;

    if (cur_lane_no == target_lane_no) {
        // lane change is done
        cout << "Lane changed" << endl;

        last_debug_->debug_message = "Lane change complete, going back to keeplanestate";

        state_ = bind(&CarDriver::KeepLaneState, this);
        state_();
        return;
    }

    SpeedDistanceCostCalculator sd_calc(sensor_fusion_, model_);

    double distance_front;
    const auto req_distance_front = speed_mph_to_mtr_per_sec(model_.ref_speed_mph) * 2;
    auto front_vehicle_in_target = cost_->GetClosestCarInFront(target_lane_no);

    if (front_vehicle_in_target)
        distance_front = sd_calc.GetDistaneToCarMeters(front_vehicle_in_target);
    else
        distance_front = numeric_limits<double>::max();

    if (distance_front < req_distance_front) {

        cout << "TL " << target_lane_no << ", CF is very close. " << distance_front << " required: " << req_distance_front << endl;

        // we can overtake and get in front or
        // we can wait for the car behind us to pass
        // for time being lets just keep going at same speed_mps and decide later
        //auto lane_speed = GetLaneSpeedMph(target_lane_no);
        auto target_lane_speed = sd_calc.GetLaneSpeedMph(target_lane_no) * 0.75;
        auto cur_lane_speed = sd_calc.GetLaneSpeedMph(cur_lane_no);
        double driving_speed_mph = min(target_lane_speed, cur_lane_speed);

        if (target_lane_speed < cur_lane_speed) {
            // in case we are going too slow just to change lanes, and now
            // there is enough distance for us to speed up while keeping car in
            // current lane, then might as well just continue here and abort
            // lane change decision
            auto vehicle_in_front = cost_->GetClosestCarInFront(cur_lane_no);

            if (vehicle_in_front) {
                double last_speed = model_.ref_speed_mph;
                double dead_stop_distance = cost_->GetDistanceToDeadStop(last_speed);
                double front_distance = cost_->GetDistaneToCarMeters(vehicle_in_front);

                if (front_distance > dead_stop_distance * 2) {
                    state_ = bind(&CarDriver::KeepLaneState, this);
                    state_();
                    return;
                }
            }
        }
        else {
            driving_speed_mph = target_lane_speed;
        }

        DriveAtSpeed(driving_speed_mph);

        cout << "TL " << target_lane_no << ", Speed: " << target_lane_speed << " CL Speed:" << cur_lane_speed
             << " driving at: " << driving_speed_mph << endl;

        last_debug_->desired_speed_mph = driving_speed_mph;
        last_debug_->desired_lane_no = desired_lane_no_;

        stringstream ss;
        ss << "Cant change lane, car in front in target lane: " << target_lane_no << ", with id: " << front_vehicle_in_target->car_id;
        last_debug_->debug_message = ss.str();
    }
    else {

        cout << "TL " << target_lane_no << ", CF distance is OK. " << distance_front << " required: " << req_distance_front << endl;

        const auto req_distance_behind = speed_mph_to_mtr_per_sec(50.0) * 2;
        const auto behind_vehicle = cost_->GetClosestCarBehind(target_lane_no);
        double distance_behind;

        if (behind_vehicle)
            distance_behind = sd_calc.GetDistaneToCarMeters(behind_vehicle);
        else
            distance_behind = numeric_limits<double>::max();

        if (distance_behind < req_distance_behind) {

            cout << "TL " << target_lane_no << ", CB is very close. " << distance_behind << " required: " << req_distance_behind << endl;

            // wait for the car to pass us
            const auto speed_for_waiting_mph = max(25.0, behind_vehicle->speed_mps() * 0.5);
            auto cur_lane_speed = sd_calc.GetLaneSpeedMph(cur_lane_no);
            auto driving_speed_mph = min(speed_for_waiting_mph, cur_lane_speed);

            DriveAtSpeed(driving_speed_mph);

            cout << "TL " << target_lane_no << ", Waiting for car behind."
                 << " Behind Speed: " << behind_vehicle->speed_mps()
                 << " Wait Speed: " << speed_for_waiting_mph
                 << " CL Speed:" << cur_lane_speed
                 << " driving at: " << driving_speed_mph << endl;

            stringstream ss;
            ss << "Cant change lane, car at back in target lane: " << target_lane_no << ", with id: " << behind_vehicle->car_id;
            last_debug_->debug_message = ss.str();
            last_debug_->desired_speed_mph = speed_for_waiting_mph;

        } else {

            cout << "TL " << target_lane_no << ": SAFE " << endl
                 << "  Distance Behind: " << distance_behind
                 << "  Required: " << req_distance_behind << endl;
            cout << "  Distance Front: " << distance_front
                 << "  Required : " << req_distance_front << endl;

            // change lanes as it is safe to do so
            desired_lane_no_ = target_lane_no;
            DriveAtSpeed(ideal_speed_mph_, 50, 40);

            last_debug_->debug_message = "Ok to switch over to lane";
            last_debug_->desired_speed_mph = ideal_speed_mph_;
        }
    }
}


std::vector<double> CarDriver::DriveAtSpeed(double speed_mph, int spline_distance_start /*= 30*/, int spline_distance_inc /* = 30 */) {
    auto prev_size = previous_path_x_.size();
    next_x_vals_.clear();
    next_y_vals_.clear();


    tk::spline path_spline = GetPathToFollow(spline_distance_start, spline_distance_inc);


    // add previous points to the next x/y to keep some continuity and minimize
    // jerk

    for (int i = 0; i < prev_size; i++) {
        double prev_x = previous_path_x_[i];
        double prev_y = previous_path_y_[i];

        next_x_vals_.push_back(prev_x);
        next_y_vals_.push_back(prev_y);

        last_debug_->previous_pts.push_back(CartesianPoint(prev_x, prev_y));
    }

    FigureOutSpeedFromPrevious();

    auto speeds = GenerateTrajectory(model_.ref_speed_mph, speed_mph, path_spline);
    return speeds;
//    if (CloseToCar(speed_between_points))
//        state_ = DrivingState::PrepareLaneChangeState;
}

void CarDriver::FigureOutSpeedFromPrevious() {
    auto prev_size = previous_path_x_.size();

    if (prev_size > 1) {
        // translate the last point back into the car origin to figure out
        // the X distance between the last two points. That X distance is basically
        // the speed_mps of the car between those two points. So we start from that
        // speed and then accelerate / decelerate to match the desired speed_mps
        double x_translated = model_.ref_prev_prev.x - model_.ref_prev.x;
        double y_translated = model_.ref_prev_prev.y - model_.ref_prev.y;

        auto x_in_car_frame = x_translated * cos(0 - model_.ref_yaw) - y_translated * sin(0 - model_.ref_yaw);
        model_.ref_speed_mph = speed_mtr_per_sec_to_mph(fabs(x_in_car_frame) * 50);
    } else {
        model_.ref_speed_mph = model_.cur_speed_mph;
    }
}


std::vector<double> CarDriver::GenerateTrajectory(double cur_speed_mph, double required_speed_mph, const tk::spline &path_spline) {
    auto prev_size = previous_path_x_.size();
    auto speeds = GetPerPointSpeed(cur_speed_mph, required_speed_mph, 50 - prev_size);

    auto speed_iterator = speeds.begin();
    double speed_between_points = *speed_iterator++;
    double x_from_origin = 0;

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

        last_debug_->next_pts.push_back(CartesianPoint(x_point, y_point));

        if (speed_iterator != speeds.end())
            speed_between_points = *speed_iterator++;
    }

    return speeds;
}



// Shortcomming: it should have been based on the points that have been generated not
// on the current car_d as previous path might have quite a few points left over

bool CarDriver::CloseToCar(double speed_mph) {
    bool car_close = false;
    double meters_to_stop = GetMetersToStop(speed_mph);

    auto lane_no = (int) (model_.car_d / 4);

    for (auto &other_car: sensor_fusion_) {
        if (other_car.d >= lane_no * 4 && other_car.d <= lane_no * 4 + 4) {
            auto distance = other_car.s - model_.car_s;
            if (distance > 0 && distance < meters_to_stop) {
                car_close = true;
                cout << "Car is close!!! Looks like it is travelling at " << other_car.x_dot << endl;

                break;
            }
        }
    }

    return car_close;
}

double CarDriver::GetMetersToStop(double speed_mph) {
    const auto max_decelerate_mps = 10.0;
    const auto speed_mps = speed_mph_to_mtr_per_sec(speed_mph);
    auto meters_to_stop = speed_mps * speed_mps / max_decelerate_mps + 5.0;
    return meters_to_stop;
}


tk::spline CarDriver::GetPathToFollow(int start_distance, int increment) {
    vector<double> spline_pts_x;
    vector<double> spline_pts_y;

    spline_pts_x.push_back(model_.ref_prev_prev.x);
    spline_pts_x.push_back(model_.ref_prev.x);
    spline_pts_y.push_back(model_.ref_prev_prev.y);
    spline_pts_y.push_back(model_.ref_prev.y);

    auto lane_no_d = GetDesiredFrenetLaneNo(desired_lane_no_);

    FrenetPoint wp[3] = {{model_.car_s + start_distance, lane_no_d},
                         {model_.car_s + start_distance + increment, lane_no_d},
                         {model_.car_s + start_distance + increment * 2, lane_no_d}};

    copy(begin(wp), end(wp), back_inserter(last_debug_->spline_addons));

    for (auto &point : wp) {
        CartesianPoint cp = (CartesianPoint) point;
        spline_pts_x.push_back(cp.x);
        spline_pts_y.push_back(cp.y);
    }

    for (int i = 0; i < spline_pts_x.size(); i++) {
        // copy original spline points for debugging
        last_debug_->spline_pts.push_back(CartesianPoint(spline_pts_x[i], spline_pts_y[i]));

//        const double origin_x = model_.ref_prev.x;
//        const double origin_y = model_.ref_prev.y;
//        // shift back to start of car
//        double x_translated = spline_pts_x[i] - origin_x;
//        double y_translated = spline_pts_y[i] - origin_y;
//
//        auto x_in_car_frame = x_translated * cos(0 - model_.ref_yaw) - y_translated * sin(0-model_.ref_yaw);
//        auto y_in_car_frame = x_translated * sin(0 - model_.ref_yaw) + y_translated * cos(0-model_.ref_yaw);

        auto pt_in_body = model_.TranslateXYToBodyFrame(spline_pts_x[i], spline_pts_y[i]);
        spline_pts_x[i] = pt_in_body.x;
        spline_pts_y[i] = pt_in_body.y;
    }

    tk::spline s;
    s.set_points(spline_pts_x, spline_pts_y);

    // generate debug points for spline curve

    double x = spline_pts_x[0];

    for (int i = 0; i < 50; i++) {
        double y = s(x);
        last_debug_->spline_generated_pts.push_back(CartesianPoint(x, y));

        x++;
    }

    return s;
}

void CarDriver::FigureOutCarOrigin(CartesianPoint *last_pt, CartesianPoint *last_last_pt, double *ref_yaw) {
    auto prev_size = previous_path_x_.size();

    if (prev_size < 2) {
        *ref_yaw = deg2rad(model_.car_yaw);
        *last_pt = CartesianPoint(model_.car_x, model_.car_y);
        *last_last_pt = CartesianPoint(model_.car_x - cos(*ref_yaw), model_.car_y - sin(*ref_yaw));

        last_debug_->ref_yaw.based_on = YawCalculation::CarYaw;
    }
    else {
        last_last_pt->x = previous_path_x_[prev_size - 2];
        last_last_pt->y = previous_path_y_[prev_size - 2];

        last_pt->x = previous_path_x_[prev_size - 1];
        last_pt->y = previous_path_y_[prev_size - 1];

        *ref_yaw = atan2(last_pt->y - last_last_pt->y, last_pt->x - last_last_pt->x);

        last_debug_->ref_yaw.based_on = YawCalculation::PreviousPoints;
    }

    last_debug_->ref_yaw.ref_prev = *last_pt;
    last_debug_->ref_yaw.ref_prev_prev = *last_last_pt;
    last_debug_->ref_yaw.angle_rad = *ref_yaw;
}

std::vector<double> CarDriver::GetPerPointSpeed(double cur_speed_mph, double required_speed_mph, int points_needed) {
    // max_acceleration is 10m/s^2

    const double min_diff = 0.5;    // mph difference

    vector<double> speed;
    const double max_a = 20.0;      // 10m/s * 60 * 60 / 1000 / 1.60934

    for (int i = 0; i < points_needed; i++){
        double diff = required_speed_mph - cur_speed_mph;

        // v = 1/2 * a * t^2
        // we know v, have to find a needed to acheive this

        // how much do we need to accelerate to get to this speed_mps
        auto a = min(max_a, diff) * 0.02;
        cur_speed_mph += a;

        speed.push_back(cur_speed_mph);
    }

    // add one speed_mps in case there was no need to accelerate / decelerate
    if (0 == speed.size())
        speed.push_back(cur_speed_mph);

    return speed;
}

//double CarDriver::GetLaneCosts(int current_lane, int intended_lane, int final_lane) {
//    // TODO: for the time being stick to current -> intended lane only
//
//    // lane that has the farthest car (or no car) is the best lane to be in
//    // but consider the other car's speed_mps as well. A car that is far away
//    // but is dead stop might be worse than a car that is some what closer but going
//    // faster than us.
//    // formula to calculate the distance_meters to reach car:
//    // time = distance_meters / difference in speed_mps (in meters)
//
//    auto closest_car = GetClosestCarInFront(intended_lane);
//    auto distance_meters = GetDistaneToCarMeters(closest_car);
//    auto our_speed_mps = speed_mph_to_mtr_per_sec(ideal_speed_mph_);
//    auto other_car_speed_mps = closest_car->x_dot;
//    auto delta_speed = our_speed_mps - other_car_speed_mps;
//
//    auto catchup_distance = distance_meters / delta_speed;
//    auto cost = 1 - exp(-abs(catchup_distance));
//    return cost;
//}



//double CarDriver::CostSpeed(double intended_speed, double target_speed) {
//    double cost = (target_speed - intended_speed) / target_speed;
//    if (cost < 0)
//        cost = 0;
//    return cost;
//}

//double CarDriver::CostDrivingInCurrentLane(double *lane_speed_ptr) {
//    auto lane_no = (int) (model_.car_d / 4);
//    auto lane_speed = GetLaneSpeedMph(lane_no);
//    auto cost = CostSpeed(lane_speed, ideal_speed_mph_);
//
//    if (lane_speed_ptr)
//        *lane_speed_ptr = lane_speed;
//
//    return cost;
//}

//double CarDriver::GetLaneSpeedMph(int lane_no) {
//    auto closest_car = cost_->GetClosestCarInFront(lane_no);
//    if (!closest_car)
//        return ideal_speed_mph_;
//
//    const auto traffic_speed_mph_ = speed_mtr_per_sec_to_mph(closest_car->speed_mps());
//    return traffic_speed_mph_ ;
//}


void CarDriver::KeepLaneState() {
    bool car_close = DriveWhileKeepingDistance();
    if (car_close)
        state_ = bind(&CarDriver::PrepareLaneChangeState, this);

    last_debug_->desired_speed_mph = ideal_speed_mph_;
    last_debug_->desired_lane_no = desired_lane_no_;
    last_debug_->state = __FUNCTION__;
}


bool CarDriver::DriveWhileKeepingDistance() {
    bool car_close = false;
    auto speed_to_maintain_mph = ideal_speed_mph_;
    auto vehicle_in_front = cost_->GetClosestCarInFront(desired_lane_no_);

    if (vehicle_in_front) {
        double last_speed = model_.ref_speed_mph;
        double dead_stop_distance = cost_->GetDistanceToDeadStop(last_speed);
        double front_distance = cost_->GetDistaneToCarMeters(vehicle_in_front);

        if (front_distance < dead_stop_distance) {

            auto front_car_speed = vehicle_in_front->speed_mps();

            double speed_mps;
            if (front_distance < dead_stop_distance * 0.5) {
                cout << "Half speed" << endl;
                speed_mps = front_car_speed * 0.5;
            } else
                speed_mps = front_car_speed;

            speed_to_maintain_mph = speed_mtr_per_sec_to_mph(speed_mps);

            stringstream ss;
            ss << "Car is close to car in front, with id: " << vehicle_in_front->car_id
               << " DS: " << dead_stop_distance
               << " FC->car_s " << vehicle_in_front->s
               << " FC->distance " << cost_->GetDistaneToCarMeters(vehicle_in_front)
               << " FC->speed " << speed_mtr_per_sec_to_mph(front_car_speed)
               << endl;
            cout << ss.str();
            last_debug_->debug_message = ss.str();

            car_close = true;
        }
    }

    DriveAtSpeed(speed_to_maintain_mph);

    return car_close;
}

void CarDriver::InitState() {
//    int closest_lane = model_.car_d / 4;
    int closest_lane = -1;
    int closest_distance = 999999;

    for (int i = 0; i < 3; i++) {
        auto car = cost_->GetClosestCarInFront(i);
        if (car) {
            double front_distance = cost_->GetDistaneToCarMeters(car);

            if (closest_distance > front_distance) {
                cout << "Found a car closer in lane: " << i << " at distance: " << front_distance << endl;

                closest_distance = front_distance;
                closest_lane = i;
            }
        }
    }

    if (closest_lane >= 0) {
        desired_lane_no_ = closest_lane;
        cout << "Starting off with lane: " << desired_lane_no_ << endl;
        state_ = bind(&CarDriver::KeepLaneState, this);
    }
}
