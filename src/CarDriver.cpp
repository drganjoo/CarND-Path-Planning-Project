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
#ifdef _DEBUG_DATA
    state_ = bind(&CarDriver::InitState, this);
#else
    state_ = bind(&CarDriver::KeepLaneState, this);
#endif
}

std::array<std::vector<double>, 2> CarDriver::GetPath() {
    return {next_x_vals_, next_y_vals_};
}


void CarDriver::UpdateModel(json &j) {
    model_ = CarModel(j);
#ifdef _DEBUG_DATA
    last_debug_ = unique_ptr<DebugValues>(new DebugValues(model_));
#endif

    previous_path_x_ = j["previous_path_x"];
    previous_path_y_ = j["previous_path_y"];

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

        sensor_fusion_.push_back(v);
    }

    FigureOutCarOrigin(&model_.ref_prev, &model_.ref_prev_prev, &model_.ref_yaw);
    FigureOutSpeedFromPrevious();

#ifdef _DEBUG_DATA
    last_debug_->sensor_fusion = sensor_fusion_;
    last_debug_->debug_message.clear();
#endif

    cost_ = unique_ptr<CostCalculator>(new CostCalculator(sensor_fusion_, model_));
    state_();
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

    if (desired_lane_no_ == best_lane_no) {
        bool car_close = DriveWhileKeepingDistance();
        if (!car_close)
            state_ = bind(&CarDriver::KeepLaneState, this);

#ifdef _DEBUG_DATA
        last_debug_->desired_speed_mph = lane_speeds[best_lane_no];
        last_debug_->desired_lane_no = desired_lane_no_;
        last_debug_->state = __FUNCTION__;
        last_debug_->debug_message = "Already in best lane to go in";
#endif
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
#ifdef _DEBUG_DATA
    last_debug_->state = __FUNCTION__;
#endif

    // 1) Enough distance from the car in front in current lane
    // 2) Enough distance from the closest car in front of us in the target lane
    // 3) closest car behind us in the target lane should not be going so fast that it hits us, when we change

    auto cur_lane_no = (int) (model_.car_d / 4);

    cout << "CHANGE_LANE: cur_lane_no = " << cur_lane_no << " target_lane_no = " << target_lane_no;

    if (cur_lane_no == target_lane_no) {
        cout << "Lane changed" << endl;

        state_ = bind(&CarDriver::KeepLaneState, this);
        state_();

#ifdef _DEBUG_DATA
        last_debug_->debug_message = "Lane change complete, going back to keeplanestate";
#endif

        return;
    }


    bool front_ok = TargetLaneFrontOk(target_lane_no);
    bool back_ok = false;

    if (front_ok) {

        back_ok = TargetLaneBackOk(target_lane_no);

        if (back_ok) {

            desired_lane_no_ = target_lane_no;

            const auto lane_change_speed_mph = min(ideal_speed_mph_, MAX_LANE_CHANGE_SPEED_MPH);
            DriveAtSpeed(lane_change_speed_mph, 50, 40);

            cout << ", SAFE!!!, switching to lane: " << target_lane_no << endl;

#ifdef _DEBUG_DATA
            last_debug_->debug_message = "Ok to switch over to lane";
            last_debug_->desired_speed_mph = ideal_speed_mph_;
#endif

            return;
        }
    }


    // either front or back is not ok
    if (EnoughDistanceInFront(cur_lane_no)) {
        cout << ", big distance in front, aborting lane change";

        state_ = bind(&CarDriver::KeepLaneState, this);
        state_();

        return;
    }


    if (!front_ok) {
        // there is a car in the target lane that is too close for us to change lanes
        // slow down and create some distance (BUT if we are going so slow that now
        // car in front of current lane is quite far then speed up, stay here and abort lane change)

        auto target_lane_speed = cost_->GetLaneSpeedMph(target_lane_no);
        auto cur_lane_speed = cost_->GetLaneSpeedMph(cur_lane_no);

        double driving_speed_mph = min(target_lane_speed * CL_SLOW_DOWN_FACTOR, cur_lane_speed);

        DriveAtSpeed(driving_speed_mph);

        cout << ", car in front in target, slowing down to = " << driving_speed_mph << " mph " << endl;

#ifdef _DEBUG_DATA
        last_debug_->desired_speed_mph = driving_speed_mph;
        last_debug_->desired_lane_no = desired_lane_no_;

        stringstream ss;
        ss << "Can't change lane, car in front in target lane: " << target_lane_no;
        last_debug_->debug_message = ss.str();
#endif
    }
    else {
        // behind is not ok otherwise we would have never landed here
        assert(!back_ok);

        // lets see if we can speed up and then change lanes
        bool speed_up = true;

        auto vehicle_in_front = cost_->GetClosestCarInFront(cur_lane_no);
        if (vehicle_in_front) {
            double last_speed = model_.ref_speed_mph;
            double dead_stop_distance = cost_->GetDistanceToDeadStop(last_speed);
            double front_distance = cost_->GetDistaneToCarMeters(vehicle_in_front);

            speed_up = front_distance > dead_stop_distance;
        }

        if (speed_up) {
            cout << ", behind car too close, speeding up as we have space up front" << endl;

            DriveAtSpeed(ideal_speed_mph_);
        }
        else {
            auto vehicle_behind_target = cost_->GetClosestCarBehind(target_lane_no);

            // very rare case that there is no vehicle behind us and still we end up here
            if (!vehicle_behind_target) {

                cout << ", behind car too close, BUT NO vehicle there so just driving here" << endl;

                DriveAtSpeed(ideal_speed_mph_);
                return;
            }

            // wait for the car to pass us
            const auto cur_lane_speed = cost_->GetLaneSpeedMph(cur_lane_no);
            const auto bv_speed_mph = speed_mtr_per_sec_to_mph(vehicle_behind_target->speed_mps());

            // half of behind vehicle or 25mph, whichever is higher
            const auto speed_for_waiting_mph = max(25.0, bv_speed_mph * 0.5);
            const auto driving_speed_mph = min(speed_for_waiting_mph, cur_lane_speed);

            DriveAtSpeed(driving_speed_mph);

            cout << ", Waiting for car behind, coming at speed = " << bv_speed_mph
                 << " driving at = " << driving_speed_mph << endl;

#ifdef _DEBUG_DATA
            stringstream ss;
            ss << "Cant change lane, car at back in target lane: " << target_lane_no << ", with id: "
               << vehicle_behind_target->car_id;
            last_debug_->debug_message = ss.str();
            last_debug_->desired_speed_mph = speed_for_waiting_mph;
#endif
        }
   }
}


std::vector<double> CarDriver::DriveAtSpeed(double speed_mph, int spline_distance_start /*= 30*/, int spline_distance_inc /* = 30 */) {
    if (speed_mph > MAX_ALLOWED_SPEED_MPH_)
        speed_mph = MAX_ALLOWED_SPEED_MPH_;

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

#ifdef _DEBUG_DATA
        last_debug_->previous_pts.push_back(CartesianPoint(prev_x, prev_y));
#endif
    }

    FigureOutSpeedFromPrevious();

    auto speeds = GenerateTrajectory(model_.ref_speed_mph, speed_mph, path_spline);
    return speeds;
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

#ifdef _DEBUG_DATA
        last_debug_->next_pts.push_back(CartesianPoint(x_point, y_point));
#endif
        if (speed_iterator != speeds.end())
            speed_between_points = *speed_iterator++;
    }

    return speeds;
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

    for (auto &point : wp) {
        CartesianPoint cp = (CartesianPoint) point;
        spline_pts_x.push_back(cp.x);
        spline_pts_y.push_back(cp.y);
    }


    for (int i = 0; i < spline_pts_x.size(); i++) {
        auto pt_in_body = model_.TranslateXYToBodyFrame(spline_pts_x[i], spline_pts_y[i]);
        spline_pts_x[i] = pt_in_body.x;
        spline_pts_y[i] = pt_in_body.y;
    }

    tk::spline s;
    s.set_points(spline_pts_x, spline_pts_y);

#ifdef _DEBUG_DATA
    // copy original spline generating points for debugging

    copy(begin(wp), end(wp), back_inserter(last_debug_->spline_addons));
    for (int i = 0; i < spline_pts_x.size(); i++) {
        last_debug_->spline_pts.push_back(CartesianPoint(spline_pts_x[i], spline_pts_y[i]));
    }

    double x = spline_pts_x[0];
    for (int i = 0; i < 50; i++) {
        double y = s(x);
        last_debug_->spline_generated_pts.push_back(CartesianPoint(x, y));
        x++;
    }
#endif

    return s;
}

void CarDriver::FigureOutCarOrigin(CartesianPoint *last_pt, CartesianPoint *last_last_pt, double *ref_yaw) {
    auto prev_size = previous_path_x_.size();

    if (prev_size < 2) {
        *ref_yaw = deg2rad(model_.car_yaw);
        *last_pt = CartesianPoint(model_.car_x, model_.car_y);
        *last_last_pt = CartesianPoint(model_.car_x - cos(*ref_yaw), model_.car_y - sin(*ref_yaw));

#ifdef _DEBUG_DATA
        last_debug_->ref_yaw.based_on = YawCalculation::CarYaw;
#endif
    }
    else {
        last_last_pt->x = previous_path_x_[prev_size - 2];
        last_last_pt->y = previous_path_y_[prev_size - 2];

        last_pt->x = previous_path_x_[prev_size - 1];
        last_pt->y = previous_path_y_[prev_size - 1];

        *ref_yaw = atan2(last_pt->y - last_last_pt->y, last_pt->x - last_last_pt->x);

#ifdef _DEBUG_DATA
        last_debug_->ref_yaw.based_on = YawCalculation::PreviousPoints;
#endif
    }

#ifdef _DEBUG_DATA
    last_debug_->ref_yaw.ref_prev = *last_pt;
    last_debug_->ref_yaw.ref_prev_prev = *last_last_pt;
    last_debug_->ref_yaw.angle_rad = *ref_yaw;
#endif
}

std::vector<double> CarDriver::GetPerPointSpeed(double cur_speed_mph, double required_speed_mph, int points_needed) {

    vector<double> speed;

    for (int i = 0; i < points_needed; i++){
        double diff = required_speed_mph - cur_speed_mph;

        // v = 1/2 * a * t^2
        // we know v, have to find a needed to acheive this

        // how much do we need to accelerate to get to this speed_mps
        auto a = min(MAX_ACCELERATION, diff) * 0.02;
        cur_speed_mph += a;

        speed.push_back(cur_speed_mph);
    }

    // add one speed_mps in case there was no need to accelerate / decelerate
    if (0 == speed.size())
        speed.push_back(cur_speed_mph);

    return speed;
}


void CarDriver::KeepLaneState() {
    bool car_close = DriveWhileKeepingDistance();
    if (car_close)
        state_ = bind(&CarDriver::PrepareLaneChangeState, this);

#ifdef _DEBUG_DATA
    last_debug_->desired_speed_mph = ideal_speed_mph_;
    last_debug_->desired_lane_no = desired_lane_no_;
    last_debug_->state = __FUNCTION__;
#endif
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

#ifdef _DEBUG_DATA
            stringstream ss;
            ss << "Car is close to car in front, with id: " << vehicle_in_front->car_id
               << " DS: " << dead_stop_distance
               << " FC->car_s " << vehicle_in_front->s
               << " FC->distance " << cost_->GetDistaneToCarMeters(vehicle_in_front)
               << " FC->speed " << speed_mtr_per_sec_to_mph(front_car_speed)
               << endl;
            cout << ss.str();
            last_debug_->debug_message = ss.str();
#endif
            car_close = true;
        }
    }

    DriveAtSpeed(speed_to_maintain_mph);

    return car_close;
}

#ifdef _DEBUG_DATA

void CarDriver::InitState() {

    // start of from a lane that has a car really close by as need to test lane change

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

#endif

bool CarDriver::TargetLaneFrontOk(int target_lane_no) {
    SpeedDistanceCostCalculator sd_calc(sensor_fusion_, model_);

    double distance_front_target;
    const auto req_distance_front = speed_mph_to_mtr_per_sec(model_.ref_speed_mph) * 2;
    auto front_vehicle_in_target = cost_->GetClosestCarInFront(target_lane_no);

    if (front_vehicle_in_target)
        distance_front_target = sd_calc.GetDistaneToCarMeters(front_vehicle_in_target);
    else
        distance_front_target = numeric_limits<double>::max();

    return distance_front_target > req_distance_front;
}


bool CarDriver::TargetLaneBackOk(int target_lane_no) {
    bool behind_ok = true;
    const auto behind_vehicle = cost_->GetClosestCarBehind(target_lane_no);

    double bv_s_after = 0;
    double our_s_after = 0;
    double bv_speed_mps = 0;

    if (behind_vehicle) {
        bv_speed_mps = behind_vehicle->speed_mps();
        bv_s_after = behind_vehicle->s + bv_speed_mps * CL_BEHIND_DISTANCE_SECS;
        bv_s_after += 2; // length of the car is added

        // since we won't be going straight, we would be a little behind had we gone
        // directly
        const auto our_speed_mps = speed_mph_to_mtr_per_sec(model_.ref_speed_mph) * 0.5;
        our_s_after = model_.car_s + our_speed_mps;

        behind_ok = bv_s_after < our_s_after;
    }

    return behind_ok;
}


bool CarDriver::EnoughDistanceInFront(int lane_no) {

    bool abort = true;
    auto vehicle_in_front = cost_->GetClosestCarInFront(lane_no);

    if (vehicle_in_front) {
        double last_speed = model_.ref_speed_mph;
        double dead_stop_distance = cost_->GetDistanceToDeadStop(last_speed);
        double front_distance = cost_->GetDistaneToCarMeters(vehicle_in_front);

        abort = front_distance > dead_stop_distance * CL_FRONT_ABORT_SECS;
    }

    return abort;
}