//
// Created by fahad on 11/29/17.
//

#include "CarDriver.h"
#include <memory>

using namespace std;

static double deg2rad(double x) { return x * M_PI / 180.0; }
static double rad2deg(double x) { return x * 180 / M_PI; }

vector<double> getFrenet(double x, double y, double theta);

CarDriver::CarDriver() {
    end_path_s_ = 0.0;
    end_path_d_ = 0.0;
    state_ = DrivingState::FollowSpeedLimit;
}


std::array<std::vector<double>, 2> CarDriver::GetPath() {
    return {next_x_vals_, next_y_vals_};
}


void CarDriver::UpdateModel(json &j) {
    model_ = unique_ptr<CarModel>(new CarModel(j));
    last_debug_ = unique_ptr<DebugValues>(new DebugValues(*model_));

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

        sensor_fusion_.push_back(v);
    }

    //DriveAtSpeed();
    DoState();

    last_debug_->sensor_fusion = sensor_fusion_;
    last_debug_->desired_speed_mph = desired_speed_mph_;
    last_debug_->desired_lane_no = desired_lane_no_;

//    debug_packets_.push_back(last_debug_);
}

void CarDriver::DoState() {
    switch (state_){
        case DrivingState::FollowSpeedLimit:
            DriveAtSpeed(desired_speed_mph_);
            break;

        case DrivingState ::MatchCarSpeed:
            MatchCarSpeed();
            break;

        default:
            break;
    }
}

void CarDriver::MatchCarSpeed() {
    VehicleSensed *closest_car = nullptr;

    cout << "---------------------------- MatchCarSpeed ------------------" << endl;

    // find the closest car to us

    auto sensor_size = sensor_fusion_.size();
    if (sensor_size !=  0) {

        auto max_decelerate_mps = 10.0;
        auto speed_mps = speed_mph_to_mtr_per_sec(model_->cur_speed_mph);
        auto meters_to_stop = speed_mps * speed_mps / max_decelerate_mps + 5.0;

        // find the car that is closest to us
        auto lane_no = (int) (model_->car_d / 4);

        for (auto &other_car : sensor_fusion_) {
            // only look at cars that are in our lane
            if (other_car.d >= lane_no * 4 && other_car.d <= lane_no * 4 + 4) {

                // the car should be close to us
                auto distance = other_car.s - model_->car_s;
                if (distance > 0 && distance < meters_to_stop) {

                    // mark this car as the closest in case other cars in our lane
                    // are farther than this

                    if (!closest_car)
                        closest_car = &other_car;
                    else if (other_car.s < closest_car->s)
                        closest_car = &other_car;
                }
            }
        }
    }

    // switch states in case there is no closest car
    if (!closest_car) {
        cout << "Couldn't find a closest car. Going for FollowSpeedLimit" << endl;

        state_ = DrivingState::FollowSpeedLimit;
        DriveAtSpeed(desired_speed_mph_);
        return;
    }

    auto other_car_speed = speed_mtr_per_sec_to_mph(closest_car->x_dot);
    cout << "Closed car found: " << closest_car->car_id << " travelling at: " << other_car_speed << " mph" << endl;

    desired_speed_mph_ = speed_mtr_per_sec_to_mph(closest_car->x_dot);

    if (fabs(desired_speed_mph_ - model_->cur_speed_mph) < 2) {
        //ChangeLanes();
        cout << "Speed matched, we can now go for changing lanes now " << endl;
    }

    DriveAtSpeed(desired_speed_mph_);
}

void CarDriver::DriveAtSpeed(double speed_mph) {
    auto prev_size = previous_path_x_.size();
    next_x_vals_.clear();
    next_y_vals_.clear();

    cout << "-------------------- DriveAtSpeed " << prev_size << ", " << model_->cur_speed_mph
         << " mph --------------------" << endl;

    tk::spline path_spline = GetPathToFollow();

    // add previous points to the next x/y to keep some continuity and minimize
    // jerk

    for (int i = 0; i < prev_size; i++) {
        double prev_x = previous_path_x_[i];
        double prev_y = previous_path_y_[i];

        next_x_vals_.push_back(prev_x);
        next_y_vals_.push_back(prev_y);

        last_debug_->previous_pts.push_back(CartesianPoint(prev_x, prev_y));
    }

    if (prev_size > 1) {
        // translate the last point back into the car origin to figure out
        // the X distance between the last two points. That X distance is basically
        // the speed of the car between those two points. So we start from that
        // speed and then accelerate / decelerate to match the desired speed
        double x_translated = model_->ref_prev_prev.x - model_->ref_prev.x;
        double y_translated = model_->ref_prev_prev.y - model_->ref_prev.y;

        auto x_in_car_frame = x_translated * cos(0 - model_->ref_yaw) - y_translated * sin(0-model_->ref_yaw);
        model_->ref_speed_mph = speed_mtr_per_sec_to_mph(fabs(x_in_car_frame) * 50);
    } else {
        model_->ref_speed_mph = model_->cur_speed_mph;
    }

    cout << "Speed at end of prev: " << model_->ref_speed_mph << " mph. Ideally should be: " <<
         get_ideal_speed() << " mph" << endl;

    auto speed_between_points = GenerateNextXYForSpeed(model_->ref_speed_mph, speed_mph, path_spline);

    if (CloseToCar(speed_between_points))
        state_ = DrivingState::MatchCarSpeed;
}


double CarDriver::GenerateNextXYForSpeed(double cur_speed_mph, double required_speed_mph, const tk::spline &path_spline) {
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
        auto x_point = x_from_origin * cos(model_->ref_yaw) - y_from_origin * sin(model_->ref_yaw);
        auto y_point = x_from_origin * sin(model_->ref_yaw) + y_from_origin * cos(model_->ref_yaw);
        x_point += model_->ref_prev.x;
        y_point += model_->ref_prev.y;

        next_x_vals_.push_back(x_point);
        next_y_vals_.push_back(y_point);

        last_debug_->next_pts.push_back(CartesianPoint(x_point, y_point));

        if (speed_iterator != speeds.end())
            speed_between_points = *speed_iterator++;
    }

    return speed_between_points;
}



// Shortcomming: it should have been based on the points that have been generated not
// on the current car_d as previous path might have quite a few points left over

bool CarDriver::CloseToCar(double speed_mph) {
    bool car_close = false;

    auto max_decelerate_mps = 10.0;
    auto speed_mps = speed_mph_to_mtr_per_sec(speed_mph);
    auto meters_to_stop = speed_mps * speed_mps / max_decelerate_mps + 5.0;

    auto lane_no = (int) (model_->car_d / 4);

    for (auto &other_car: sensor_fusion_) {
        if (other_car.d >= lane_no * 4 && other_car.d <= lane_no * 4 + 4) {
            cout << "car: " << other_car.car_id << " with d: " << other_car.d
                 << " is in our lane, and it will take us  "
                 << meters_to_stop << " meters to stop" << endl;

            auto distance = other_car.s - model_->car_s;
            if (distance > 0 && distance < meters_to_stop) {
                car_close = true;
                cout << "Car is close!!! Looks like it is travelling at " << other_car.x_dot << endl;

                break;
            }
        }
    }

    return car_close;
}

tk::spline CarDriver::GetPathToFollow() {
    vector<double> spline_pts_x;
    vector<double> spline_pts_y;

    FigureOutCarOrigin(&model_->ref_prev, &model_->ref_prev_prev, &model_->ref_yaw);

    spline_pts_x.push_back(model_->ref_prev_prev.x);
    spline_pts_x.push_back(model_->ref_prev.x);
    spline_pts_y.push_back(model_->ref_prev_prev.y);
    spline_pts_y.push_back(model_->ref_prev.y);

    double origin_x = model_->ref_prev.x;
    double origin_y = model_->ref_prev.y;

    auto lane_no_d = GetDesiredFrenetLaneNo();
    FrenetPoint wp[3] = {{model_->car_s + 30, lane_no_d}, {model_->car_s + 60, lane_no_d}, {model_->car_s + 90, lane_no_d}};

    copy(begin(wp), end(wp), back_inserter(last_debug_->spline_addons));

    for (auto &point : wp) {
        CartesianPoint cp = (CartesianPoint) point;
        spline_pts_x.push_back(cp.x);
        spline_pts_y.push_back(cp.y);
    }

    for (int i = 0; i < spline_pts_x.size(); i++) {
        // copy original spline points for debugging
        last_debug_->spline_pts.push_back(CartesianPoint(spline_pts_x[i], spline_pts_y[i]));

        // shift back to start of car
        double x_translated = spline_pts_x[i] - origin_x;
        double y_translated = spline_pts_y[i] - origin_y;

        auto x_in_car_frame = x_translated * cos(0 - model_->ref_yaw) - y_translated * sin(0-model_->ref_yaw);
        auto y_in_car_frame = x_translated * sin(0 - model_->ref_yaw) + y_translated * cos(0-model_->ref_yaw);

        spline_pts_x[i] = x_in_car_frame;
        spline_pts_y[i] = y_in_car_frame;
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
        *ref_yaw = deg2rad(model_->car_yaw);
        *last_pt = CartesianPoint(model_->car_x, model_->car_y);
        *last_last_pt = CartesianPoint(model_->car_x - cos(*ref_yaw), model_->car_y - sin(*ref_yaw));

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

double CarDriver::get_ideal_speed() {
//    // ensure we are not going to crash
//    for (auto &other_car: sensor_fusion_) {
//        auto distance = other_car.s - model_.car_s;
//        if (distance > 0 && other_car.s - model_.car_s < 20) {
//            cout << "Car is close!!!" << endl;
//        }
//    }

    // return
    return desired_speed_mph_;
}

std::vector<double> CarDriver::GetPerPointSpeed(double cur_speed_mph, double required_speed_mph, int points_needed) {
    // max_acceleration is 10m/s^2

    const double min_diff = 0.5;    // mph difference

    vector<double> speed;
//    speed.push_back(desired_speed_mph_);
//    return speed;

    // && fabs(cur_speed_mph - desired_speed_mph_) > min_diff

    const double max_a = 20.0;      // 10m/s * 60 * 60 / 1000 / 1.60934

    for (int i = 0; i < points_needed; i++){
        double diff = required_speed_mph - cur_speed_mph;

        // v = 1/2 * a * t^2
        // we know v, have to find a needed to acheive this

        // how much do we need to accelerate to get to this speed
        auto a = min(max_a, diff) * 0.02;
        cur_speed_mph += a;

        speed.push_back(cur_speed_mph);
    }

    // add one speed in case there was no need to accelerate / decelerate
    if (0 == speed.size())
        speed.push_back(cur_speed_mph);

    return speed;
}
