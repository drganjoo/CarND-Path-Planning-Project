//
// Created by fahad on 11/29/17.
//

#include "CarDriver.h"
#include <memory>

using namespace std;

static double deg2rad(double x) { return x * M_PI / 180.0; }
static double rad2deg(double x) { return x * 180 / M_PI; }

vector<double> getFrenet(double x, double y, double theta);

std::array<std::vector<double>, 2> CarDriver::GetPath() {
    return {next_x_vals_, next_y_vals_};
}

//double CarDriver::HowManyMetersTravelledIn20ms(double speed_mph) {
//    auto distance_in_20ms = speed_mph_to_mtr_per_sec(speed_mph) * 0.02;
//    return distance_in_20ms;
//}


void CarDriver::UpdateModel(json &j) {
    model_ = unique_ptr<CarModel>(new CarModel(j));
    last_debug_ = unique_ptr<DebugValues>(new DebugValues(*model_));

//    model_.car_x = j["x"];
//    model_.car_y = j["y"];
//    model_.car_s = j["s"];
//    model_.car_d = j["d"];
//    model_.car_yaw = j["yaw"];
//    model_.speed_mph = j["speed"];

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

    Process();

    last_debug_->sensor_fusion = sensor_fusion_;
    last_debug_->desired_speed_mph = desired_speed_mph;
    last_debug_->desired_lane_no = desired_lane_no;

//    debug_packets_.push_back(last_debug_);
}

void CarDriver::Process() {
    auto prev_size = previous_path_x_.size();
    next_x_vals_.clear();
    next_y_vals_.clear();

    cout << "-------------------- Process " << prev_size << ", " << speed_mph_to_mtr_per_sec(model_->speed_mph) * 0.02
         << " mp20 --------------------" << endl;

//    model_.desired_speed_mph = get_ideal_speed();

//    for (auto &constraint : constraints_) {
//        constraint.Apply(&model_);
//    }

    double ref_yaw;
    CartesianPoint ref_prev_prev, ref_prev;
    FigureOutCarOrigin(&ref_prev, &ref_prev_prev, &ref_yaw);

    auto s = GetSpline(ref_prev, ref_prev_prev, ref_yaw);

    // add previous points to the next x/y to keep some continuity and minimize
    // jerk
    double speed_cur_mp20;

    for (int i = 0; i < prev_size; i++) {
        double prev_x = previous_path_x_[i];
        double prev_y = previous_path_y_[i];

        next_x_vals_.push_back(prev_x);
        next_y_vals_.push_back(prev_y);

        last_debug_->previous_pts.push_back(CartesianPoint(prev_x, prev_y));
    }

    if (prev_size > 1) {
        double prev_x_1 = previous_path_x_[prev_size - 1];
        double prev_x = previous_path_x_[prev_size - 2];
        speed_cur_mp20 = prev_x_1 - prev_x;
    } else {
        speed_cur_mp20 = speed_mph_to_mtr_per_sec(model_->speed_mph) * 0.02;
    }

    cout << "Speed at end of prev: " << speed_cur_mp20 << " mp20. Ideally should be: " <<
         speed_mph_to_mtr_per_sec(get_ideal_speed()) * 0.02 << " mp20" << endl;

    const auto target_x = 30.0;           // maximum m/s that the car can travel
    const auto target_y = s(target_x);
//    const auto distance = sqrt(target_x * target_x + target_y * target_y);
//    const auto distance_in_20ms = speed_mph_to_mtr_per_sec(model_.speed_mph) * 0.02;
//    const double N = distance / distance_in_20ms;
//    const double part_x = target_x / N;
//    const auto part_x = distance_in_20ms;

    auto speeds = GetDesiredSpeedPerSecond(speed_cur_mp20, 50 - prev_size + 1);
    auto speed_iterator = speeds.begin();
    double desired_speed_mp20ms = *speed_iterator++;

    for (int i = 1; i < 50 - prev_size; i++) {

//    const auto distance_in_20ms = speed_mph_to_mtr_per_sec(model_.speed_mph) * 0.02;
        const auto part_x = desired_speed_mp20ms;

        double x_point = i * part_x;
        double y_point = s(x_point);

        x_point = x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
        y_point = x_point * sin(ref_yaw) + y_point * cos(ref_yaw);

        x_point += ref_prev.x;
        y_point += ref_prev.y;

        next_x_vals_.push_back(x_point);
        next_y_vals_.push_back(y_point);

        last_debug_->next_pts.push_back(CartesianPoint(x_point, y_point));

        if (speed_iterator != speeds.end())
            desired_speed_mp20ms = *speed_iterator++;

        cout << "Speed set to: " << desired_speed_mp20ms << " mp20 == " <<
             speed_mtr_per_sec_to_mph(desired_speed_mp20ms / 0.02) << " mph" << endl;
    }
}

tk::spline CarDriver::GetSpline(const CartesianPoint &ref_prev, CartesianPoint &ref_prev_prev, double ref_yaw) {
    vector<double> spline_pts_x;
    vector<double> spline_pts_y;

    spline_pts_x.push_back(ref_prev_prev.x);
    spline_pts_x.push_back(ref_prev.x);
    spline_pts_y.push_back(ref_prev_prev.y);
    spline_pts_y.push_back(ref_prev.y);

    double origin_x = ref_prev.x;
    double origin_y = ref_prev.y;

    auto lane_no_d = GetDesiredFrenetLaneNo();

//    auto prev_frenet = getFrenet(ref_prev.x, ref_prev.y, ref_yaw);
//    auto prev_prev_frenet = getFrenet(ref_prev_prev.x, ref_prev_prev.y, ref_yaw);

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

        auto x_in_car_frame = x_translated * cos(0 - ref_yaw) - y_translated * sin(0-ref_yaw);
        auto y_in_car_frame = x_translated * sin(0 - ref_yaw) + y_translated * cos(0-ref_yaw);

//        cout << x_translated << ",\t" << y_translated << ",\t" << x_in_car_frame << ",\t" << y_in_car_frame << endl;

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
    return desired_speed_mph;
}

std::vector<double> CarDriver::GetDesiredSpeedPerSecond(double speed_cur_mp20, int points_needed) {
    // max_acceleration is 10m/s^2

    const double min_diff_20ms = speed_mph_to_mtr_per_sec(2.0) * 0.02;
//    const double desired_speed_mp20 = speed_mph_to_mtr_per_sec(get_ideal_speed()) * 0.02;
    const double desired_speed_mp20 = speed_mph_to_mtr_per_sec(50) * 0.02;

    vector<double> speed;
//    speed.push_back(desired_speed_mp20);
//    return speed;

    int i = 0;
    while (i++ < points_needed && fabs(speed_cur_mp20 - desired_speed_mp20) > min_diff_20ms) {
        double diff = desired_speed_mp20 - speed_cur_mp20;

        // v = 1/2 * a * t^2
        // we know v, have to find a needed to acheive this

        auto a = (diff * 2) / (0.02 * 0.02);
        a = min(a, 10.0);        // 10 m/s^2 is the maximum acceleration

        auto old = speed_cur_mp20;
        speed_cur_mp20 += 0.5 * a * 0.02 * 0.02;

        if (old > speed_cur_mp20)
            cout << "speed decreasing" << endl;

        speed.push_back(speed_cur_mp20);
    }

    // add one speed in case there was no need to accelerate / decelerate
    if (0 == speed.size())
        speed.push_back(speed_cur_mp20);

    return speed;
}

