//
// Created by fahad on 11/29/17.
//

#include "CarDriver.h"

using namespace std;

std::array<std::vector<double>, 2> CarDriver::GetPath() {
    return {next_x_vals_, next_y_vals_};
}

double CarDriver::GetSpeedInMetersPerSec(double speed_mph) {
    auto speed_per_sec = (speed_mph * 1.60934 * 1000.0) / (60.0 * 60.0) * 0.02;
    return speed_per_sec;
}

void CarDriver::UpdateModel(json &j) {
    model_.car_x = j["x"];
    model_.car_y = j["y"];
    model_.car_s = j["s"];
    model_.car_d = j["d"];
    model_.car_yaw = j["yaw"];
    model_.speed_mph = j["speed"];

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
}

void CarDriver::Process() {
    for (int i = 0; i < previous_path_x_.size(); i++) {
        // copy over to next_x
    }

    auto lane_no_d = model_.GetDesiredFrenetLaneNo();
    model_.speed_mph = get_ideal_speed();

    FrenetPoint wp[3] = {{model_.car_s + 30, lane_no_d}, {model_.car_s + 60, lane_no_d}, {model_.car_s + 90, lane_no_d}};

    for (auto &constraint : constraints_) {
        constraint.Apply(&model_);
    }


    auto speed_per_sec = GetSpeedInMetersPerSec(model_.speed_mph);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (int i = 1; i < 50; i++) {
        FrenetPoint f;
        f.s = model_.car_s + i * speed_per_sec;
        //f.d = model_.car_d;
        f.d = model_.lane_d;

        CartesianPoint c = (CartesianPoint)f;
        next_x_vals.push_back(c.x);
        next_y_vals.push_back(c.y);
    }
}
