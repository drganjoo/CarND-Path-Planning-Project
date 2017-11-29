//
// Created by fahad on 11/29/17.
//

#ifndef PATH_PLANNING_CARDRIVER_H
#define PATH_PLANNING_CARDRIVER_H

#include <vector>
#include <array>
#include <memory.h>
#include "json.hpp"
#include "DrivingConstraints.h"
#include "CarModel.h"

using json = nlohmann::json;

class CarDriver {
public:
    CarDriver() {
        memset(&model_, 0, sizeof(model_));
        end_path_s_ = 0.0;
        end_path_d_ = 0.0;
    }

    void UpdateModel(json &x);

    void set_ideal_speed(double speed) {
        model_.desired_speed_mph = speed;
    }

    double get_ideal_speed() {
        return model_.desired_speed_mph;
    }

    void set_lane(int lane) {
        model_.lane_d = 2 + lane * 4.0;
    }

    std::array<std::vector<double>, 2> GetPath();

private:
    double GetSpeedInMetersPerSec(double speed_mph);
    void Process();

private:
    CarModel model_;
    json previous_path_x_;
    json previous_path_y_;
    double end_path_s_;
    double end_path_d_;
    std::vector<VehicleSensed> sensor_fusion_;
    std::vector<DrivingConstraints> constraints_;
    std::vector<double> next_x_vals_;
    std::vector<double> next_y_vals_;
};


#endif //PATH_PLANNING_CARDRIVER_H
