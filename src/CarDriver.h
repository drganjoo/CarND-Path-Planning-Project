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
#include "spline.h"

using json = nlohmann::json;

class CarDriver {
public:
    CarDriver() {
        end_path_s_ = 0.0;
        end_path_d_ = 0.0;
    }

    void UpdateModel(json &x);

    void set_ideal_speed(double speed) {
        desired_speed_mph_ = speed;
    }

    double get_ideal_speed();
    std::vector<double> GetPerPointSpeed(double speed_cur_mp20, int points_needed);

    void set_desired_lane(int lane) {
        desired_lane_no_ = lane;
    }

    double speed_mph_to_mtr_per_sec(double speed_mph) {
        return (speed_mph * 1.60934 * 1000.0) / (60.0 * 60.0);
    }
    double speed_mtr_per_sec_to_mph(double speed_mps) {
        return (speed_mps * 60 * 60 ) / (1.60934 * 1000.0);
    }

    std::array<std::vector<double>, 2> GetPath();

    double GetDesiredFrenetLaneNo() const {
        return 2 + desired_lane_no_ * 4;
    }

public:
    std::unique_ptr<DebugValues> last_debug_;
//    std::vector<DebugValues> debug_packets_;

private:
    void FigureOutCarOrigin(CartesianPoint *last_pt, CartesianPoint *last_last_pt, double *ref_yaw);
//    double HowManyMetersTravelledIn20ms(double speed_mph);
    void Process();
    tk::spline GetSpline(const CartesianPoint &ref_prev, CartesianPoint &ref_prev_prev, double ref_yaw);

private:
    std::unique_ptr<CarModel> model_;
    json previous_path_x_;
    //std::vector<double> previous_path_x_;
    json previous_path_y_;
    double end_path_s_;
    double end_path_d_;
    std::vector<VehicleSensed> sensor_fusion_;
    std::vector<DrivingConstraints> constraints_;
    std::vector<double> next_x_vals_;
    std::vector<double> next_y_vals_;

    double desired_speed_mph_;
    int desired_lane_no_;
};


#endif //PATH_PLANNING_CARDRIVER_H
