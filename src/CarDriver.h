//
// Created by fahad on 11/29/17.
//

#ifndef PATH_PLANNING_CARDRIVER_H
#define PATH_PLANNING_CARDRIVER_H

#include <vector>
#include <array>
#include <memory.h>
#include "json.hpp"
#include "CarModel.h"
#include "spline.h"
#include "CostCalculator.h"

const auto CL_BEHIND_DISTANCE_SECS = 7.0;
const auto CL_FRONT_DISTANCE_SECS = 1.0;
const auto CL_FRONT_ABORT_SECS = 2.0;
const auto CL_SLOW_DOWN_FACTOR = 0.75;      // there is no space in the target lane, slow down by this much factor to create some room
const double MAX_ALLOWED_SPEED_MPH_ = 48.0;
const double MAX_LANE_CHANGE_SPEED_MPH = 45.0;
const double MAX_ACCELERATION = 18.0;      // 10m/s * 60 * 60 / 1000 / 1.60934

using json = nlohmann::json;

class CarDriver {
public:
    CarDriver();

    void UpdateModel(json &x);
    std::array<std::vector<double>, 2> GetPath();

    void set_ideal_speed(double speed) {
        ideal_speed_mph_ = speed;
    }
    void set_desired_lane(int lane) {
        desired_lane_no_ = lane;
    }

public:
#ifdef _DEBUG_DATA
    std::unique_ptr<DebugValues> last_debug_;
    void InitState();
#endif

protected:
    std::vector<double> GetPerPointSpeed(double cur_speed_mph, double required_speed_mph, int points_needed);

    void KeepLaneState();
    void PrepareLaneChangeState();
    void ChangeLaneState(int goto_lane_no, int best_lane_no);

    std::vector<double> DriveAtSpeed(double speed_mph, int spline_distance_start = 30, int spline_distance_inc = 30);

    void FigureOutCarOrigin(CartesianPoint *last_pt, CartesianPoint *last_last_pt, double *ref_yaw);
    tk::spline GetPathToFollow(int spline_distance_start, int spline_distance_inc);
    std::vector<double> GenerateTrajectory(double cur_speed_mph, double required_speed_mph, const tk::spline &path_spline);

    void FigureOutSpeedFromPrevious();
    bool DriveWhileKeepingDistance();

    bool TargetLaneFrontOk(int target_lane_no);
    bool TargetLaneBackOk(int target_lane_no);
    bool EnoughDistanceInFront(int lane_no);

private:
    CarModel model_;
    json previous_path_x_;
    json previous_path_y_;
    std::vector<VehicleSensed> sensor_fusion_;
    std::vector<double> next_x_vals_;
    std::vector<double> next_y_vals_;
    std::function<void()> state_;
    std::unique_ptr<CostCalculator> cost_;

    double ideal_speed_mph_;
    int desired_lane_no_;
};


#endif //PATH_PLANNING_CARDRIVER_H
