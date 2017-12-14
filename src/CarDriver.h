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

using json = nlohmann::json;

class CarDriver {
public:
    CarDriver();

    void UpdateModel(json &x);
    std::vector<double> GetPerPointSpeed(double cur_speed_mph, double required_speed_mph, int points_needed);
    std::array<std::vector<double>, 2> GetPath();

    void set_ideal_speed(double speed) {
        ideal_speed_mph_ = speed;
    }
    void set_desired_lane(int lane) {
        desired_lane_no_ = lane;
    }

public:
    std::unique_ptr<DebugValues> last_debug_;
//    std::vector<DebugValues> debug_packets_;

private:
    void InitState();
    void KeepLaneState();
    void PrepareLaneChangeState();
    void ChangeLaneState(int goto_lane_no, int best_lane_no);

//    void DoState();
    void FigureOutCarOrigin(CartesianPoint *last_pt, CartesianPoint *last_last_pt, double *ref_yaw);
//    double HowManyMetersTravelledIn20ms(double cur_speed_mph);
    std::vector<double> DriveAtSpeed(double speed_mph, int spline_distance_start = 30, int spline_distance_inc = 30);
    tk::spline GetPathToFollow(int spline_distance_start, int spline_distance_inc);
    bool CloseToCar(double speed);
    std::vector<double> GenerateTrajectory(double cur_speed_mph, double required_speed_mph, const tk::spline &path_spline);
    double GetMetersToStop(double speed_mph);
    CartesianPoint TranslateXYToBodyFrame(const double x, const double y);
    double GetLaneCosts(int current_lane, int intended_lane, int final_lane);
//    VehicleSensed* GetClosestCarInFront(int lane_no);

    double CostSpeed(double intended_speed, double target_speed);
    double CostDrivingInCurrentLane(double *lane_speed_ptr);
    void FigureOutSpeedFromPrevious();
    bool DriveWhileKeepingDistance();

//    double GetLaneSpeedMph(int lane_no);

private:
    //std::unique_ptr<CarModel> model_;
    CarModel model_;
    json previous_path_x_;
    //std::vector<double> previous_path_x_;
    json previous_path_y_;
    double end_path_s_;
    double end_path_d_;
    std::vector<VehicleSensed> sensor_fusion_;
    std::vector<double> next_x_vals_;
    std::vector<double> next_y_vals_;
//    DrivingState state_;
    std::function<void()> state_;
    std::unique_ptr<CostCalculator> cost_;

    double ideal_speed_mph_;
    int desired_lane_no_;
};


#endif //PATH_PLANNING_CARDRIVER_H
