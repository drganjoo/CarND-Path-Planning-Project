//
// Created by fahad on 11/29/17.
//

#ifndef PATH_PLANNING_CARMODEL_H
#define PATH_PLANNING_CARMODEL_H

#include <cmath>
#include "WorldMap.h"
#include "json.hpp"
#include <chrono>

using json = nlohmann::json;

struct VehicleSensed {
    int car_id;
    double x;
    double y;
    double x_dot;
    double y_dot;
    double s;
    double d;

    double speed_mps() const {
        auto velocity = sqrt(x_dot * x_dot + y_dot * y_dot);
        return velocity;
    }
};

struct CartesianPoint {
    double x;
    double y;

    CartesianPoint(){}

    CartesianPoint(double x, double y) {
        this->x = x;
        this->y = y;
    }

    void reset() {
        x = y = 0.0;
    }
};

struct FrenetPoint {
    double s;
    double d;

    void reset() {
        s = d = 0.0;
    }

    explicit operator CartesianPoint(){
        WorldMap *map = WorldMap::GetInstance();

        int prev_wp = -1;

        while (s > map->map_waypoints_s[prev_wp + 1] && (prev_wp < (int) (map->map_waypoints_s.size() - 1))) {
            prev_wp++;
        }

        auto wp2 = (prev_wp + 1) % map->map_waypoints_x.size();

        double heading = atan2((map->map_waypoints_y[wp2] - map->map_waypoints_y[prev_wp]), (map->map_waypoints_x[wp2] - map->map_waypoints_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s - map->map_waypoints_s[prev_wp]);

        double seg_x = map->map_waypoints_x[prev_wp] + seg_s * cos(heading);
        double seg_y = map->map_waypoints_y[prev_wp] + seg_s * sin(heading);

        double perp_heading = heading - M_PI / 2;

        return CartesianPoint(seg_x + d * cos(perp_heading), seg_y + d * sin(perp_heading));
    }
};

struct CarModel {
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double cur_speed_mph;

    CartesianPoint ref_prev_prev;
    CartesianPoint ref_prev;
    double ref_yaw;
    double ref_speed_mph;

    CarModel() :
            car_x(0),
            car_y(0),
            car_s (0),
            car_d (0),
            car_yaw (0),
            cur_speed_mph (0),
            ref_yaw (0),
            ref_speed_mph (0)
    {
    }

    CarModel(const json &j) :
            car_x(j["x"]),
            car_y(j["y"]),
            car_s(j["s"]),
            car_d(j["d"]),
            car_yaw(j["yaw"]),
            cur_speed_mph(j["speed"])
    {
        ref_yaw = 0.0;
        ref_speed_mph = 0.0;
    }

    CartesianPoint TranslateXYToBodyFrame(const double x, const double y) const{
        const double origin_x = ref_prev.x;
        const double origin_y = ref_prev.y;
        // shift back to start of car
        double x_translated = x - origin_x;
        double y_translated = y - origin_y;

        CartesianPoint pt_in_body;

        pt_in_body.x = x_translated * cos(0 - ref_yaw) - y_translated * sin(0-ref_yaw);
        pt_in_body.y = x_translated * sin(0 - ref_yaw) + y_translated * cos(0-ref_yaw);

        return pt_in_body;
    }
};


/*------------------------------------    DEBUG Values   ------------------------------------------ */


enum class YawCalculation { CarYaw, PreviousPoints };

struct DebugRefYaw {
    CartesianPoint ref_prev;
    CartesianPoint ref_prev_prev;
    YawCalculation based_on;
    double angle_rad;

    void reset() {
        ref_prev.reset();
        ref_prev_prev.reset();
        based_on = YawCalculation::CarYaw;
        angle_rad = 0.0;
    }
};



struct DebugValues {
    std::chrono::system_clock::time_point acq_time;
    CarModel model;
    std::vector<FrenetPoint> spline_addons;     // extra points added at the end of the previous ones
    std::vector<CartesianPoint> spline_pts;     // points based on which spline was generated
    std::vector<CartesianPoint> previous_pts;
    std::vector<CartesianPoint> next_pts;
    std::vector<CartesianPoint> spline_generated_pts;
    std::vector<VehicleSensed> sensor_fusion;
    DebugRefYaw ref_yaw;
    double desired_speed_mph;
    int desired_lane_no;
    int target_lane_no;
    std::string state;
    std::string debug_message;

    DebugValues(const CarModel &init_model) :
            model(init_model)
    {
        acq_time = std::chrono::system_clock::now();
        desired_speed_mph = -1;
        desired_lane_no = -1;
        target_lane_no = -1;
    }
};


void to_json(json& j, const CartesianPoint& cp);
void to_json(json& j, const FrenetPoint& fp);
void to_json(json& j, const CarModel& cm);
void to_json(json& j, const DebugValues& dv);
void to_json(json& j, const DebugRefYaw& yaw);
void to_json(json& j, const YawCalculation &y);
void to_json(json& j, const VehicleSensed &v);

#endif //PATH_PLANNING_CARMODEL_H
