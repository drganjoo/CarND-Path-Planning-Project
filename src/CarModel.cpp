//
// Created by Fahad Zubair on 01/12/2017.
//

#include "CarModel.h"

void to_json(json& j, const CartesianPoint& cp){
    j = { cp.x, cp.y };
}

void to_json(json& j, const FrenetPoint& fp){
    j = {fp.s, fp.d};
}

void to_json(json& j, const CarModel& cm) {
    j["speedMph"] = cm.cur_speed_mph;
    j["x"] = cm.car_x;
    j["y"] = cm.car_y;
    j["s"] = cm.car_s;
    j["d"] = cm.car_d;
    j["yaw"] = cm.car_yaw;
}

void to_json(json& j, const DebugRefYaw& yaw){
    j["prev"] = yaw.ref_prev;
    j["prevPrev"] = yaw.ref_prev_prev;
    j["basedOn"] = yaw.based_on;
    j["angleRad"] = yaw.angle_rad;
}

void to_json(json& j, const YawCalculation &y) {
    j = y == YawCalculation::CarYaw ? "car_yaw" : "previous_xy";
}

void to_json(json& j, const DebugValues& dv){
    j["type"] = "debug";
    j["acqTime"] = std::chrono::system_clock::to_time_t(dv.acq_time);

    j["model"] = dv.model;
    auto &model_json = j["model"];
    model_json["desiredSpeedMph"] = dv.desired_speed_mph;
    model_json["desiredLaneNo"] = dv.desired_lane_no;

    j["refYaw"] = dv.ref_yaw;
    j["splinePts"] = dv.spline_pts;
    j["splineAddonsFrenet"] = dv.spline_addons;
    j["splineGeneratedPts"] = dv.spline_generated_pts;
    j["previousPts"] = dv.previous_pts;
    j["nextPts"] = dv.next_pts;
    j["sensorFusion"] = dv.sensor_fusion;
    j["state"] = dv.state;
    j["debugMessage"] = dv.debug_message;
}

void to_json(json &j, const VehicleSensed &v) {
    j["car_id"] = v.car_id;
    j["s"] = v.s;
    j["d"] = v.d;
    j["x"] = v.x;
    j["y"] = v.y;
}