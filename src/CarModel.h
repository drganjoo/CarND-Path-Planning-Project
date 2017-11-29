//
// Created by fahad on 11/29/17.
//

#ifndef PATH_PLANNING_CARMODEL_H
#define PATH_PLANNING_CARMODEL_H

#include <cmath>
#include "WorldMap.h"

struct VehicleSensed {
    int car_id;
    double x;
    double y;
    double x_dot;
    double y_dot;
    double s;
    double d;
};

struct CarModel {
    double desired_speed_mph;
    int desired_lane_no;
    double speed_mph;
    double lane_d;
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;

    double GetDesiredFrenetLaneNo() {
        return 2 + desired_lane_no * 4;
    }
};

struct CartesianPoint {
    double x;
    double y;
};

struct FrenetPoint {
    double s;
    double d;

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

        CartesianPoint pt{
          .x = seg_x + d * cos(perp_heading),
          .y = seg_y + d * sin(perp_heading)
        };

        return pt;
    }
};
#endif //PATH_PLANNING_CARMODEL_H
