//
// Created by fahad on 11/29/17.
//

#ifndef PATH_PLANNING_WORLDMAP_H
#define PATH_PLANNING_WORLDMAP_H

#include <string>
#include <vector>

class WorldMap {
public:
    static WorldMap* GetInstance() {
        return &worldmap_;
    }

    bool ReadMap(const std::string filename);

private:
    WorldMap() {
    }

    static WorldMap worldmap_;

public:
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
};


#endif //PATH_PLANNING_WORLDMAP_H
