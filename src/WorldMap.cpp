//
// Created by fahad on 11/29/17.
//

#include "WorldMap.h"
#include <fstream>
#include <sstream>

using namespace std;

WorldMap WorldMap::worldmap_;

bool WorldMap::ReadMap(const std::string filename) {
    ifstream in_map(filename.c_str(), ifstream::in);

    if (!in_map)
        return false;

    string line;
    while (getline(in_map, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    return true;
}
