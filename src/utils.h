//
// Created by Fahad Zubair on 10/12/2017.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>

__inline double speed_mph_to_mtr_per_sec(double speed_mph) {
    return (speed_mph * 1.60934 * 1000.0) / (60.0 * 60.0);
}

__inline double speed_mtr_per_sec_to_mph(double speed_mps) {
    return (speed_mps * 60 * 60 ) / (1.60934 * 1000.0);
}

__inline double GetDesiredFrenetLaneNo(double lane_no) {
    return 2 + lane_no * 4;
}

std::vector<double> getFrenet(double x, double y, double theta);

#endif //PATH_PLANNING_UTILS_H
