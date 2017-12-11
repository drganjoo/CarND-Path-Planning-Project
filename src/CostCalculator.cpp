//
// Created by Fahad Zubair on 10/12/2017.
//

#include "CostCalculator.h"
#include "utils.h"
#include <iostream>

using namespace std;

CostCalculator::CostCalculator(const std::vector<VehicleSensed> &sensor, const CarModel &model) :
        sensor_fusion_(sensor),
        model_(model)
{
}

const VehicleSensed *CostCalculator::GetClosestCarInFront(int lane_no) {
    const VehicleSensed *closest_car = nullptr;

    auto sensor_size = sensor_fusion_.size();
    if (sensor_size !=  0) {

        for (auto &other_car : sensor_fusion_) {
            // only look at cars that are in our lane and ahead of us

            if (other_car.d >= lane_no * 4 && other_car.d <= lane_no * 4 + 4) {

                if (other_car.s > model_.car_s) {
                    if (nullptr == closest_car)
                        closest_car = &other_car;
                    else if (other_car.s < closest_car->s)
                        closest_car = &other_car;
                }
            }
        }
    }

    return closest_car;
}

double CostCalculator::GetDistanceToDeadStop(double speed_mph) {
    auto max_decelerate_mps = 10.0;
    auto speed_mps = speed_mph_to_mtr_per_sec(speed_mph);
    auto meters_to_stop = speed_mps  * (speed_mps / max_decelerate_mps);

    return meters_to_stop;
}

double CatchupCost::GetDistaneToCarMeters(const VehicleSensed *car) {
    CartesianPoint car_in_body_frame = model_.TranslateXYToBodyFrame(car->x, car->y);
    const auto distance_to_car = sqrt(car_in_body_frame.x * car_in_body_frame.x + car_in_body_frame.y * car_in_body_frame.y);
    return distance_to_car;
}

double CatchupCost::CalculateCost(int lane_no, double speed_mph) {
    auto closest_car = GetClosestCarInFront(lane_no);
    if (nullptr == closest_car) {
        cout << "Could not find any car in lane_no: " << lane_no << endl;
        return 0.0;     // empty road, no cost at all in driving in this lane
    }

    auto distance_meters = GetDistaneToCarMeters(closest_car);
    auto our_speed_mps = speed_mph_to_mtr_per_sec(speed_mph);
    auto other_car_speed_mps = closest_car->x_dot;
    auto delta_speed = our_speed_mps - other_car_speed_mps;

    auto catchup_distance = distance_meters / delta_speed;
    auto cost = 1 - exp(-abs(catchup_distance));

    cout << "Calculate Cost (" << lane_no << ") : distance: " << distance_meters << ", our speed_mps: " << our_speed_mps
         << ", closes_car speed: " << other_car_speed_mps
         << ", catchup_distance: " << catchup_distance
         << ", cost: " << cost << endl;

    return cost;
}
