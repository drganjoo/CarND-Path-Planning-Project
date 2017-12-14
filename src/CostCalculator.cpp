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

const VehicleSensed *CostCalculator::GetClosestCar(int lane_no, std::function<bool(const VehicleSensed&)> comparator) {
    const VehicleSensed *closest_car = nullptr;

    auto sensor_size = sensor_fusion_.size();
    if (sensor_size !=  0) {

        for (auto &other_car : sensor_fusion_) {
            // only look at cars that are in our lane and ahead of us

            if (other_car.d >= lane_no * 4 && other_car.d <= lane_no * 4 + 4) {

//                if (other_car.s > model_.car_s) {
                if (comparator(other_car)) {
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

const VehicleSensed *CostCalculator::GetClosestCarInFront(int lane_no) {
    return GetClosestCar(lane_no, [this](const VehicleSensed &other_car) {
       return other_car.s > model_.car_s;
    });
//    const VehicleSensed *closest_car = nullptr;
//
//    auto sensor_size = sensor_fusion_.size();
//    if (sensor_size !=  0) {
//
//        for (auto &other_car : sensor_fusion_) {
//            // only look at cars that are in our lane and ahead of us
//
//            if (other_car.d >= lane_no * 4 && other_car.d <= lane_no * 4 + 4) {
//
//                if (other_car.s > model_.car_s) {
//                    if (nullptr == closest_car)
//                        closest_car = &other_car;
//                    else if (other_car.s < closest_car->s)
//                        closest_car = &other_car;
//                }
//            }
//        }
//    }
//
//    return closest_car;
}

const VehicleSensed *CostCalculator::GetClosestCarBehind(int lane_no) {
    return GetClosestCar(lane_no, [this](const VehicleSensed &other_car) {
        return other_car.s < model_.car_s;
    });
}

double CostCalculator::GetDistanceToDeadStop(double speed_mph) {
    auto max_decelerate_mps = 10.0;
    auto speed_mps = speed_mph_to_mtr_per_sec(speed_mph);
    auto meters_to_stop = speed_mps  * (speed_mps / max_decelerate_mps);

    return meters_to_stop;
}

double CostCalculator::GetLaneSpeedMph(int lane_no) {
    auto closest_car = GetClosestCarInFront(lane_no);
    if (!closest_car)
        return MAX_SPEED_MPH_;

    const auto traffic_speed_mph_ = speed_mtr_per_sec_to_mph(closest_car->speed_mps());
    return traffic_speed_mph_ ;
}

double CostCalculator::GetDistaneToCarMeters(const VehicleSensed *car) {
    CartesianPoint car_in_body_frame = model_.TranslateXYToBodyFrame(car->x, car->y);
    const auto distance_to_car = sqrt(car_in_body_frame.x * car_in_body_frame.x + car_in_body_frame.y * car_in_body_frame.y);
    return distance_to_car;
}

double SpeedDistanceCostCalculator::CalculateCost(int lane_no, double *speed_mph) {
    auto closest_car = GetClosestCarInFront(lane_no);
    if (nullptr == closest_car) {
        cout << lane_no << ", " << 999999
             << ", " << *speed_mph
             << ", " << 0
             << ", " << 999999
             << ", " << 999999
             << ", " << 0.0 << endl;
        return 0.0;     // empty road, no cost at all in driving in this lane
    }

    auto distance_meters = GetDistaneToCarMeters(closest_car);
    auto our_speed_mps = speed_mph_to_mtr_per_sec(*speed_mph);
    auto other_car_speed_mps = closest_car->speed_mps();
    auto delta_speed = our_speed_mps - other_car_speed_mps;

    auto catchup_seconds = distance_meters / delta_speed;
    const auto SECS_TO_ZERO_COST = 6.0;
    auto catchup_seconds_percent = catchup_seconds / SECS_TO_ZERO_COST;

    auto cost = exp(-abs(catchup_seconds_percent));

    cout << lane_no << ", " << distance_meters
            << ", " << our_speed_mps
            << ", " << other_car_speed_mps
            << ", " << catchup_seconds
            << ", " << catchup_seconds_percent
            << ", " << cost << endl;

    return cost;
}

double SpeedCostCalculator::CalculateCost(int lane_no, double *speed_mph) {
    double lane_speed;
    double distance_meters;

    auto closest_car = GetClosestCarInFront(lane_no);
    if (!closest_car) {
        cout << "~~~~NO CAR IN Lane: " << lane_no << endl;

        lane_speed = MAX_SPEED_MPH_;
        distance_meters = 999999;
    }
    else {
        lane_speed = speed_mtr_per_sec_to_mph(closest_car->speed_mps());
        distance_meters = GetDistaneToCarMeters(closest_car);
    }

    double cost = (*speed_mph - lane_speed) / *speed_mph;
    if (cost < 0)
        cost = 0;

    *speed_mph = lane_speed;

    cout << lane_no << ", " << '*'
         << ", " << *speed_mph
         << ", " << lane_speed
         << ", " << distance_meters
         << ", " << '*'
         << ", " << cost << endl;

    return cost;
}
