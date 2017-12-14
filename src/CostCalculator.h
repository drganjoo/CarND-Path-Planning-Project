//
// Created by Fahad Zubair on 10/12/2017.
//

#ifndef PATH_PLANNING_COSTCALCULATOR_H
#define PATH_PLANNING_COSTCALCULATOR_H

#include <memory>
#include "CarModel.h"

const double MAX_SPEED_MPH_ = 100;

class CostCalculator {
public:
    CostCalculator(const std::vector<VehicleSensed> &sensor, const CarModel &model);
    double GetDistanceToDeadStop(double speed_mph);

    const VehicleSensed *GetClosestCarInFront(int lane_no);
    const VehicleSensed *GetClosestCarBehind(int lane_no);

    double GetLaneSpeedMph(int lane_no);
    double GetDistaneToCarMeters(const VehicleSensed *car);

protected:
    const VehicleSensed *GetClosestCar(int lane_no, std::function<bool(const VehicleSensed&)> comparator);

protected:
    const std::vector<VehicleSensed> &sensor_fusion_;
    const CarModel &model_;
};

class SpeedCostCalculator : public CostCalculator {
public:
    SpeedCostCalculator (const std::vector<VehicleSensed> &sensor, const CarModel &model)
            : CostCalculator(sensor, model)
    {
    }

    double CalculateCost(int lane_no, double *speed_mph);
};

class SpeedDistanceCostCalculator : public CostCalculator {
public:
    SpeedDistanceCostCalculator(const std::vector<VehicleSensed> &sensor, const CarModel &model)
            : CostCalculator(sensor, model)
    {
    }

    double CalculateCost(int lane_no, double *speed_mph);
};


#endif //PATH_PLANNING_COSTCALCULATOR_H
