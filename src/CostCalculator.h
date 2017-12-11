//
// Created by Fahad Zubair on 10/12/2017.
//

#ifndef PATH_PLANNING_COSTCALCULATOR_H
#define PATH_PLANNING_COSTCALCULATOR_H

#include <memory>
#include "CarModel.h"

class CostCalculator {
public:
    CostCalculator(const std::vector<VehicleSensed> &sensor, const CarModel &model);
    double GetDistanceToDeadStop(double speed_mph);

    const VehicleSensed *GetClosestCarInFront(int lane_no);

protected:
    const std::vector<VehicleSensed> &sensor_fusion_;
    const CarModel &model_;
};


class CatchupCost : public CostCalculator {
public:
    CatchupCost(const std::vector<VehicleSensed> &sensor, const CarModel &model)
            : CostCalculator(sensor, model)
    {
    }

    double CalculateCost(int lane_no, double speed_mph);

private:
    double GetDistaneToCarMeters(const VehicleSensed *car);
};


#endif //PATH_PLANNING_COSTCALCULATOR_H
