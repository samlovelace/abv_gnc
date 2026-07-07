#ifndef STAMPEDABVSTATE_H
#define STAMPEDABVSTATE_H

#include <chrono>
#include "abv_common/AbvState.hpp"

// Pairs a raw AbvState measurement with the wall-clock time it was acquired,
// so VehicleStateTracker can bound how long the EKF is allowed to free-run
// (predict-only) without a real measurement.
struct StampedAbvState
{
    AbvState state;
    std::chrono::system_clock::time_point measurementTime;
};

#endif // STAMPEDABVSTATE_H
