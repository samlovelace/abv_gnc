#ifndef NAVFRESHNESS_H
#define NAVFRESHNESS_H

#include <chrono>

// Pure helper (no ROS/threads) so the dead-reckoning bound check can be unit
// tested directly with fabricated time_points.
inline bool isWithinDeadReckonBound(std::chrono::system_clock::time_point aNow,
                                     std::chrono::system_clock::time_point aLastMeasurementTime,
                                     double aBoundDuration_s)
{
    return std::chrono::duration<double>(aNow - aLastMeasurementTime).count() <= aBoundDuration_s;
}

#endif // NAVFRESHNESS_H
