#ifndef SCENETYPES_HPP
#define SCENETYPES_HPP

// Internal, ROS-message-free obstacle representation - Scene converts
// Obstacle msgs to/from this on the wire, same role InternalTypes.hpp's
// Waypoint/Command play for the rest of autobot_guidance.
struct Obstacle
{
    double mX;
    double mY;
    double mRadius;
};

#endif // SCENETYPES_HPP
