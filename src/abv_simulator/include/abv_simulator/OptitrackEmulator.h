#ifndef OPTITRACKEMULATOR_H
#define OPTITRACKEMULATOR_H

#include <atomic>
#include <memory>
#include <string>
#include <thread>

class vrpn_Connection;
class vrpn_Tracker_Server;

// Emulates an OptiTrack rigid body stream by standing up a real VRPN server
// and publishing rigid body poses over it. This lets OptitrackStateFetcher_LibMocap
// (abv_navigation) connect to it exactly as it would to the OptiTrack VRPN
// bridge on the real vehicle, without any changes to that class.
class OptitrackEmulator
{
public:
    OptitrackEmulator(const std::string& aRigidBodyName, int aPort);
    ~OptitrackEmulator();

    // Starts the VRPN server connection and the background thread that
    // services it. Must be called before publishPose().
    bool init();

    // Publishes a rigid body pose for the configured rigid body name.
    // aYaw is the rotation about the vertical (z) axis, in radians.
    void publishPose(double aX, double aY, double aYaw);

private:
    void serviceConnection();

    std::string mRigidBodyName;
    int mPort;

    std::shared_ptr<vrpn_Connection> mConnection;
    std::unique_ptr<vrpn_Tracker_Server> mTracker;

    std::atomic<bool> mRunning;
    std::thread mServiceThread;
};
#endif // OPTITRACKEMULATOR_H
