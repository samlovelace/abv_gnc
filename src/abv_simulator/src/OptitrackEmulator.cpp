#include "abv_simulator/OptitrackEmulator.h"

#include <cmath>
#include <sys/time.h>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#include "plog/Log.h"

OptitrackEmulator::OptitrackEmulator(const std::string& aRigidBodyName, int aPort)
    : mRigidBodyName(aRigidBodyName), mPort(aPort)
{
    mRunning.store(false);
}

OptitrackEmulator::~OptitrackEmulator()
{
    mRunning.store(false);

    if (mServiceThread.joinable())
    {
        mServiceThread.join();
    }
}

bool OptitrackEmulator::init()
{
    mConnection = std::shared_ptr<vrpn_Connection>(vrpn_create_server_connection(mPort));

    if (!mConnection || !mConnection->doing_okay())
    {
        LOGE << "OptitrackEmulator failed to open VRPN server connection on port " << mPort;
        return false;
    }

    mTracker = std::make_unique<vrpn_Tracker_Server>(mRigidBodyName.c_str(), mConnection.get());

    mRunning.store(true);
    mServiceThread = std::thread(&OptitrackEmulator::serviceConnection, this);

    LOGV << "OptitrackEmulator streaming rigid body '" << mRigidBodyName
         << "' via VRPN on port " << mPort;
    return true;
}

void OptitrackEmulator::publishPose(double aX, double aY, double aYaw)
{
    if (!mTracker)
    {
        return;
    }

    struct timeval now;
    gettimeofday(&now, nullptr);

    const double position[3] = {aX, aY, 0.0};

    // quaternion ordering expected by vrpn/libmotioncapture is [x, y, z, w]
    const double halfYaw = aYaw / 2.0;
    const double quaternion[4] = {0.0, 0.0, std::sin(halfYaw), std::cos(halfYaw)};

    mTracker->report_pose(0, now, position, quaternion);
}

void OptitrackEmulator::serviceConnection()
{
    while (mRunning.load())
    {
        mConnection->mainloop();
        mTracker->mainloop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
