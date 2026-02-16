
#include "abv_navigation/OptitrackStateFetcher_LibMocap.h"
#include "plog/Log.h"
#include <iostream> 

OptitrackStateFetcher_LibMocap::OptitrackStateFetcher_LibMocap(const std::string& aServerIp, 
                                                               const std::string& aLocalIp, 
                                                               const std::string& aRigidBodyName) : 
    mRigidBodyName(aRigidBodyName), mServerIp(aServerIp), mLocalIp(aLocalIp)
{
    mAcquired.store(false); 

}

OptitrackStateFetcher_LibMocap::~OptitrackStateFetcher_LibMocap()
{
    if(mListenThread.joinable())
    {
        mListenThread.join(); 
    }
}

bool OptitrackStateFetcher_LibMocap::init()
{
    std::map<std::string, std::string> cfg; 
    cfg.insert({"hostname", mServerIp}); 
    cfg.insert({"interface_ip", mLocalIp}); 
    
    mMocap = std::unique_ptr<libmotioncapture::MotionCapture>(
        libmotioncapture::MotionCapture::connect("vrpn", cfg)
    );

    if(!mMocap)
    {
        LOGD << "Failed to initialize mocap client"; 
        return false; 
    }

    LOGV << "mocap client initialized successfully"; 
    mListenThread = std::thread(&OptitrackStateFetcher_LibMocap::listen, this); 
    return true; 
}


AbvState OptitrackStateFetcher_LibMocap::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mLatestState; 
} 

void OptitrackStateFetcher_LibMocap::setLatestState(const AbvState& aLatestState)
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    mLatestState = aLatestState; 
}

void OptitrackStateFetcher_LibMocap::listen()
{
    while(true)
    {
        // Get a frame
        mMocap->waitForNextFrame();

        if (mMocap->supportsRigidBodyTracking()) 
        {
            auto rigidBodies = mMocap->rigidBodies();

            if(rigidBodies.empty())
            {
                continue;
            }

            auto dt = std::chrono::steady_clock::now() - mPrevRecvdTime; 

            for (auto const& item : rigidBodies) 
            {
                double roll, pitch, yaw;
                const auto& rigidBody = item.second;
                
                if(rigidBody.name() == mRigidBodyName)
                {
                    Eigen::Vector3f pos = rigidBody.position(); 

                    // take the xy location 
                    AbvState state; 
                    state.x = pos(0); 
                    state.y = pos(1);  

                    // compute linear velocity 
                    for(int i=0; i<3; i++)
                    {   
                        state.vx = (state.x - mPrevState.x) / dt.count(); 
                        state.vy = (state.y - mPrevState.y) / dt.count();  
                    }

                    Eigen::Quaternionf q = rigidBody.rotation(); 
                    q.normalize();

                    // Convert quaternion to Euler angles
                    double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

                    // Yaw (z-axis rotation)
                    double siny_cosp = 2 * (qw * qz + qx * qy);
                    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
                    yaw = std::atan2(siny_cosp, cosy_cosp);

                    // assign yaw to latest state 
                    state.theta = yaw; 

                    // angular velocity
                    Eigen::Quaternionf qDelta = q * mPrevQuat.inverse();
                    Eigen::Vector3f angularVelocity = 2.0 * qDelta.vec() / dt.count(); 

                    state.omega = angularVelocity.z(); 

                    // set latest state and update previous values
                    setLatestState(state); 
                    mPrevState = state; 
                    mPrevQuat = q; 
                    mPrevRecvdTime = std::chrono::steady_clock::now();  
                    
                    if(!mAcquired.load())
                        mAcquired.store(true); 
                }
            }
        }
    }
}