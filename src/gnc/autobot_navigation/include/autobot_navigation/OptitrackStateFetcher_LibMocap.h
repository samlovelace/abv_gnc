#ifndef OPTITRACKSTATEFETCHER_LIBMOCAP_H
#define OPTITRACKSTATEFETCHER_LIBMOCAP_H
 
#include <memory>
#include <thread> 

#include "IStateFetcher.h"
#include "autobot_config/GncConfigManager.h"
#include "autobot_config/GncConfigurations.h" 
#include "libmotioncapture/motioncapture.h"

class OptitrackStateFetcher_LibMocap : public IStateFetcher
{ 
public:
    OptitrackStateFetcher_LibMocap(ConsumableBuffer<StampedState>& aBuffer,
                                   const std::string& aServerIp,
                                   const std::string& aLocalIp,
                                   const std::string& aRigidBodyName);

    ~OptitrackStateFetcher_LibMocap() override; 

    bool init() override;  
    std::string type() {return "OptiTrack"; }

private:

    State mLatestState; 
    State mPrevState; 

    Eigen::Quaternionf mPrevQuat; 

    std::unique_ptr<libmotioncapture::MotionCapture> mMocap; 
    
    std::string mServerIp; 
    std::string mLocalIp; 
    std::string mRigidBodyName; 
    std::mutex mStateMutex; 
    std::chrono::steady_clock::time_point mPrevRecvdTime; 
    std::thread mListenThread; 
    
private: 

    void listen(); 
    void setLatestState(const State& aLatestState);
   
};
#endif //OPTITRACKSTATEFETCHER_LIBMOCAP_H