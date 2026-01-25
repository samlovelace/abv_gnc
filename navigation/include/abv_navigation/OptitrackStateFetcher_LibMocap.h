#ifndef OPTITRACKSTATEFETCHER_LIBMOCAP_H
#define OPTITRACKSTATEFETCHER_LIBMOCAP_H
 
#include <memory>
#include <thread> 

#include "IStateFetcher.h"
#include "common/ConfigurationManager.h"
#include "common/Configurations.h" 
#include "libmotioncapture/motioncapture.h"

class OptitrackStateFetcher_LibMocap : public IStateFetcher
{ 
public:
    OptitrackStateFetcher_LibMocap(const std::string& aServerIp, 
                                   const std::string& aLocalIp, 
                                   const std::string& aRigidBodyName);

    ~OptitrackStateFetcher_LibMocap() override; 

    bool init() override;
    AbvState fetchState() override;  
    std::string type() {return "OptiTrack"; }

private:

    AbvState mLatestState; 
    AbvState mPrevState; 

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
    void setLatestState(const AbvState& aLatestState);
   
};
#endif //OPTITRACKSTATEFETCHER_LIBMOCAP_H