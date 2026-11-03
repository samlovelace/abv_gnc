#ifndef PATHWATCHDOG_H
#define PATHWATCHDOG_H
 
#include <functional>
#include <thread> 
#include <mutex> 
#include <condition_variable>
#include <atomic>

class PathWatchdog 
{ 
public:
    PathWatchdog();
    ~PathWatchdog();
    
    void start(float aDuration_s, std::function<void()> aCallback);
    void cancel(); 

private:

    std::thread mThread; 
    std::mutex mMtx; 
    std::condition_variable mCondition; 
    std::atomic<bool> mCancelled{false}; 
    
};
#endif //PATHWATCHDOG_H