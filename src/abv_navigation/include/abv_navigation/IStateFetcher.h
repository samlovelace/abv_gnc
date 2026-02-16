#ifndef ISTATEFETCHER_H
#define ISTATEFETCHER_H

#include <mutex> 
#include <atomic>

#include "common/AbvState.hpp"

class IStateFetcher
{
public:
    virtual ~IStateFetcher() = default; 
    virtual AbvState fetchState() = 0; 
    virtual bool init() = 0; 
    virtual bool isStateAcquired() {return mAcquired.load(); }
    virtual std::string type() {return "unknown"; }

protected: 
    AbvState mState; 
    std::atomic<bool> mAcquired; 

};
#endif // ISTATEFETCHER_H