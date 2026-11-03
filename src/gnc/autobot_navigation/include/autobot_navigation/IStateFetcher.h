#ifndef ISTATEFETCHER_H
#define ISTATEFETCHER_H

#include <mutex> 
#include <atomic>

#include "autobot_common/ConsumableBuffer.hpp"
#include "autobot_common/State.hpp"
#include "autobot_navigation/StampedState.h"
class IStateFetcher
{
public:
    explicit IStateFetcher(ConsumableBuffer<StampedState>& buffer)
        : mBuffer(buffer), mAcquired(false)
    {}

    virtual ~IStateFetcher() = default;

    virtual bool init() = 0;
    virtual bool isStateAcquired() { return mAcquired.load(); }
    virtual std::string type() { return "unknown"; }

protected:
    ConsumableBuffer<StampedState>& mBuffer;
    std::atomic<bool> mAcquired;
};
#endif // ISTATEFETCHER_H