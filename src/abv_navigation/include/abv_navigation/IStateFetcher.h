#ifndef ISTATEFETCHER_H
#define ISTATEFETCHER_H

#include <mutex> 
#include <atomic>

#include "abv_common/ConsumableBuffer.hpp"
#include "abv_common/AbvState.hpp"
#include "abv_navigation/StampedAbvState.h"
class IStateFetcher
{
public:
    explicit IStateFetcher(ConsumableBuffer<StampedAbvState>& buffer)
        : mBuffer(buffer), mAcquired(false)
    {}

    virtual ~IStateFetcher() = default;

    virtual bool init() = 0;
    virtual bool isStateAcquired() { return mAcquired.load(); }
    virtual std::string type() { return "unknown"; }

protected:
    ConsumableBuffer<StampedAbvState>& mBuffer;
    std::atomic<bool> mAcquired;
};
#endif // ISTATEFETCHER_H