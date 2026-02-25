#ifndef ISTATEFETCHER_H
#define ISTATEFETCHER_H

#include <mutex> 
#include <atomic>

#include "common/ConsumableBuffer.hpp"
#include "common/AbvState.hpp"
class IStateFetcher
{
public:
    explicit IStateFetcher(ConsumableBuffer<AbvState>& buffer)
        : mBuffer(buffer), mAcquired(false)
    {}

    virtual ~IStateFetcher() = default;

    virtual bool init() = 0;
    virtual bool isStateAcquired() { return mAcquired.load(); }
    virtual std::string type() { return "unknown"; }

protected:
    ConsumableBuffer<AbvState>& mBuffer;
    std::atomic<bool> mAcquired;
};
#endif // ISTATEFETCHER_H