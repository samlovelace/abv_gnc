
#include "abv_common/Watchdog.h"
#include <chrono>

Watchdog::Watchdog()
{

}

Watchdog::~Watchdog()
{
    cancel();
}

void Watchdog::start(float aDuration_s, std::function<void()> aCallback)
{
    cancel();
    mCancelled = false;

    auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<float>(aDuration_s));

    mThread = std::thread([this, delay, aCallback]()
    {
        std::unique_lock<std::mutex> lock(mMtx);
        if (!mCondition.wait_for(lock, delay, [this]{ return mCancelled.load(); })) {
            // Timed out (not cancelled) — fire the callback
            aCallback();
        }
    });

}

void Watchdog::cancel()
{
    mCancelled = true;
    mCondition.notify_one();

    if (mThread.joinable())
        mThread.join();
}
