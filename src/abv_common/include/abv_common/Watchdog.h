#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

class Watchdog
{
public:
    Watchdog();
    ~Watchdog();

    void start(float aDuration_s, std::function<void()> aCallback);
    void cancel();

private:

    std::thread mThread;
    std::mutex mMtx;
    std::condition_variable mCondition;
    std::atomic<bool> mCancelled{false};

};
#endif //WATCHDOG_H
