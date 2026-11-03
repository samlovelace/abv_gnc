#ifndef BOUNDED_QUEUE_HPP
#define BOUNDED_QUEUE_HPP

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <utility>

template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(size_t cap = 1) : mCap(cap) {}

    void push(T item) 
    {
        { 
            std::lock_guard<std::mutex> lk(mMutex);
            if (mQ.size() >= mCap) mQ.pop_front();   // drop oldest
            mQ.push_back(std::move(item)); 
        }

        mCv.notify_one();
    }

    bool waitPop(T& out, std::chrono::milliseconds timeout) 
    {
        std::unique_lock<std::mutex> lk(mMutex);
        if (!mCv.wait_for(lk, timeout, [&]{ return !mQ.empty(); })) return false;
        
        out = std::move(mQ.front()); mQ.pop_front();
        return true;
    }

private:
    size_t mCap; 
    std::deque<T> mQ; 
    std::mutex mMutex; 
    std::condition_variable mCv;
};
#endif // BOUNDED_QUEUE_HPP
