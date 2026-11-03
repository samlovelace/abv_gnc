#ifndef THREADSAFE_HPP
#define THREADSAFE_HPP

#include <mutex>

template<typename T>
class ThreadSafe
{
public:
    ThreadSafe() = default;
    ~ThreadSafe() = default;

    void set(const T& aData)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mData = aData;
    }

    T get() const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return mData;
    }

private:
    T mData;
    mutable std::mutex mMutex;
};

#endif // THREADSAFE_HPP
