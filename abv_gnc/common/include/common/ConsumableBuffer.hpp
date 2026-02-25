#ifndef CONSUMABLEBUFFER_HPP
#define CONSUMABLEBUFFER_HPP

#include <mutex>
#include <optional>

template<typename BufferT> 
class ConsumableBuffer
{
public:
    ConsumableBuffer() = default; 
    ~ConsumableBuffer() = default; 

    ConsumableBuffer(const ConsumableBuffer&) = delete;
    ConsumableBuffer& operator=(const ConsumableBuffer&) = delete;

    ConsumableBuffer(ConsumableBuffer&&) = default;
    ConsumableBuffer& operator=(ConsumableBuffer&&) = default; 

    void put(const BufferT& aData)
    {
        std::lock_guard<std::mutex> lock(mMutex); 
        mBuffer = aData; 
    } 

    std::optional<BufferT> consume()
    {
        std::optional<BufferT> tmp = std::nullopt; 
        std::lock_guard<std::mutex> lock(mMutex); 

        if(mBuffer.has_value())
        {
            tmp = mBuffer;
            mBuffer = std::nullopt;   
        }

        return tmp; 
    } 

private: 
    
    std::mutex mMutex; 
    std::optional<BufferT> mBuffer; 

};
#endif