#ifndef ICOMMANDSOURCE_HPP
#define ICOMMANDSOURCE_HPP
 
#include <memory> 
#include "abv_guidance/ICommandSink.hpp" 

class ICommandSource
{
public:
    virtual ~ICommandSource() = default;

    virtual void listen() = 0;
    virtual void stop() = 0;

protected:
    explicit ICommandSource(ICommandSink& aSink)
        : mCommandSink(aSink)
    {}

    ICommandSink& mCommandSink;
};

#endif //ICOMMANDSOURCE_HPP 