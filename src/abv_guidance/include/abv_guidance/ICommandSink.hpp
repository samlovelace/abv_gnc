#ifndef ICOMMANDSINK_HPP
#define ICOMMANDSINK_HPP

#include "abv_guidance/InternalTypes.hpp"
 
class ICommandSink 
{ 
public:
    virtual ~ICommandSink() = default; 
    virtual void onCommand(const Command& aCommand) = 0; 

private:
   
};
#endif //ICOMMANDSINK_HPP