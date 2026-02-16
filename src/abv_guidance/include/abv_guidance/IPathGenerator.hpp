#ifndef IPATHGENERATOR_HPP
#define IPATHGENERATOR_HPP
 
#include "abv_guidance/InternalTypes.hpp"
 
class IPathGenerator 
{ 
public:

    virtual ~IPathGenerator() = default; 

    virtual bool init() = 0; 
    virtual bool hasNext() = 0; 
    virtual Waypoint getNext() = 0; 

private:
   
};
#endif //IPATHGENERATOR_HPP