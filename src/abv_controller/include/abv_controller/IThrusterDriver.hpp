#ifndef ITHRUSTERDRIVER_HPP
#define ITHRUSTERDRIVER_HPP
 
#include <string> 
#include <yaml-cpp/yaml.h> // to avoid including in all implementations 

class IThrusterDriver 
{ 
public:
    virtual ~IThrusterDriver() = default; 

    virtual bool init() = 0;  
    virtual bool fini() = 0; 

    virtual bool send(const std::string& aThrusterCommand) = 0; 

private:
   
};
#endif //ITHRUSTERDRIVER_HPP