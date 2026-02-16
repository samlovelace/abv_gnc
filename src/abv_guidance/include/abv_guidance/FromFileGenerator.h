#ifndef FROMFILEGENERATOR_H
#define FROMFILEGENERATOR_H
 
#include "abv_guidance/IPathGenerator.hpp"
 
class FromFileGenerator : public IPathGenerator
{ 
public:
    FromFileGenerator();
    ~FromFileGenerator() override; 

    bool init() override; 
    bool hasNext() override; 
    Waypoint getNext() override; 

private:

    std::size_t mIndex; 
    std::vector<Waypoint> mPath; 
   
};
#endif //FROMFILEGENERATOR_H    