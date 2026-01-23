#ifndef STRAIGHTLINEGENERATOR_H
#define STRAIGHTLINEGENERATOR_H
 
#include "abv_guidance/IPathGenerator.hpp" 

class StraightLineGenerator : public IPathGenerator
{ 
public:
    StraightLineGenerator();
    ~StraightLineGenerator() override; 

    bool init() override; 
    bool hasNext() override; 
    Waypoint getNext() override; 

private:
   
};
#endif //STRAIGHTLINEGENERATOR_H