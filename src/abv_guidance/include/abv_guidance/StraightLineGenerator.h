#ifndef STRAIGHTLINEGENERATOR_H
#define STRAIGHTLINEGENERATOR_H
 
#include "abv_guidance/IPathGenerator.hpp" 
#include <eigen3/Eigen/Dense>

class StraightLineGenerator : public IPathGenerator
{ 
public:
    StraightLineGenerator(const Waypoint& aGoal, const Eigen::Vector3d& aCurrent);
    ~StraightLineGenerator() override; 

    bool init() override; 
    bool hasNext() override; 
    Waypoint getNext() override; 

private:

    Waypoint mGoal;
    Eigen::Vector3d mStartPose; 

    bool mHasNext; 
   
};
#endif //STRAIGHTLINEGENERATOR_H