
#include "abv_guidance/StraightLineGenerator.h"

StraightLineGenerator::StraightLineGenerator(const Waypoint& aGoal, 
        const Eigen::Vector3d& aCurrent) : mGoal(aGoal), mStartPose(aCurrent)
{

}

StraightLineGenerator::~StraightLineGenerator()
{

}

bool StraightLineGenerator::init()
{
    mHasNext = true;
    return true;  
}

bool StraightLineGenerator::hasNext()
{
    return mHasNext; 
}

Waypoint StraightLineGenerator::getNext()
{
    if(mHasNext)
    {
        mHasNext = false; 
        return mGoal; 
    }
}