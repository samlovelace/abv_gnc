#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <kdl/jntarray.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

// OMPL joint-space motion planner: RRTConnect over a bounded
// RealVectorStateSpace (one dimension per controlled arm joint), with a
// caller-supplied collision validity check. Smooths/simplifies the solution
// via OMPL's own PathSimplifier (through SimpleSetup::simplifySolution()).
class MotionPlanner
{
public:
    // aIsValid(jointVector) -> true if collision-free. Joint limits are
    // enforced separately by the bounded state space, not by aIsValid.
    using ValidityFn = std::function<bool(const std::vector<double>&)>;

    MotionPlanner();
    ~MotionPlanner();

    // aLower/aUpper: per-joint limits, size == number of controlled arm
    // joints.
    bool init(const std::vector<double>& aLower, const std::vector<double>& aUpper,
              double aPlanningTimeS, double aStateValidityResolutionFrac, ValidityFn aIsValid);

    // Plans RRTConnect from aStart to aGoal. Returns false (with aMessageOut
    // set) if either endpoint is itself invalid, or if no solution is found
    // within the configured time budget.
    bool plan(const KDL::JntArray& aStart, const KDL::JntArray& aGoal,
              std::vector<KDL::JntArray>& aPathOut, std::string& aMessageOut);

private:
    std::shared_ptr<ompl::base::RealVectorStateSpace> mSpace;
    std::shared_ptr<ompl::geometric::SimpleSetup> mSetup;
    double mPlanningTimeS = 2.0;
    unsigned int mDimension = 0;
};
#endif  // MOTION_PLANNER_H
