#include "autobot_manip_planner/MotionPlanner.h"

#include <algorithm>
#include <plog/Log.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

MotionPlanner::MotionPlanner() = default;
MotionPlanner::~MotionPlanner() = default;

bool MotionPlanner::init(const std::vector<double>& aLower, const std::vector<double>& aUpper,
                          double aPlanningTimeS, double aStateValidityResolutionFrac, ValidityFn aIsValid)
{
    if (aLower.size() != aUpper.size() || aLower.empty())
    {
        LOGE << "MotionPlanner::init: joint limit size mismatch or empty";
        return false;
    }

    mDimension = static_cast<unsigned int>(aLower.size());
    mPlanningTimeS = aPlanningTimeS;

    mSpace = std::make_shared<ompl::base::RealVectorStateSpace>(mDimension);

    ompl::base::RealVectorBounds bounds(mDimension);
    for (unsigned int i = 0; i < mDimension; ++i)
    {
        bounds.setLow(i, aLower[i]);
        bounds.setHigh(i, aUpper[i]);
    }
    mSpace->setBounds(bounds);
    mSpace->setLongestValidSegmentFraction(aStateValidityResolutionFrac);
    mSpace->setup();

    mSetup = std::make_shared<ompl::geometric::SimpleSetup>(mSpace);

    const unsigned int dim = mDimension;
    mSetup->setStateValidityChecker([aIsValid, dim](const ompl::base::State* aState) {
        const auto* rv = aState->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> q(rv->values, rv->values + dim);
        return aIsValid(q);
    });

    mSetup->setPlanner(std::make_shared<ompl::geometric::RRTConnect>(mSetup->getSpaceInformation()));
    return true;
}

bool MotionPlanner::plan(const KDL::JntArray& aStart, const KDL::JntArray& aGoal,
                          std::vector<KDL::JntArray>& aPathOut, std::string& aMessageOut)
{
    if (aStart.rows() != mDimension || aGoal.rows() != mDimension)
    {
        aMessageOut = "MotionPlanner::plan: joint array size does not match planner dimension";
        return false;
    }

    mSetup->clear();  // OMPL planners accumulate tree state across calls otherwise

    // The measured current configuration can land a hair outside its exact
    // URDF joint limit from ordinary sensor/settling noise (seen in practice:
    // ~0.0004 rad past a 0.0 limit) - OMPL's bounded state space rejects such
    // a state outright as "invalid bounds" even though it's physically at the
    // limit. Clamp both endpoints into bounds rather than let that hard-fail
    // planning; collision validity is still checked separately via the
    // validity checker, this only prevents a false bounds rejection.
    const ompl::base::RealVectorBounds& bounds =
        mSpace->as<ompl::base::RealVectorStateSpace>()->getBounds();

    ompl::base::ScopedState<> startState(mSpace);
    ompl::base::ScopedState<> goalState(mSpace);
    for (unsigned int i = 0; i < mDimension; ++i)
    {
        const double clampedStart = std::clamp(aStart(i), bounds.low[i], bounds.high[i]);
        const double clampedGoal = std::clamp(aGoal(i), bounds.low[i], bounds.high[i]);
        startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = clampedStart;
        goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = clampedGoal;
    }

    if (!mSetup->getStateValidityChecker()->isValid(startState.get()))
    {
        aMessageOut = "start configuration is not collision-free";
        return false;
    }
    if (!mSetup->getStateValidityChecker()->isValid(goalState.get()))
    {
        aMessageOut = "goal configuration is not collision-free";
        return false;
    }

    mSetup->setStartAndGoalStates(startState, goalState);

    const ompl::base::PlannerStatus status = mSetup->solve(mPlanningTimeS);
    if (!status)
    {
        aMessageOut = "no collision-free path found within " + std::to_string(mPlanningTimeS) + "s";
        return false;
    }

    mSetup->simplifySolution();
    // Deliberately not interpolated - autobot_manip_executor dispatches each
    // joint-space waypoint and waits for arrival before sending the next, so
    // a densified path would mean dozens of small stop-start motions instead
    // of a few smooth controller-generated (Ruckig) segments.
    const auto& path = mSetup->getSolutionPath();
    aPathOut.clear();
    aPathOut.reserve(path.getStateCount());
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto* rv = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        KDL::JntArray q(mDimension);
        for (unsigned int j = 0; j < mDimension; ++j)
        {
            q(j) = rv->values[j];
        }
        aPathOut.push_back(q);
    }

    return true;
}
