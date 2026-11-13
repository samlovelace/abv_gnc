#ifndef ROBOT_COLLISION_MODEL_H
#define ROBOT_COLLISION_MODEL_H

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <kdl/jntarray.hpp>
#include <urdf/model.h>
#include <coal/collision_object.h>

#include "autobot_manip_common/KinematicsHandler.h"

// One robot link's collision geometry + its current world-frame pose.
struct LinkCollisionBody
{
    std::string linkName;
    std::shared_ptr<coal::CollisionGeometry> geometry;  // loaded once at init, reused every call
    coal::Transform3s worldPose;                        // recomputed per joint configuration
};

// Parses <collision><geometry><mesh> entries for every URDF link (independently
// of KinematicsHandler's own, unimplemented collision stubs - this class re-parses
// the same urdf file cheaply at init rather than extending a class already shared
// in production by autobot_manip_controller) and, given a joint configuration,
// computes each link's current collision geometry + world-frame pose via
// KinematicsHandler::solveFk. Owns no collision-checking logic itself -
// CollisionChecker consumes its output.
class RobotCollisionModel
{
public:
    RobotCollisionModel();
    ~RobotCollisionModel();

    // Parses aUrdfPath's <collision><geometry><mesh> entries (one mesh per
    // link, per the abv_rebot urdf convention). Resolves package:// mesh URIs
    // via ament_index_cpp::get_package_share_directory. Links with zero
    // collision meshes are skipped (logged, not fatal). aKinematics is used
    // to probe (once, here) which links are actually solvable via FK in its
    // kinematic tree - links KinematicsHandler pruned out (gripper fingers,
    // excluded from arm FK/IK since their joints never enter trajectory
    // generation) are dropped from the tracked body list rather than failing
    // every updatePoses() call.
    bool init(const std::string& aUrdfPath, KinematicsHandler& aKinematics);

    // Recomputes mBodies[i].worldPose for every link via
    // aKinematics.solveFk(aJointConfig, frame, linkName). Returns false if
    // any FK solve fails.
    bool updatePoses(const KDL::JntArray& aJointConfig, KinematicsHandler& aKinematics);

    const std::vector<LinkCollisionBody>& getBodies() const { return mBodies; }

    // Unordered link-name pairs that share a joint - excluded from
    // self-collision checks since adjacent links are expected to touch/
    // overlap near their shared joint axis at many valid configurations.
    bool isAdjacentPair(const std::string& aLinkA, const std::string& aLinkB) const;

private:
    std::shared_ptr<coal::CollisionGeometry> loadMeshGeometry(const std::string& aResolvedStlPath,
                                                                double aScaleX, double aScaleY, double aScaleZ) const;
    static std::string resolvePackageUri(const std::string& aUri);

    urdf::Model mModel;
    std::vector<LinkCollisionBody> mBodies;
    std::set<std::pair<std::string, std::string>> mAdjacentPairs;
};
#endif  // ROBOT_COLLISION_MODEL_H
