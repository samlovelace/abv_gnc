#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <memory>
#include <string>
#include <vector>

#include <coal/collision_object.h>

#include "autobot_manip_planner/RobotCollisionModel.h"

// Extension point for future non-robot obstacles (perceived objects, the
// table). Unused today - the scene-object list stays empty - but
// CollisionChecker's API already accommodates it so the self-collision-only
// call sites won't need to change shape when scene checking is added later.
struct SceneObject
{
    std::string name;
    std::shared_ptr<coal::CollisionGeometry> geometry;
    coal::Transform3s worldPose;
};

class CollisionChecker
{
public:
    CollisionChecker() = default;

    // Self-collision only: pairwise coal collision queries between every
    // pair of aModel's bodies, skipping link pairs that share a joint.
    // Early-out on first hit (adequate for a boolean validity check, not
    // exhaustive). If aWhyOut is non-null, it's set to "<linkA> x <linkB>"
    // on collision.
    bool isSelfColliding(const RobotCollisionModel& aModel, std::string* aWhyOut = nullptr) const;

    // Scene objects - empty today. Kept as a separate method (rather than
    // folded into isSelfColliding) so today's self-collision-only call
    // sites stay obviously self-collision-only.
    void setSceneObjects(std::vector<SceneObject> aObjects) { mSceneObjects = std::move(aObjects); }
    bool isSceneColliding(const RobotCollisionModel& aModel, std::string* aWhyOut = nullptr) const;

private:
    bool pairCollides(const coal::CollisionGeometry* aGeomA, const coal::Transform3s& aPoseA,
                       const coal::CollisionGeometry* aGeomB, const coal::Transform3s& aPoseB) const;

    std::vector<SceneObject> mSceneObjects;  // empty today - future scene-collision extension point
};
#endif  // COLLISION_CHECKER_H
