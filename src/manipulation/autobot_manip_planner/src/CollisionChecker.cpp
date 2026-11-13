#include "autobot_manip_planner/CollisionChecker.h"

#include <coal/collision.h>
#include <coal/collision_data.h>

bool CollisionChecker::pairCollides(const coal::CollisionGeometry* aGeomA, const coal::Transform3s& aPoseA,
                                     const coal::CollisionGeometry* aGeomB, const coal::Transform3s& aPoseB) const
{
    // Standard hpp-fcl/coal free-function collision query: two geometry+pose
    // pairs, a request, and a result struct it populates. NOTE: verify this
    // exact signature against the installed coal headers (coal wasn't
    // installed on the machine this was written on) - adjust if coal 3.0.3
    // differs (e.g. if it only exposes a CollisionObject-based overload).
    coal::CollisionRequest request;
    coal::CollisionResult result;
    coal::collide(aGeomA, aPoseA, aGeomB, aPoseB, request, result);
    return result.isCollision();
}

bool CollisionChecker::isSelfColliding(const RobotCollisionModel& aModel, std::string* aWhyOut) const
{
    const auto& bodies = aModel.getBodies();
    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        for (std::size_t j = i + 1; j < bodies.size(); ++j)
        {
            if (aModel.isAdjacentPair(bodies[i].linkName, bodies[j].linkName)) continue;

            if (pairCollides(bodies[i].geometry.get(), bodies[i].worldPose,
                              bodies[j].geometry.get(), bodies[j].worldPose))
            {
                if (aWhyOut)
                {
                    *aWhyOut = bodies[i].linkName + " x " + bodies[j].linkName;
                }
                return true;
            }
        }
    }
    return false;
}

bool CollisionChecker::isSceneColliding(const RobotCollisionModel& aModel, std::string* aWhyOut) const
{
    const auto& bodies = aModel.getBodies();
    for (const auto& body : bodies)
    {
        for (const auto& scene : mSceneObjects)
        {
            if (pairCollides(body.geometry.get(), body.worldPose, scene.geometry.get(), scene.worldPose))
            {
                if (aWhyOut)
                {
                    *aWhyOut = body.linkName + " x " + scene.name;
                }
                return true;
            }
        }
    }
    return false;
}
