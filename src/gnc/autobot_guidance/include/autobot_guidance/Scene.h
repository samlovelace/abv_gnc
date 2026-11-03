#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include "autobot_msgs/msg/obstacle_array.hpp"

#include "autobot_common/ThreadSafe.hpp"

#include "autobot_guidance/AxisAlignedBoundingBox.hpp"
#include "autobot_guidance/SceneTypes.hpp"

// Holds the 2D scene the ABV operates in: the fixed tabletop bounds (from
// SceneConfig) and the live set of obstacles reported on abv/scene/obstacles
// (e.g. from autobot_gui's click-to-place stub). Intended to be queried by a
// future CollisionAvoidPathGenerator when evaluating candidate poses/paths.
class Scene
{
public:
    Scene();
    ~Scene();

    // the fixed tabletop bounds - the region isWithinBounds() checks against
    const AxisAlignedBoundingBox& getBounds() const { return mBounds; }

    // true iff aBox lies fully within the configured scene limits
    bool isWithinBounds(const AxisAlignedBoundingBox& aBox) const;

    // true iff aBox does not overlap any currently known obstacle
    bool isCollisionFree(const AxisAlignedBoundingBox& aBox) const;

    std::vector<Obstacle> getObstacles() const;

private:
    void obstaclesCallback(autobot_msgs::msg::ObstacleArray::SharedPtr aMsg);

    AxisAlignedBoundingBox mBounds;
    ThreadSafe<std::vector<Obstacle>> mObstacles;
};
#endif // SCENE_H
