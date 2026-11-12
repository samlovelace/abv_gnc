#include "autobot_manip_planner/RobotCollisionModel.h"

#include <plog/Log.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <coal/mesh_loader/loader.h>
#include <coal/BVH/BVH_model.h>

namespace {
// package://<pkg>/<rest> -> <share_dir_of_pkg>/<rest>
std::string resolvePackageUriImpl(const std::string& aUri)
{
    constexpr const char* kPrefix = "package://";
    if (aUri.rfind(kPrefix, 0) != 0)
    {
        return aUri;  // already a plain filesystem path
    }

    const std::string rest = aUri.substr(std::string(kPrefix).length());
    const auto slash = rest.find('/');
    const std::string pkgName = rest.substr(0, slash);
    const std::string relPath = (slash == std::string::npos) ? "" : rest.substr(slash + 1);

    return ament_index_cpp::get_package_share_directory(pkgName) + "/" + relPath;
}
}  // namespace

RobotCollisionModel::RobotCollisionModel() = default;
RobotCollisionModel::~RobotCollisionModel() = default;

std::string RobotCollisionModel::resolvePackageUri(const std::string& aUri)
{
    return resolvePackageUriImpl(aUri);
}

bool RobotCollisionModel::init(const std::string& aUrdfPath, KinematicsHandler& aKinematics)
{
    if (!mModel.initFile(aUrdfPath))
    {
        LOGE << "RobotCollisionModel: failed to parse urdf: " << aUrdfPath;
        return false;
    }

    const KDL::JntArray probeQ(aKinematics.getNrJoints());  // zero-initialized

    mBodies.clear();
    for (const auto& [linkName, link] : mModel.links_)
    {
        if (!link || !link->collision || !link->collision->geometry)
        {
            LOGW << "RobotCollisionModel: link '" << linkName << "' has no collision geometry, skipping";
            continue;
        }

        // TOOD: handle non-mesh collision geometry (e.g. box, cylinder, sphere)
        if (link->collision->geometry->type != urdf::Geometry::MESH)
        {
            LOGW << "RobotCollisionModel: link '" << linkName
                 << "' has non-mesh collision geometry, skipping (only mesh collision is supported)";
            continue;
        }

        KDL::Frame probeFrame;
        if (!aKinematics.solveFk(probeQ, probeFrame, linkName))
        {
            // Not part of the pruned kinematics tree (e.g. gripper fingers,
            // whose joints never enter arm FK/IK/trajectory generation) -
            // nothing to track here today; gripper collision would need the
            // gripper's own joint state threaded through separately.
            LOGD << "RobotCollisionModel: link '" << linkName
                 << "' is not part of the kinematics tree, excluding from self-collision checks";
            continue;
        }

        const auto mesh = std::static_pointer_cast<urdf::Mesh>(link->collision->geometry);
        const std::string resolvedPath = resolvePackageUri(mesh->filename);

        auto geometry = loadMeshGeometry(resolvedPath, mesh->scale.x, mesh->scale.y, mesh->scale.z);
        if (!geometry)
        {
            LOGE << "RobotCollisionModel: failed to load collision mesh for link '" << linkName
                 << "': " << resolvedPath;
            continue;
        }

        LinkCollisionBody body;
        body.linkName = linkName;
        body.geometry = geometry;
        body.worldPose = coal::Transform3s::Identity();
        mBodies.push_back(std::move(body));
    }

    mAdjacentPairs.clear();
    for (const auto& [jointName, joint] : mModel.joints_)
    {
        if (!joint) continue;
        auto a = joint->parent_link_name;
        auto b = joint->child_link_name;
        if (a > b) std::swap(a, b);
        mAdjacentPairs.emplace(std::move(a), std::move(b));
    }

    LOGD << "RobotCollisionModel initialized with " << mBodies.size() << " collision bodies";
    return !mBodies.empty();
}

bool RobotCollisionModel::updatePoses(const KDL::JntArray& aJointConfig, KinematicsHandler& aKinematics)
{
    for (auto& body : mBodies)
    {
        KDL::Frame frame;
        if (!aKinematics.solveFk(aJointConfig, frame, body.linkName))
        {
            // mBodies was pre-filtered in init() to links solvable in this
            // tree, so this should not normally happen - treat as fatal if
            // it does (a real FK problem, not the expected pruned-link case).
            LOGE << "RobotCollisionModel: FK failed for link '" << body.linkName << "'";
            return false;
        }

        coal::Matrix3s rotation;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                rotation(r, c) = frame.M(r, c);
            }
        }
        const coal::Vec3s translation(frame.p.x(), frame.p.y(), frame.p.z());
        body.worldPose = coal::Transform3s(rotation, translation);
    }

    return true;
}

bool RobotCollisionModel::isAdjacentPair(const std::string& aLinkA, const std::string& aLinkB) const
{
    std::string a = aLinkA;
    std::string b = aLinkB;
    if (a > b) std::swap(a, b);
    return mAdjacentPairs.count({a, b}) > 0;
}

std::shared_ptr<coal::CollisionGeometry> RobotCollisionModel::loadMeshGeometry(
    const std::string& aResolvedStlPath, double aScaleX, double aScaleY, double aScaleZ) const
{
    // coal ships an assimp-backed mesh loader (coal/mesh_loader/loader.h) that
    // produces a BVHModel-based CollisionGeometry from common mesh formats,
    // including the STL collision meshes used by every link in this URDF.
    // NOTE: exact class/method names here are based on hpp-fcl/coal's
    // well-established MeshLoader convention - verify against the actual
    // installed coal headers (coal wasn't installed on the machine this plan
    // was written on) and adjust if the real API differs.
    coal::MeshLoader loader;
    const coal::Vec3s scale(aScaleX, aScaleY, aScaleZ);
    const coal::BVHModelPtr_t bvh = loader.load(aResolvedStlPath, scale);
    return std::static_pointer_cast<coal::CollisionGeometry>(bvh);
}
