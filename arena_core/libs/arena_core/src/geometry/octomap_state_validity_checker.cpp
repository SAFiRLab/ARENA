#include "arena_core/geometry/octomap_state_validity_checker.h"

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>

// FCL


namespace arena_core
{

OctomapStateValidityChecker::OctomapStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
														 std::shared_ptr<octomap::OcTree> tree, double step)
: ompl::base::StateValidityChecker(si), octree_(std::move(tree)), step_(step)
{
    if (!octree_)
        throw std::runtime_error("arena_core::OctomapStateValidityChecker(): Octree is null.");

    if (step_ <= 0.0)
        throw std::invalid_argument("arena_core::OctomapStateValidityChecker(): Step size must be positive.");
}

bool OctomapStateValidityChecker::isValid(const ompl::base::State *state) const
{
    // Cast the abstract state type to the type we expect
    // Cast to RealVectorStateSpace
    const auto *real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    if (!real_state)
        throw std::runtime_error("arena_core::OctomapStateValidityChecker::isValid(): State is not of type RealVectorStateSpace.");

    // The rest of this isValid check verifies for 3D collision using the octree and expects the state values to be [x, y, z, ...]
    const double x = real_state->values[0];
    const double y = real_state->values[1];
    const double z = real_state->values[2];

    // Define the bounding box around the robot's center position
    const octomap::point3d center(x, y, z);
    const octomap::point3d bbx_min(center.x() - step_, center.y() - step_, center.z() - step_);
    const octomap::point3d bbx_max(center.x() + step_, center.y() + step_, center.z() + step_);

    // Iterate over all occupied voxels in the bounding box
    for (auto it = octree_->begin_leafs_bbx(bbx_min, bbx_max); it != octree_->end_leafs_bbx(); ++it)
    {
        if (octree_->isNodeOccupied(*it))
            return false;
    }

    return true;
}


double OctomapStateValidityChecker::clearance(const ompl::base::State *state) const
{
    return 0.0;
}

}; // namespace arena_core
