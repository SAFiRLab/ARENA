#include "arena_core/geometry/octomap_state_validity_checker.h"

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>

// FCL


namespace arena_core
{

OctomapStateValidityChecker::OctomapStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
														 std::shared_ptr<octomap::OcTree> tree)
: ompl::base::StateValidityChecker(si), tree_(std::make_shared<fcl::OcTreed>(tree)),
tree_object_(std::make_shared<fcl::CollisionObjectd>(tree_)),
robot_geometry_(std::make_shared<fcl::Boxd>(0.25, 0.25, 0.25)),
robot_object_(std::make_shared<fcl::CollisionObjectd>(robot_geometry_))
{}

bool OctomapStateValidityChecker::isValid(const ompl::base::State *state) const
{
    const auto *se3state = state->as<ompl::base::SE3StateSpace::StateType>();

    const auto *pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const auto *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

    fcl::Vector3<double> translation(pos->values[0] + 1.3, pos->values[1] + 1.3, pos->values[2] + 0.85);
    fcl::Quaternion<double> rotation(rot->w, rot->x, rot->y, rot->z);

    robot_object_->setTranslation(translation);
    robot_object_->setRotation(rotation.toRotationMatrix());
    robot_object_->computeAABB();

    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;

    fcl::collide(robot_object_.get(), tree_object_.get(), request, result);

    return !result.isCollision();
}

double OctomapStateValidityChecker::clearance(const ompl::base::State *state) const
{
    return 0.0;
}

}; // namespace arena_core
