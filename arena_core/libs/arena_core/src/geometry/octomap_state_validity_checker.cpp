#include "linedrone_navigation/geometry/octomap_state_validity_checker.h"

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>

// FCL


namespace linedrone_navigation
{

OctomapStateValidityChecker::OctomapStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                                                         octomap::OcTree* tree)
: ompl::base::StateValidityChecker(si),
tree_(new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree))),
robot_geometry_(new fcl::Box(0.25, 0.25, 0.25)),
robot_object_(new fcl::CollisionObject(robot_geometry_)),
tree_object_(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(tree_)))
{};

bool OctomapStateValidityChecker::isValid(const ompl::base::State *state) const
{
    // cast the abstract state type to the type we expect
	const ompl::base::SE3StateSpace::StateType *se3state = state->as<ompl::base::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
	const ompl::base::RealVectorStateSpace::StateType *pos = se3state->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
	const ompl::base::SO3StateSpace::StateType *rot = se3state->as<ompl::base::SO3StateSpace::StateType>(1);

    // check validity of state Fdefined by pos & rot
	fcl::Vec3f translation(pos->values[0] + 1.3, pos->values[1] + 1.3, pos->values[2] + 0.85);
	fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
    robot_object_->setTransform(rotation, translation);
	fcl::CollisionRequest requestType(1,false,1,false);
	fcl::CollisionResult collisionResult;
	fcl::collide(robot_object_, tree_object_, requestType, collisionResult);

	return(!collisionResult.isCollision());
}

double OctomapStateValidityChecker::clearance(const ompl::base::State *state) const
{
    return 0.0;
}

}; // namespace linedrone_navigation
