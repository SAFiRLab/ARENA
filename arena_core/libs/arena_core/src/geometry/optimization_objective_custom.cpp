#include "arena_core/geometry/optimization_objective_custom.h"

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>


namespace arena_core
{

OptimizationObjectiveCustom::OptimizationObjectiveCustom(const ompl::base::SpaceInformationPtr& si, octomap::ColorOcTree* octree)
: ompl::base::OptimizationObjective(si), octree_(octree)
{};

ompl::base::Cost OptimizationObjectiveCustom::stateCost(const ompl::base::State* s) const
{
    //const ompl::base::SE3StateSpace::StateType* c_state_3D = s->as<ompl::base::SE3StateSpace::StateType>();
    return ompl::base::Cost(1.0);
}

ompl::base::Cost OptimizationObjectiveCustom::motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const
{
    return ompl::base::Cost(stateCost(s1).value() + stateCost(s2).value());
}


// @brief: This method returns a vector that represents the obstacle field in the configuration space
//         The obstacle field is a vector that points away from the obstacles and is scaled according
//         to the distance to the obstacle
// @param: base_state: The state in the configuration space
// @return: The obstacle field vector
Eigen::VectorXd OptimizationObjectiveCustom::obstacleFieldConfigSpace(const ompl::base::State* base_state)
{
    const ompl::base::SE3StateSpace::StateType* state = base_state->as<ompl::base::SE3StateSpace::StateType>();

    Eigen::Vector3d position(state->getX(), state->getY(), state->getZ());

    // Define a vector for the obstacle field
    Eigen::Vector3d obstacleField(0.0, 0.0, 0.0);

    // Iterate over the octree nodes to check for obstacles
    double max_distance = 2.0; // Adjust this radius according to your needs
    double resolution = octree_->getResolution();

    for (octomap::ColorOcTree::leaf_iterator it = octree_->begin_leafs(),
         end = octree_->end_leafs(); it != end; ++it)
    {
        // Check if the node is occupied
        if (octree_->isNodeOccupied(*it))
        {
            octomap::point3d obstacle_position = it.getCoordinate();
            Eigen::Vector3d toObstacle = Eigen::Vector3d(obstacle_position.x(),
                                                        obstacle_position.y(),
                                                        obstacle_position.z()) - position;
            double distance = toObstacle.norm();

            if (distance < max_distance)
            {
                // You are too close to the obstacle, generate a repulsive force
                double scale = 1.0 - (distance / max_distance);
                obstacleField -= (toObstacle.normalized() * scale);
            }
        }
    }

    return obstacleField;
}

}; // namespace arena_core
