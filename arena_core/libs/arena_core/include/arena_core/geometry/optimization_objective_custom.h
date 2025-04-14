#pragma once


// Octomap
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

// OMPL
#include <ompl/base/OptimizationObjective.h>


namespace arena_core
{

class OptimizationObjectiveCustom : public ompl::base::OptimizationObjective
{
public:

    OptimizationObjectiveCustom(const ompl::base::SpaceInformationPtr& si, octomap::ColorOcTree* octree);

    ompl::base::Cost stateCost(const ompl::base::State* s) const override;
    ompl::base::Cost motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const override;

    Eigen::VectorXd obstacleFieldConfigSpace(const ompl::base::State* base_state);

private:

    octomap::ColorOcTree* octree_;

}; // class OptimizationObjectiveCustom

}; // namespace arena_core
