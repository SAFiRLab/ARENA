#pragma once

// OMPL
#include <ompl/base/StateValidityChecker.h>

// FCL
#include <fcl/fcl.h>

// Octomap
#include <octomap/OcTree.h>
#include <octomap/octomap.h>


namespace arena_core
{

class OctomapStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    // Constructor
    OctomapStateValidityChecker(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<octomap::OcTree> tree);

    bool isValid(const ompl::base::State *state) const override;
    double clearance(const ompl::base::State *state) const override;

private:

    std::shared_ptr<fcl::OcTreed> tree_;
    std::shared_ptr<fcl::CollisionGeometryd> robot_geometry_;
    std::shared_ptr<fcl::CollisionObjectd> robot_object_;
    std::shared_ptr<fcl::CollisionObjectd> tree_object_;
    
}; // class OctomapStateValidityChecker


}; // namespace arena_core
