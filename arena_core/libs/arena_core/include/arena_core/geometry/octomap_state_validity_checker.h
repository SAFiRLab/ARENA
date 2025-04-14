#pragma once

// OMPL
#include <ompl/base/StateValidityChecker.h>

// FCL
#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision.h"

// Octomap
#include <octomap/OcTree.h>
#include <octomap/octomap.h>


namespace arena_core
{

class OctomapStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    // Constructor
    OctomapStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                                octomap::OcTree* tree);

    bool isValid(const ompl::base::State *state) const override;
    double clearance(const ompl::base::State *state) const override;

private:

    fcl::OcTree* tree_;
    fcl::CollisionObject* tree_object_;
    std::shared_ptr<fcl::CollisionGeometry> robot_geometry_;
    fcl::CollisionObject* robot_object_;
    
}; // class OctomapStateValidityChecker


}; // namespace arena_core
