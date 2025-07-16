#pragma once

// OMPL
#include <ompl/base/StateValidityChecker.h>

// Octomap
#include <octomap/OcTree.h>
#include <octomap/octomap.h>


namespace arena_core
{

class OctomapStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    OctomapStateValidityChecker(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<octomap::OcTree> tree, double step = 0.8);

    bool isValid(const ompl::base::State *state) const override;
    double clearance(const ompl::base::State *state) const override;

    /************* Setters *************/
    /**
    * @brief Set the step size for the bounding box around the state point.
    * @param step The step size to set.
    */
    void setStep(double step)
    {
        if (step <= 0.0)
            throw std::invalid_argument("Step size must be positive.");
        step_ = step;
    };


private:

    std::shared_ptr<octomap::OcTree> octree_;
    double step_; // Step size for bounding box around the state point
    
}; // class OctomapStateValidityChecker


}; // namespace arena_core
