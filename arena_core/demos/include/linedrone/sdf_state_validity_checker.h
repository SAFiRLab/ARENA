/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, David-Alexandre Poissant, Université de Sherbrooke
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

// Local
#include "linedrone/costmap_mapping.h"

// External Libraries
// OMPL
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// System
#include <memory>
#include <stdexcept>


namespace arena_demos
{

/**
 * @brief State validity checker of the RRT initialization of ARENA using the signed distance field (SDF octomap) on top of
 * the inflated octomap: a state is valid if it is clear of the inflated octomap (same check as
 * arena_core::OMPLStateValidityChecker) and if its SDF collision cost is at most a_max_cost. With a_max_cost the highest
 * cost of the start and the goal, the initial paths stay at least as far from the obstacles as the start and the goal (out
 * of the SDF when both are out of it).
 */
class SdfStateValidityChecker : public ompl::base::StateValidityChecker
{
public:

    SdfStateValidityChecker(const ompl::base::SpaceInformationPtr& a_si, std::shared_ptr<CostmapMapping> a_mapping,
                            double a_max_cost, double a_clearance = 0.8)
    : ompl::base::StateValidityChecker(a_si), mapping_(std::move(a_mapping)), max_cost_(a_max_cost), clearance_(a_clearance)
    {
        sdf_ = mapping_->getCapability<CostSDFCapability>();
        if (!sdf_)
            throw std::runtime_error("SdfStateValidityChecker: the SDF octomap is not available.");
    }

    bool isValid(const ompl::base::State* a_state) const override
    {
        const auto* state = a_state->as<ompl::base::RealVectorStateSpace::StateType>();
        const Eigen::Vector3d point(state->values[0], state->values[1], state->values[2]);
        return mapping_->isClearance(point, clearance_) && sdf_->getCollisionCost(point) <= max_cost_;
    }

    double getMaxCost() const { return max_cost_; }

private:
    std::shared_ptr<CostmapMapping> mapping_;
    std::shared_ptr<CostSDFCapability> sdf_;
    double max_cost_;
    double clearance_;

}; // class SdfStateValidityChecker

}; // namespace arena_demos
