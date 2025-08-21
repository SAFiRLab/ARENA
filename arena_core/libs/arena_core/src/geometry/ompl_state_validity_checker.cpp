/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2025, David-Alexandre Poissant, Université de Sherbrooke
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

#include "arena_core/geometry/ompl_state_validity_checker.h"

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>


namespace arena_core
{

OMPLStateValidityChecker::OMPLStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                                                         std::shared_ptr<IMapping> mapping, double step)
: ompl::base::StateValidityChecker(si), mapping_(std::move(mapping)), step_(step)
{
    if (!mapping_)
        throw std::runtime_error("arena_core::OMPLStateValidityChecker(): Mapping isntance is null.");
}

bool OMPLStateValidityChecker::isValid(const ompl::base::State *state) const
{
    // Cast the abstract state type to the type we expect
    // Cast to RealVectorStateSpace
    const auto *real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    if (!real_state)
        throw std::runtime_error("arena_core::OMPLStateValidityChecker::isValid(): State is not of type RealVectorStateSpace.");

    Eigen::Vector3d point(real_state->values[0], real_state->values[1], real_state->values[2]);
    return mapping_->isClearance(point, step_);

    return true;
}


double OMPLStateValidityChecker::clearance(const ompl::base::State *state) const
{
    return 0.0;
}

}; // namespace arena_core
