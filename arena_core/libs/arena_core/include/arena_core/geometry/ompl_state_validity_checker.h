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

#pragma once

// Local
#include "arena_core/mapping/IMapping.h"

// External Libraries
// OMPL
#include <ompl/base/StateValidityChecker.h>


namespace arena_core
{

class OMPLStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    OMPLStateValidityChecker(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<IMapping> mapping, double step = 0.8);

    /**
     * \brief Check if the state is valid.
     * 
     * This method checks if the state is valid by verifying if the point represented by the state
     * is clear of obstacles within a specified clearance distance.
     * 
     * @param state The state to check for validity.
     * @return true if the state is valid, false otherwise.
     */
    bool isValid(const ompl::base::State *state) const override;

    /**
     * \brief Report the distance to the nearest invalid state.
     * 
     * This method reports the distance to the nearest invalid state when starting from the given state.
     * If the distance is negative, it indicates the penetration depth.
     * 
     * @param state The state to check for clearance.
     * @return The distance to the nearest invalid state.
     */
    double clearance(const ompl::base::State *state) const override;

private:

    std::shared_ptr<IMapping> mapping_; // Pointer to the mapping instance used for collision checking
    double step_; // Step size for bounding box around the state point
    
}; // class OMPLStateValidityChecker


}; // namespace arena_core
