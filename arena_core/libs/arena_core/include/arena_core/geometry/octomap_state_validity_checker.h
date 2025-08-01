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
