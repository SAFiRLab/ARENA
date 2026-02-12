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

#include "husky/husky_nurbs_analyzer.h"

// System
#include <vector>


namespace arena_demos
{

HuskyNurbsAnalyzer::HuskyNurbsAnalyzer(const HuskyNurbsAnalyzerConfig& a_config)
    : husky_config_(a_config),
      husky_output_(),
      arena_core::INurbsAnalyzer(a_config.base_config)
{}

void HuskyNurbsAnalyzer::eval(const Eigen::MatrixXd& a_curve_points, arena_core::EvalNurbsOutput& a_output)
{
    if (a_output.fitness_array_.empty())
        throw std::runtime_error("Fitness array is empty. Please initialize it before evaluation.");
    
    if (a_output.fitness_array_.size() != a_output.fitness_size_)
        throw std::runtime_error("Fitness array size does not match the expected fitness output size.");

    if (a_output.constraint_size_ > 0 && a_output.constraint_array_.empty())
        throw std::runtime_error("Constraint array is empty but constraint size is greater than zero. Please initialize it before evaluation.");

    if (a_output.constraint_size_ > 0 && a_output.constraint_array_.size() != a_output.constraint_size_)
        throw std::runtime_error("Constraint array size does not match the expected constraint output size.");
    
    // TODO

    husky_output_ = HuskyEvalNurbsOutput();
}



}; // arena_demos
