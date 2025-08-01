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

// System
#include <vector>

// External Libraries
#include <Eigen/Dense>


namespace arena_core
{

/**
 * @brief Output structure for NURBS evaluation.
 * This structure holds the output size and a pointer to the output array. Note that we
 * use raw pointers 
 */
struct EvalNurbsOutput
{
    int fitness_size_; // Size of the fitness output
    std::vector<double> fitness_array_;

    int constraint_size_; // Size of the constraint output
    std::vector<double> constraint_array_;

    EvalNurbsOutput() : fitness_size_(0), fitness_array_(std::vector<double>(fitness_size_)),
                        constraint_size_(0), constraint_array_(std::vector<double>(constraint_size_))
    {}
}; // struct EvalNurbsOutput


/**
 * @brief Configuration structure for NURBS analyzers.
 * This structure holds the basic configuration parameters for all NURBS analyzers.
 */
struct NurbsAnalyzerConfig
{
    unsigned int sample_size; // Number of samples for NURBS evaluation

    NurbsAnalyzerConfig(unsigned int a_sample_size) : sample_size(a_sample_size) {}
}; // NurbsAnalyzerConfig


/**
 * @brief Interface for NURBS analysis.
 * This interface defines the methods required for evaluating NURBS curves.
 */
class INurbsAnalyzer
{
public:

    /**
     * @brief Default constructor for INurbsAnalyzer.
     * This constructor is virtual to allow derived classes to implement their own constructors.
     */
    INurbsAnalyzer(NurbsAnalyzerConfig a_config)
        : config_(a_config) {}

    /**
     * @brief Virtual destructor for INurbsAnalyzer.
     * Ensures proper cleanup of derived classes.
     */
    virtual ~INurbsAnalyzer() = default;

    /**
     * @brief Evaluate the NURBS curve.
     * This method should be implemented to perform the evaluation of the NURBS curve
     * and populate the output structure with the results.
     *
     * @param a_output Pointer to an EvalNurbsOutput structure where results will be stored.
     * @param a_curve_points Matrix of control points for the NURBS curve.
     */
    virtual void eval(const Eigen::MatrixXd& a_curve_points, EvalNurbsOutput& a_output) = 0;

    /**
     * @brief Get the sample size for the NURBS evaluation.
     * This method should return the number of samples used in the evaluation.
     *
     * @return The sample size as an unsigned integer.
     */
    unsigned int getSampleSize() const { return config_.sample_size; };

protected:

    NurbsAnalyzerConfig config_; // Basic Cconfiguration for all NURBS analyzers

}; // class INurbsAnalyzer

}; // namespace arena_core
