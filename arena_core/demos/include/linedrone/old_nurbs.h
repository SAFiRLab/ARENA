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
#include <cmath>

// External libraries
// Eigen
#include <Eigen/Dense>


namespace arena_demos
{

class OldNurbs
{
public:
    OldNurbs(const std::vector<Eigen::VectorXd>& control_points, int sample_size, 
          std::vector<double> weights = std::vector<double>(), int degree = 5);

    Eigen::VectorXd* evaluate() const;
    std::vector<std::vector<std::vector<double>>> derivatives(int order) const;

    // Setters
    bool setDegree(int degree);
    bool setSampleSize(int sample_size);
    bool setControlPoints(const std::vector<Eigen::VectorXd>& control_points);
    bool setControlPoint(const Eigen::VectorXd& control_point, int index);
    bool setWeightedControlPoints(const std::vector<Eigen::VectorXd>& control_points, 
                                    const std::vector<double>& weights);

    // Getters
    int getDegree() const { return degree_; };
    int getSampleSize() const { return sample_size_; };
    std::vector<Eigen::VectorXd> getControlPoints() const { return control_points_; };

private:

    // User defined methods
    bool initialize();
    std::vector<double> generateKnotVector() const;

    std::vector<int> findSpans() const;
    int findSpan(double parameter) const;

    void basisFunctions(double** basis) const;
    void basisFunction(int span, double parameter, double* N) const;
    std::vector<std::vector<std::vector<double>>> basisFunctionsDerivatives(int order) const;
    std::vector<std::vector<double>> basisFunctionDerivatives(int order, int span, double parameter) const;

    // User defined attributes
    std::vector<Eigen::VectorXd> control_points_;
    std::vector<double> weights_;
    std::vector<double> knot_vector_;
    int sample_size_;
    int degree_;
    int dimension_;
    bool adequate_conf_;
    std::vector<double> parameters_space_;
    std::vector<int> spans_;

}; // class OldNurbs

}; // namespace arena_demos
