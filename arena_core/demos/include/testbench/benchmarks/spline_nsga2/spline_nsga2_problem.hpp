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

// System
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

// External libraries
// Eigen
#include <Eigen/Dense>
// Pagmo2
#include <pagmo/types.hpp>


namespace arena_benchmarks
{

/**
 * @brief Pagmo problem of the spline path planning of Ahmed and Deb (ROBIO 2011, Soft Computing 2013).
 *
 * The decision vector holds the (x, y, z) coordinates of the free control points of a B-spline clamped at the start and
 * the goal, bounded by the map. The fitness is given by a callback (3 objectives).
 */
class spline_nsga2_problem
{
public:

    using fitness_callback = std::function<pagmo::vector_double(const pagmo::vector_double&)>;

    spline_nsga2_problem(unsigned a_nb_of_control_points = 1u, std::shared_ptr<fitness_callback> a_fitness = nullptr,
                         const Eigen::Vector3d& a_min_bounds = Eigen::Vector3d::Zero(),
                         const Eigen::Vector3d& a_max_bounds = Eigen::Vector3d::Ones())
    : nb_of_control_points_(a_nb_of_control_points), fitness_(std::move(a_fitness)),
      min_bounds_(a_min_bounds), max_bounds_(a_max_bounds)
    {
        if (nb_of_control_points_ == 0u)
            throw std::invalid_argument("spline_nsga2_problem: at least one free control point is needed.");
    }

    pagmo::vector_double fitness(const pagmo::vector_double& a_dv) const
    {
        if (!fitness_)
            throw std::runtime_error("spline_nsga2_problem: the fitness callback is null.");
        return (*fitness_)(a_dv);
    }

    pagmo::vector_double::size_type get_nobj() const { return 3u; }

    std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const
    {
        pagmo::vector_double lb(3 * nb_of_control_points_), ub(3 * nb_of_control_points_);
        for (unsigned i = 0; i < nb_of_control_points_; ++i)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                lb[3 * i + axis] = min_bounds_[axis];
                ub[3 * i + axis] = max_bounds_[axis];
            }
        }
        return {lb, ub};
    }

    std::string get_name() const { return "Spline NSGA-II (Ahmed and Deb)"; }

private:

    unsigned nb_of_control_points_;
    std::shared_ptr<fitness_callback> fitness_;
    Eigen::Vector3d min_bounds_;
    Eigen::Vector3d max_bounds_;

}; // class spline_nsga2_problem

}; // namespace arena_benchmarks
