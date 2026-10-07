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

/**
 * Spline NSGA-II benchmark planner: multi-objective path planning with splines of Ahmed and Deb ("Multi-objective path
 * planning using spline representation", ROBIO 2011, and Soft Computing 2013), extended to 3D.
 *
 * As in the paper:
 *     - the path is a uniform B-spline of order 4 (degree 3) clamped at the start and the goal, the decision vector holds
 *       the coordinates of spline_nsga2.nb_of_control_points free control points (5 to 8 in the paper), in their order
 *       (non-monotonic paths), bounded by the map
 *     - the initial population is random within the variable bounds
 *     - crossing obstacles is not a constraint: all the objectives are penalized proportionally to the number of
 *       obstacles crossed (here the segments between two samples crossing an occupied voxel of the inflated octomap)
 *     - NSGA-II, with a mutation probability in the range of the paper (0.03 to 0.08)
 * Differences, needed to compare the planners on the same objectives:
 *     - the objectives are the time, safety and energy costs of ARENA (the paper minimizes the length and a safety measure)
 *       on the curve sampled like ARENA, with a constant speed (no speed in the decision vector)
 *     - with spline_nsga2.penalize_acceleration, the acceleration above the limit of the robot is also penalized (the
 *       paper plans 2D paths without dynamics)
 *     - pagmo's NSGA-II uses simulated binary crossover and polynomial mutation (uniform operators in the paper)
 * The final population is evaluated without penalty, with the constraints and the safety check of ARENA.
 */

// Local
#include "common/benchmark_planner_node.h"
#include "testbench/benchmarks/spline_nsga2/spline_nsga2_problem.hpp"

// External Libraries
// Pagmo
#include <pagmo/algorithms/nsga2.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/rng.hpp>

// System
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>


namespace arena_benchmarks
{

class SplineNSGA2Node : public BenchmarkPlannerNode
{
public:

    explicit SplineNSGA2Node(const rclcpp::NodeOptions& a_options)
    : BenchmarkPlannerNode("linedrone_test_node", a_options)
    {}

protected:

    void plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result) override
    {
        const unsigned nb_of_control_points = static_cast<unsigned>(param<int64_t>("spline_nsga2.nb_of_control_points", 8));
        const double mutation_probability = param<double>("spline_nsga2.mutation_probability", 0.05);
        const double crossover_probability = param<double>("spline_nsga2.crossover_probability", 0.95);
        const double penalty_coefficient = param<double>("spline_nsga2.penalty_coefficient", 1.0);
        const bool penalize_acceleration = param<bool>("spline_nsga2.penalize_acceleration", true);
        const int nb_of_generations = static_cast<int>(param<int64_t>("optimization.NSGA-II.generations", 1000));
        int population_size = static_cast<int>(param<int64_t>("optimization.NSGA-II.population_size", 40));
        // NSGA-II needs a population size divisible by 4, like ARENA
        if (population_size % 4 != 0)
            population_size += 4 - (population_size % 4);
        const double speed = param<double>("benchmark.speed", linedrone_config_.robot_max_speed_);
        const double ramp_length = param<double>("benchmark.speed_ramp_length", 0.0);
        const double max_acceleration = linedrone_config_.robot_max_acceleration_;

        auto toControlPoints = [&](const pagmo::vector_double& dv) -> Eigen::MatrixXd
        {
            Eigen::MatrixXd waypoints(3, nb_of_control_points + 2);
            waypoints.col(0) = a_start;
            for (unsigned i = 0; i < nb_of_control_points; ++i)
                waypoints.col(i + 1) = Eigen::Vector3d(dv[3 * i], dv[3 * i + 1], dv[3 * i + 2]);
            waypoints.col(nb_of_control_points + 1) = a_goal;
            return scorer().toControlPoints(waypoints, speed, ramp_length);
        };

        // Objectives of ARENA penalized proportionally to the number of obstacles crossed
        auto fitness = std::make_shared<spline_nsga2_problem::fitness_callback>([&](const pagmo::vector_double& dv) -> pagmo::vector_double
        {
            ScoredPath scored = scorer().evaluate(toControlPoints(dv));
            const double max_cost = std::numeric_limits<double>::max();
            pagmo::vector_double f = scored.fitness;
            for (double value : f)
            {
                if (!std::isfinite(value) || value == max_cost)
                    return pagmo::vector_double(3, max_cost);
            }

            double violation = static_cast<double>(scorer().countUnsafeSegments(scored.samples));
            if (penalize_acceleration && scored.max_acceleration > max_acceleration)
                violation += (scored.max_acceleration - max_acceleration) / max_acceleration;

            const double factor = 1.0 + penalty_coefficient * violation;
            for (double& value : f)
                value *= factor;
            return f;
        });

        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        pagmo::problem problem{spline_nsga2_problem(nb_of_control_points, fitness, bounds.row(0).transpose(), bounds.row(1).transpose())};

        auto start_time = std::chrono::steady_clock::now();

        // Random initial population within the variable bounds, as in the paper
        pagmo::population population{problem, static_cast<pagmo::population::size_type>(population_size), pagmo::random_device::next()};

        // Same NSGA-II as ARENA (crossover and distribution indices), with the mutation probability of the paper
        pagmo::algorithm nsga2{pagmo::nsga2(static_cast<unsigned>(nb_of_generations), crossover_probability, 10.0,
                                            mutation_probability, 50.0, pagmo::random_device::next(), nullptr)};
        // The adaptive sorting of the pagmo fork is disabled (null matrix), like ARENA
        nsga2.extract<pagmo::nsga2>()->set_adaptive_matrix(pagmo::vector_double(problem.get_nf(), 0.0));
        population = nsga2.evolve(population);

        a_result.optimization_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        a_result.nb_of_generations = nb_of_generations;
        a_result.population_size = population_size;

        // Final population evaluated without penalty, with the constraints and the safety check of ARENA
        for (const pagmo::vector_double& dv : population.get_x())
        {
            Eigen::MatrixXd control_points = toControlPoints(dv);
            a_result.solutions.push_back(scorer().score(control_points));
            a_result.nb_of_control_points = static_cast<int>(control_points.cols());
        }
    }

}; // class SplineNSGA2Node

}; // namespace arena_benchmarks


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<arena_benchmarks::SplineNSGA2Node>(options));
    rclcpp::shutdown();
    return 0;
}
