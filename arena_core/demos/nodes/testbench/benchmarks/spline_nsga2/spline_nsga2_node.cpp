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
 *     - the path is a uniform B-spline of order 4 (degree 3) clamped at the start and the goal by knot multiplicity, the
 *       decision vector holds the coordinates of spline_nsga2.nb_of_control_points free control points (5 to 8 in the
 *       paper), in their order (non-monotonic paths), bounded by the map (the curve stays in the convex hull of its
 *       control points, it can't overshoot the map)
 *     - the initial population is random within the variable bounds
 *     - the two objectives are the path length (numerical approximation on spline_nsga2.curve_samples points of the
 *       curve) and the path potential: the potential field summed along the path, each value weighted by the length of
 *       the path where it applies (line integral of the potential). The paper lets the potential field be customized to
 *       the environment ("euclidean distance, Gaussian distribution or linearly decreasing potential value"); here it is
 *       the collision cost of the SDF octomap (1 on the obstacles, decreasing with the distance to them), the same field
 *       as the safety costs of MOAR-3D and PO-RRT*
 *     - crossing obstacles is not a constraint: all the objectives are multiplied by 1 + spline_nsga2.penalty_coefficient
 *       times the number of obstacles crossed (here the segments between two points of the curve crossing an occupied
 *       voxel of the inflated octomap)
 *     - NSGA-II, with a mutation probability in the range of the paper (0.03 to 0.08)
 * Differences: pagmo's NSGA-II uses simulated binary crossover and polynomial mutation (uniform operators in the paper);
 * the population size and the number of generations are the ones of ARENA (set by the testbench).
 *
 * The paths have no speed: every path of the non-dominated front of the final population is flown as it is (the curve
 * sampled on spline_nsga2.evaluation_curve_samples points) at the constant speed benchmark.speed, reached and left
 * instantaneously, evaluated with the cost models of ARENA (BenchmarkPlannerNode::scoreTrajectory, the acceleration isn't
 * checked) and reported as a solution of the front.
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
#include <pagmo/utils/multi_objective.hpp>

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
        const int curve_samples = static_cast<int>(param<int64_t>("spline_nsga2.curve_samples", 100));
        const int evaluation_curve_samples = static_cast<int>(param<int64_t>("spline_nsga2.evaluation_curve_samples", 1000));
        const double mutation_probability = param<double>("spline_nsga2.mutation_probability", 0.05);
        const double crossover_probability = param<double>("spline_nsga2.crossover_probability", 0.95);
        const double penalty_coefficient = param<double>("spline_nsga2.penalty_coefficient", 1.0);
        const int nb_of_generations = static_cast<int>(param<int64_t>("optimization.NSGA-II.generations", 1000));
        int population_size = static_cast<int>(param<int64_t>("optimization.NSGA-II.population_size", 40));
        // NSGA-II needs a population size divisible by 4, like ARENA
        if (population_size % 4 != 0)
            population_size += 4 - (population_size % 4);
        const double speed = param<double>("benchmark.speed", linedrone_config_.robot_max_speed_);

        auto sdf = costmap_mapping_->getCapability<arena_demos::CostSDFCapability>();
        if (!sdf)
            throw std::runtime_error("The SDF octomap is not available");
        auto mapping = costmap_mapping_;

        // Points of the clamped uniform cubic B-spline of a decision vector
        auto curve = [&](const pagmo::vector_double& dv, int nb_of_samples) -> Eigen::MatrixXd
        {
            std::vector<arena_core::ControlPoint<double, 4>> control_points;
            control_points.push_back(arena_core::ControlPoint<double, 4>(Eigen::Vector4d(a_start.x(), a_start.y(), a_start.z(), 0.0)));
            for (unsigned i = 0; i < nb_of_control_points; ++i)
                control_points.push_back(arena_core::ControlPoint<double, 4>(Eigen::Vector4d(dv[3 * i], dv[3 * i + 1], dv[3 * i + 2], 0.0)));
            control_points.push_back(arena_core::ControlPoint<double, 4>(Eigen::Vector4d(a_goal.x(), a_goal.y(), a_goal.z(), 0.0)));
            arena_core::Nurbs<4> spline(control_points, nb_of_samples, 3);
            return spline.evaluate().topRows<3>();
        };

        // Objectives of the paper: path length and path potential, penalized proportionally to the obstacles crossed
        auto fitness = std::make_shared<spline_nsga2_problem::fitness_callback>([&](const pagmo::vector_double& dv) -> pagmo::vector_double
        {
            const Eigen::MatrixXd points = curve(dv, curve_samples);
            if (points.hasNaN())
                return pagmo::vector_double(2, std::numeric_limits<double>::max());
            double length = 0.0, potential = 0.0;
            int nb_of_crossings = 0;
            for (int i = 1; i < points.cols(); ++i)
            {
                const Eigen::Vector3d a = points.col(i - 1), b = points.col(i);
                const double segment = (b - a).norm();
                length += segment;
                potential += sdf->getCollisionCost(0.5 * (a + b)) * segment;
                if (segment > 1.0e-9 && mapping->isOccupiedRayTracing(a, b))
                    nb_of_crossings++;
            }
            const double factor = 1.0 + penalty_coefficient * nb_of_crossings;
            return {length * factor, potential * factor};
        });

        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        pagmo::problem problem{spline_nsga2_problem(nb_of_control_points, fitness, bounds.row(0).transpose(), bounds.row(1).transpose(), 2u)};

        auto start_time = std::chrono::steady_clock::now();

        // Random initial population within the variable bounds, as in the paper
        pagmo::population population{problem, static_cast<pagmo::population::size_type>(population_size), pagmo::random_device::next()};

        // Same NSGA-II as ARENA (crossover and distribution indices), with the mutation probability of the paper
        pagmo::algorithm nsga2{pagmo::nsga2(static_cast<unsigned>(nb_of_generations), crossover_probability, 10.0,
                                            mutation_probability, 50.0, pagmo::random_device::next(), nullptr)};
        // The adaptive sorting of the pagmo fork is disabled (null matrix), like ARENA
        nsga2.extract<pagmo::nsga2>()->set_adaptive_matrix(pagmo::vector_double(problem.get_nf(), 0.0));
        population = nsga2.evolve(population);

        // Trade-off front of the paper: non-dominated paths of the final population
        const auto front = std::get<0>(pagmo::fast_non_dominated_sorting(population.get_f()))[0];
        a_result.optimization_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        a_result.nb_of_generations = nb_of_generations;
        a_result.population_size = population_size;
        a_result.nb_of_control_points = static_cast<int>(nb_of_control_points + 2);

        for (pagmo::pop_size_t index : front)
        {
            const Eigen::MatrixXd points = curve(population.get_x()[index], evaluation_curve_samples);
            if (points.hasNaN())
                continue;
            a_result.solutions.push_back(scoreTrajectory(constantSpeedTrajectory(points, speed), false));
        }
        RCLCPP_INFO(get_logger(), "Spline NSGA-II: %zu non-dominated paths in %.2f s", front.size(), a_result.optimization_time);
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
