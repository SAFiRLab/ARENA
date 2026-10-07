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
 * MOAR-3D benchmark planner: 3D re-implementation of the MOAR planner (Petit and Desbiens, ICRA 2024).
 *
 * MOAR combines the time, safety and energy costs in a single weighted cost, the weights being adapted to the mission
 * risks, and finds the path of minimal cost with a graph search. MOAR searches a 2D graph, this version searches a 3D
 * grid (26-connectivity) with A*. The cost of an edge of length l in the direction d ending in the cell c is
 *     l * (w_T + w_S * sdf_cost(c) + w_E * P(d) / P_max) (+ a small length cost so null weights don't make detours)
 * with the per-meter costs normalized to [0, 1]: time (constant speed), SDF safety cost of ARENA, steady-state power of
 * the energy model of ARENA divided by its maximum over the grid directions. With w = (1, 0, 0) it is a plain A*.
 *
 * Modes (moar_3d.mode):
 *     single: one search with the weights optimization.adaptive_costs_weights.* (set by the testbench, e.g. from the risks)
 *     weight_sweep: one search per weight set of the simplex with a step of moar_3d.sweep_step, the solutions are reported
 *                   like a Pareto front and the adaptive voting algorithm chooses one, like ARENA
 *
 * The cells are valid like the states of the RRT of ARENA (no occupied voxel of the inflated octomap closer than
 * moar_3d.clearance on every axis). The path goes through the cell centers, it is resampled every
 * moar_3d.waypoint_spacing meters to give the control points of the NURBS curve evaluated like the trajectories of ARENA.
 */

// Local
#include "common/benchmark_planner_node.h"
#include "testbench/benchmarks/moar_3d/weighted_astar_3d.h"

// System
#include <chrono>
#include <cmath>
#include <memory>


namespace arena_benchmarks
{

class Moar3DNode : public BenchmarkPlannerNode
{
public:

    explicit Moar3DNode(const rclcpp::NodeOptions& a_options)
    : BenchmarkPlannerNode("linedrone_test_node", a_options)
    {}

protected:

    void plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result) override
    {
        const std::string mode = param<std::string>("moar_3d.mode", "single");
        const double resolution = param<double>("moar_3d.grid_resolution", 1.0);
        const double clearance = param<double>("moar_3d.clearance", 0.8);
        const double waypoint_spacing = param<double>("moar_3d.waypoint_spacing", 2.0);
        const double length_cost = param<double>("moar_3d.length_cost", 1.0e-3);
        const double speed = param<double>("benchmark.speed", linedrone_config_.robot_max_speed_);
        const double ramp_length = param<double>("benchmark.speed_ramp_length", 0.0);

        // Grid on the map, the cells are rebuilt when the map bounds or the resolution change
        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        WeightedAStar3DConfig config;
        config.resolution = resolution;
        config.min_bounds = bounds.row(0).transpose();
        config.max_bounds = bounds.row(1).transpose();
        if (!astar_ || astar_->getConfig().resolution != resolution || !astar_->getConfig().min_bounds.isApprox(config.min_bounds) ||
            !astar_->getConfig().max_bounds.isApprox(config.max_bounds))
        {
            auto mapping = costmap_mapping_;
            astar_ = std::make_unique<WeightedAStar3D>(config, [mapping, clearance](const Eigen::Vector3d& p)
                                                       { return mapping->isClearance(p, clearance); });
        }
        else
        {
            // The map is received again before every planning, the validity is computed again like the RRT of ARENA
            astar_->clearCache();
        }

        auto sdf = costmap_mapping_->getCapability<arena_demos::CostSDFCapability>();
        if (!sdf)
            throw std::runtime_error("The SDF octomap is not available");

        // Steady-state power in the 26 directions of the grid, normalized by its maximum
        double max_power = 0.0, min_power = std::numeric_limits<double>::max();
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    double power = scorer().steadyStatePower(Eigen::Vector3d(dx, dy, dz));
                    max_power = std::max(max_power, power);
                    min_power = std::min(min_power, power);
                }

        std::vector<std::vector<double>> weight_sets;
        if (mode == "weight_sweep")
        {
            const int steps = std::max(1, static_cast<int>(std::lround(1.0 / param<double>("moar_3d.sweep_step", 0.1))));
            for (int i = 0; i <= steps; ++i)
                for (int j = 0; j <= steps - i; ++j)
                    weight_sets.push_back({static_cast<double>(i) / steps, static_cast<double>(j) / steps,
                                           static_cast<double>(steps - i - j) / steps});
        }
        else if (mode == "single")
            weight_sets.push_back(getCostsWeights());
        else
            throw std::runtime_error("Unknown moar_3d.mode " + mode + " (single or weight_sweep)");

        double search_time = 0.0;
        for (std::vector<double> weights : weight_sets)
        {
            // Only the ratios of the weights matter
            double sum = weights[0] + weights[1] + weights[2];
            if (sum <= 0.0)
                weights = {1.0, 0.0, 0.0};
            else
                for (double& w : weights)
                    w /= sum;

            auto edge_cost = [&](const Eigen::Vector3d& from, const Eigen::Vector3d& to) -> double
            {
                Eigen::Vector3d displacement = to - from;
                double length = displacement.norm();
                double energy = weights[2] > 0.0 ? scorer().steadyStatePower(displacement) / max_power : 0.0;
                double safety = weights[1] > 0.0 ? sdf->getCollisionCost(to) : 0.0;
                return length * (length_cost + weights[0] + weights[1] * safety + weights[2] * energy);
            };
            // Admissible: every edge costs at least its length times this factor
            const double heuristic_factor = length_cost + weights[0] + weights[2] * min_power / max_power;
            auto heuristic = [heuristic_factor](const Eigen::Vector3d& from, const Eigen::Vector3d& goal) -> double
            { return heuristic_factor * (goal - from).norm(); };

            std::vector<Eigen::Vector3d> path;
            WeightedAStar3DStats stats;
            auto start_time = std::chrono::steady_clock::now();
            bool found = astar_->search(a_start, a_goal, edge_cost, heuristic, path, &stats);
            double duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            search_time += duration;

            RCLCPP_INFO(get_logger(), "Weights (%.2f, %.2f, %.2f): %s in %.3f s, %zu expansions, %zu cells",
                        weights[0], weights[1], weights[2], found ? "path found" : "no path", duration,
                        stats.nb_of_expansions, path.size());
            if (!found)
                continue;

            Eigen::MatrixXd waypoints(3, path.size());
            for (size_t i = 0; i < path.size(); ++i)
                waypoints.col(i) = path[i];
            Eigen::MatrixXd control_points = scorer().toControlPoints(BaselinePathScorer::resampleByDistance(waypoints, waypoint_spacing),
                                                                      speed, ramp_length);
            a_result.solutions.push_back(scorer().score(control_points));
            a_result.nb_of_control_points = static_cast<int>(control_points.cols());
        }

        a_result.initialization_time = search_time;
        a_result.rrt_range = waypoint_spacing;
    }

private:

    std::unique_ptr<WeightedAStar3D> astar_;

}; // class Moar3DNode

}; // namespace arena_benchmarks


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<arena_benchmarks::Moar3DNode>(options));
    rclcpp::shutdown();
    return 0;
}
