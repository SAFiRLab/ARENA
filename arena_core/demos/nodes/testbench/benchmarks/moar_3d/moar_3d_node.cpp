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
 * MOAR-3D benchmark planner: the MOAR planner (Petit and Desbiens, "MOAR Planner: Multi-Objective and Adaptive Risk-Aware
 * Path Planning for Infrastructure Inspection with a UAV", ICRA 2024) extended from its 2D plane to 3D, and the A*
 * benchmark of the same paper.
 *
 * Modes (moar_3d.mode):
 *     astar: A* minimizing the path length on the grid (benchmark "A*" of the MOAR paper), flown at benchmark.speed
 *     moar: MOAR graph search (MoarGraphSearch) with the coefficients k_T, k_S, k_E of Eq. 9 of MOAR (Eq. 11 of the
 *           first ARENA submission, arena_core::RankVotingAlgorithm) computed from the risks optimization.voting.risks.*
 *           and the nominal coefficients optimization.voting.nominal_coefficients.* with optimization.voting.use_risks,
 *           given by optimization.adaptive_costs_weights.{time, safety, energy} otherwise. The normalization factor
 *           gamma doesn't change the optimal path.
 *     moar_weight_sweep: one MOAR search per coefficient set of the simplex (step moar_3d.sweep_step), the solutions are
 *           reported like a Pareto front and the voting algorithm of ARENA (optimization.voting) chooses one
 *
 * As in the paper and its implementation (MOPP): A* on states (cell, direction of flight), every action of Fig. 5 including
 * the static action "0" (stop in the cell: branch length of a diagonal step, time cost of 1 at a null speed, velocity
 * change to stop), costs of Eq. 7 (SC d, TC = d / v(d_obs, theta), EC = ||v_i - v_i-1|| / (2 v_MAX)), heuristic of Eq. 8,
 * speeds v = min(v_sat(d_obs), v_max,theta) with the maximum speed variation function v_sat of Eq. 5 (d_min of the
 * collision cost, moar_3d.speeds.d_max = d_max_speed of the implementation), coefficients of Eq. 9 (with the adaptation of
 * k_E and the normalization gamma of the paper; the implementation keeps k_E constant).
 *
 * Differences with the 2D MOAR planner, needed to run it on the 3D octomaps of ARENA:
 *     - 3D grid of moar_3d.grid_resolution with 26 motion primitives (+ the static action) instead of 8 (+ static) in 2D
 *     - the safety cost of a cell is the collision cost C(d_obs) only (k_a = 1 in Eq. 2). C is the potential function of MOAR
 *       (Eq. 3, computeCollisionCost of the implementation), read from the SDF octomap of ARENA (same function, d_min and
 *       d_max of the costmap, shared with ARENA's safety cost). The insertion (convex hull)
 *       and damage costs of MOAR need the semantics of a power line (inspected structure, conductors below the drone) that
 *       the octomaps don't have, and ARENA doesn't use its own insertion cost in these tests either.
 *     - the obstacle distance d_obs of the speed function (Eq. 5) is computed back from the SDF cost
 *     - the speeds are the ones of the MOAR implementation (minimum, horizontal and directional cruising speeds) scaled to
 *       the maximum speed of the robot (robot.max_speed is the horizontal cruising speed v_MAX)
 * The cells are valid like the states of the RRT of ARENA (no occupied voxel of the inflated octomap closer than
 * moar_3d.clearance on every axis).
 *
 * Trajectories, as planned (like the nav_msgs/Path of the cell centers published by the MOAR implementation): the path goes
 * through the cell centers (the start and the goal replace the centers of their cells), every segment is flown at the
 * speed planned by MOAR (A*: the constant speed benchmark.speed), the speed changes instantaneously at the cell centers
 * (MOAR assumes the cruising speed is reached quickly). They are evaluated with the cost models of ARENA
 * (BenchmarkPlannerNode::scoreTrajectory, the acceleration isn't checked).
 */

// Local
#include "common/benchmark_planner_node.h"
#include "testbench/benchmarks/moar_3d/moar_graph_search.h"
#include "testbench/benchmarks/moar_3d/moar_graph_setup.h"
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
        const std::string mode = param<std::string>("moar_3d.mode", "moar");
        a_result.rrt_range = param<double>("moar_3d.grid_resolution", 1.0);

        if (mode == "astar")
            planAStar(a_start, a_goal, a_result);
        else if (mode == "moar" || mode == "moar_weight_sweep")
            planMoar(a_start, a_goal, mode == "moar_weight_sweep", a_result);
        else
            throw std::runtime_error("Unknown moar_3d.mode " + mode + " (astar, moar or moar_weight_sweep)");
    }

private:

    /** @brief Grid configuration on the map bounds, true if it changed since the last planning. */
    bool gridConfig(double& a_resolution, Eigen::Vector3d& a_min, Eigen::Vector3d& a_max)
    {
        a_resolution = param<double>("moar_3d.grid_resolution", 1.0);
        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        a_min = bounds.row(0).transpose();
        a_max = bounds.row(1).transpose();
        bool changed = a_resolution != resolution_ || !a_min.isApprox(min_bounds_) || !a_max.isApprox(max_bounds_);
        resolution_ = a_resolution;
        min_bounds_ = a_min;
        max_bounds_ = a_max;
        return changed;
    }

    void planAStar(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result)
    {
        const double clearance = param<double>("moar_3d.clearance", 0.8);
        const double speed = param<double>("benchmark.speed", linedrone_config_.robot_max_speed_);

        WeightedAStar3DConfig config;
        bool changed = gridConfig(config.resolution, config.min_bounds, config.max_bounds);
        if (!astar_ || changed)
        {
            auto mapping = costmap_mapping_;
            astar_ = std::make_unique<WeightedAStar3D>(config, [mapping, clearance](const Eigen::Vector3d& p)
                                                       { return mapping->isClearance(p, clearance); });
        }
        else
            astar_->clearCache(); // The map is received again before every planning, like the RRT of ARENA

        // Shortest path: cost and admissible heuristic are the euclidean length
        auto length = [](const Eigen::Vector3d& from, const Eigen::Vector3d& to) { return (to - from).norm(); };

        std::vector<Eigen::Vector3d> path;
        WeightedAStar3DStats stats;
        auto start_time = std::chrono::steady_clock::now();
        bool found = astar_->search(a_start, a_goal, length, length, path, &stats);
        a_result.initialization_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        RCLCPP_INFO(get_logger(), "A*: %s in %.3f s, %zu expansions, length %.2f m", found ? "path found" : "no path",
                    a_result.initialization_time, stats.nb_of_expansions, stats.cost);
        if (!found)
            return;

        // Path of the cell centers flown at a constant speed
        a_result.solutions.push_back(scoreTrajectory(constantSpeedTrajectory(path, speed), false));
        a_result.nb_of_control_points = static_cast<int>(path.size());
    }

    void planMoar(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, bool a_sweep, PlanningResult& a_result)
    {
        const double clearance = param<double>("moar_3d.clearance", 0.8);
        const double sdf_d_min = param<double>("moar_3d.sdf_min_distance", 2.0);
        const double sdf_d_max = param<double>("moar_3d.sdf_max_distance", 9.0);
        const MoarSpeeds speeds = moarSpeeds(linedrone_config_.robot_max_speed_,
                                             [this](const std::string& name, double value) { return param<double>(name, value); });

        auto sdf = costmap_mapping_->getCapability<arena_demos::CostSDFCapability>();
        if (!sdf)
            throw std::runtime_error("The SDF octomap is not available");

        MoarGraphConfig config;
        bool changed = gridConfig(config.resolution, config.min_bounds, config.max_bounds);
        if (!moar_ || changed)
            moar_ = std::make_unique<MoarGraphSearch>(config, moarCellFn(costmap_mapping_, sdf, clearance, sdf_d_min, sdf_d_max));
        else
            moar_->clearCache();

        // Coefficients (k_S, k_T, k_E), normalized to a sum of 1 (gamma in Eq. 9)
        std::vector<MoarWeights> weight_sets;
        if (a_sweep)
        {
            const int steps = std::max(1, static_cast<int>(std::lround(1.0 / param<double>("moar_3d.sweep_step", 0.1))));
            for (int i = 0; i <= steps; ++i)
                for (int j = 0; j <= steps - i; ++j)
                    weight_sets.push_back({static_cast<double>(j) / steps, static_cast<double>(i) / steps,
                                           static_cast<double>(steps - i - j) / steps});
        }
        else
        {
            // Eq. 9 of MOAR from the risks, or the fixed coefficients (time, safety, energy)
            const arena_core::ObjectiveCoefficients w = votingUsesRisks() ?
                arena_core::RankVotingAlgorithm(getVotingSettings().nominal_coefficients).computeCoefficients(getOperationalRisks()) :
                getFixedCoefficients();
            double sum = w[0] + w[1] + w[2];
            if (sum <= 0.0)
                weight_sets.push_back(MoarWeights());
            else
                weight_sets.push_back({w[1] / sum, w[0] / sum, w[2] / sum});
        }

        double search_time = 0.0;
        for (const MoarWeights& weights : weight_sets)
        {
            std::vector<MoarPathNode> path;
            MoarSearchStats stats;
            auto start_time = std::chrono::steady_clock::now();
            bool found = moar_->search(a_start, a_goal, weights, speeds, path, &stats);
            double duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            search_time += duration;

            RCLCPP_INFO(get_logger(), "MOAR (k_S %.2f, k_T %.2f, k_E %.2f): %s in %.3f s, %zu expansions, %zu nodes",
                        weights.k_s, weights.k_t, weights.k_e, found ? "path found" : "no path", duration,
                        stats.nb_of_expansions, path.size());
            if (!found)
                continue;

            // Path of the cell centers with the speeds planned by MOAR
            a_result.solutions.push_back(scoreTrajectory(moarTrajectory(path), false));
            a_result.nb_of_control_points = static_cast<int>(path.size());
        }

        a_result.initialization_time = search_time;
    }

    std::unique_ptr<WeightedAStar3D> astar_;
    std::unique_ptr<MoarGraphSearch> moar_;
    double resolution_ = 0.0;
    Eigen::Vector3d min_bounds_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bounds_ = Eigen::Vector3d::Zero();

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
