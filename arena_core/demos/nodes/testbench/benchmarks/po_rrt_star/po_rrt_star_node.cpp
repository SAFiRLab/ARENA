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
 * PO-RRT* benchmark planner: the Pareto-optimal RRT* of Batonisashvili ("PO-RRT*: Pareto-Optimal, Safety-Aware,
 * Multi-Objective Path Planning", MS thesis, Boston University, 2026), in 3D (PoRrtStar).
 *
 * As in the thesis, the two objectives are the Euclidean length and the log-survival of the path over an occupancy
 * probability map, cells at or above po_rrt_star.occupancy_threshold (0.9) are impassable. The occupancy probability q of a
 * point is the collision cost of the SDF octomap (1 on the obstacles, decreasing with the distance to them, like the
 * "safe distance" circular obstacles of the thesis), the cells are the voxels of the octomap. The points must also be valid
 * like the states of the RRT* benchmark and of the RRT of ARENA (no occupied voxel of the inflated octomap within 0.8 m).
 * The step size eta is the RRT range (optimization.initialization.rrt_range, like RRT*), the tree grows during
 * po_rrt_star.time_budget seconds.
 *
 * PO-RRT* plans geometric paths without speeds: every path of the Pareto front is flown as it is at the constant speed of
 * the RRT* benchmark (benchmark.speed, reached and left instantaneously), evaluated with the cost models of ARENA
 * (BenchmarkPlannerNode::scoreTrajectory, the acceleration isn't checked) and reported as a solution of the front.
 */

// Local
#include "common/benchmark_planner_node.h"
#include "testbench/benchmarks/po_rrt_star/po_rrt_star.h"

// System
#include <chrono>
#include <memory>


namespace arena_benchmarks
{

class PoRrtStarNode : public BenchmarkPlannerNode
{
public:

    explicit PoRrtStarNode(const rclcpp::NodeOptions& a_options)
    : BenchmarkPlannerNode("linedrone_test_node", a_options)
    {}

protected:

    void plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result) override
    {
        const double speed = param<double>("benchmark.speed", linedrone_config_.robot_max_speed_);
        const double clearance = param<double>("po_rrt_star.clearance", 0.8);

        PoRrtStarConfig config;
        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        config.min_bounds = bounds.row(0).transpose();
        config.max_bounds = bounds.row(1).transpose();
        config.step_size = param<double>("optimization.initialization.rrt_range", 5.0);
        config.rewire_factor = param<double>("po_rrt_star.rewire_factor", 1.1);
        config.time_budget = param<double>("po_rrt_star.time_budget", 5.0);
        config.occupancy_threshold = param<double>("po_rrt_star.occupancy_threshold", 0.9);
        config.edge_resolution = param<double>("po_rrt_star.edge_resolution", 0.5);
        config.cell_size = param<double>("po_rrt_star.cell_size", 1.0);
        config.max_nodes = static_cast<size_t>(param<int64_t>("po_rrt_star.max_nodes", 5000000));
        a_result.rrt_range = config.step_size;

        auto sdf = costmap_mapping_->getCapability<arena_demos::CostSDFCapability>();
        if (!sdf)
            throw std::runtime_error("The SDF octomap is not available");
        auto mapping = costmap_mapping_;
        PoRrtStar planner(config,
                          [mapping, clearance](const Eigen::Vector3d& p) { return mapping->isClearance(p, clearance); },
                          [sdf](const Eigen::Vector3d& p) { return sdf->getCollisionCost(p); });

        PoRrtStarStats stats;
        auto start_time = std::chrono::steady_clock::now();
        std::vector<PoRrtStarPath> paths = planner.plan(a_start, a_goal, &stats);
        a_result.initialization_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        RCLCPP_INFO(get_logger(), "PO-RRT*: %zu Pareto-optimal paths in %.2f s (%zu iterations, %zu nodes, %zu parent changes, "
                    "%zu alternative nodes, %zu goal nodes)", paths.size(), a_result.initialization_time, stats.nb_of_iterations,
                    stats.nb_of_nodes, stats.nb_of_parent_changes, stats.nb_of_alternative_nodes, stats.nb_of_goal_nodes);

        // Every Pareto-optimal path flown as it is at constant speed, like RRT*
        for (const PoRrtStarPath& path : paths)
        {
            a_result.solutions.push_back(scoreTrajectory(constantSpeedTrajectory(path.waypoints, speed), false));
            a_result.nb_of_control_points = static_cast<int>(path.waypoints.size());
        }
    }

}; // class PoRrtStarNode

}; // namespace arena_benchmarks


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<arena_benchmarks::PoRrtStarNode>(options));
    rclcpp::shutdown();
    return 0;
}
