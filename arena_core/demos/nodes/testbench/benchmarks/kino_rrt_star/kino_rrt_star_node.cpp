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
 * Kinodynamic RRT* benchmark planner: Kinodynamic RRT* of Webb and van den Berg (ICRA 2013), the kinodynamic extension of
 * RRT* for linear dynamics, on the 3D double integrator of their experiments (KinodynamicRrtStar).
 *
 * As in the paper, the bounds of the state and control spaces are part of the problem (X_free, U_free): the limits of the
 * robot ||v|| <= robot.max_speed and ||u|| <= robot.max_acceleration, and positions valid like the states of the RRT*
 * benchmark and of the RRT of ARENA (no occupied voxel of the inflated octomap within kino_rrt_star.clearance), in the
 * map bounds extended to the start and the goal. The start and the goal states are at rest. The control penalty r of
 * R = r I is chosen like in the paper: so that the robot reaches velocities near its bound without frequently exceeding
 * it (kino_rrt_star_params.yaml). The trajectory is evaluated as planned with the cost models of ARENA, its acceleration
 * included (BenchmarkPlannerNode::scoreTrajectory).
 */

// Local
#include "common/benchmark_planner_node.h"
#include "testbench/benchmarks/kino_rrt_star/kinodynamic_rrt_star.h"


namespace arena_benchmarks
{

class KinoRrtStarNode : public BenchmarkPlannerNode
{
public:

    explicit KinoRrtStarNode(const rclcpp::NodeOptions& a_options)
    : BenchmarkPlannerNode("linedrone_test_node", a_options)
    {}

protected:

    void plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result) override
    {
        const double clearance = param<double>("kino_rrt_star.clearance", 0.8);

        KinodynamicRrtStarConfig config;
        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        // Positions of X_free: the map bounds (octomap leaves) extended to the start and the goal
        config.min_bounds = bounds.row(0).transpose().cwiseMin(a_start).cwiseMin(a_goal);
        config.max_bounds = bounds.row(1).transpose().cwiseMax(a_start).cwiseMax(a_goal);
        config.max_speed = linedrone_config_.robot_max_speed_;
        config.max_acceleration = linedrone_config_.robot_max_acceleration_;
        config.control_penalty = param<double>("kino_rrt_star.control_penalty", 50.0);
        config.neighbor_radius = param<double>("kino_rrt_star.neighbor_radius", 20.0);
        config.check_step = param<double>("kino_rrt_star.check_step", 0.1);
        config.time_budget = param<double>("kino_rrt_star.time_budget", 5.0);
        config.max_nodes = static_cast<size_t>(param<int64_t>("kino_rrt_star.max_nodes", 2000000));
        a_result.rrt_range = config.neighbor_radius;

        auto mapping = costmap_mapping_;
        NativeTrajectory trajectory;
        KinodynamicRrtStarStats stats;
        const bool found = KinodynamicRrtStar(config, [mapping, clearance](const Eigen::Vector3d& p) { return mapping->isClearance(p, clearance); })
                               .plan(a_start, a_goal, trajectory, param<double>("kino_rrt_star.sampling_step", 0.05), &stats);
        a_result.initialization_time = stats.time;
        RCLCPP_INFO(get_logger(), "Kinodynamic RRT*: %s in %.2f s (%zu iterations, %zu nodes, %zu improvements, first after %.2f s), "
                    "cost %.1f, duration %.1f s", found ? "goal reached" : "no trajectory", stats.time, stats.nb_of_iterations,
                    stats.nb_of_nodes, stats.nb_of_goal_improvements, stats.first_solution_time, found ? stats.cost : 0.0,
                    found ? trajectory.back().time : 0.0);
        if (!found)
            return;

        a_result.solutions.push_back(scoreTrajectory(trajectory, true));
        a_result.nb_of_control_points = static_cast<int>(trajectory.size());
    }

}; // class KinoRrtStarNode

}; // namespace arena_benchmarks


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<arena_benchmarks::KinoRrtStarNode>(options));
    rclcpp::shutdown();
    return 0;
}
