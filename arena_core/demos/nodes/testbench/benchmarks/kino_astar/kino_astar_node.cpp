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
 * Kinodynamic A* benchmark planner: the kinodynamic path searching of Fast-Planner (Zhou, Gao, Wang, Liu and Shen, RA-L
 * 2019), hybrid-state A* with double integrator motion primitives minimizing int ||u||^2 dt + rho T (KinodynamicAstar,
 * port of its code). Only the front-end (path searching) is used, like the comparison of path searching of the paper
 * (Table I), the B-spline optimization is a separate stage.
 *
 * As in Fast-Planner (kinodynamicReplan): a first search with the initial acceleration (init), and if no path is found,
 * a second search with all the inputs; the search succeeds when the analytic expansion reaches the goal at rest
 * (REACH_END), the other outcomes don't reach the goal. Parameters of kino_algorithm.xml, except:
 *     - the limits of Fast-Planner apply to every component (box), they are set to the limits of the robot divided by
 *       sqrt(3) so that ||v|| <= robot.max_speed and ||u|| <= robot.max_acceleration (kino_astar.max_vel/max_acc override)
 *     - horizon: the whole map (global planning instead of the receding horizon of Fast-Planner)
 *     - allocate_num: sized for the maps of the tests
 * The voxels are occupied when the validity of the other benchmarks fails (occupied voxel of the inflated octomap within
 * kino_astar.clearance). The trajectory is evaluated as planned with the cost models of ARENA, its acceleration included
 * (BenchmarkPlannerNode::scoreTrajectory); the analytic expansion isn't checked against the limits in Fast-Planner.
 */

// Local
#include "common/benchmark_planner_node.h"
#include "testbench/benchmarks/kino_astar/kinodynamic_astar.h"

// System
#include <chrono>
#include <cmath>


namespace arena_benchmarks
{

class KinoAstarNode : public BenchmarkPlannerNode
{
public:

    explicit KinoAstarNode(const rclcpp::NodeOptions& a_options)
    : BenchmarkPlannerNode("linedrone_test_node", a_options)
    {}

protected:

    void plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result) override
    {
        const double clearance = param<double>("kino_astar.clearance", 0.8);
        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();

        KinodynamicAstarConfig config;
        config.max_tau = param<double>("kino_astar.max_tau", 0.4);
        config.init_max_tau = param<double>("kino_astar.init_max_tau", 0.8);
        config.max_vel = param<double>("kino_astar.max_vel", linedrone_config_.robot_max_speed_ / std::sqrt(3.0));
        config.max_acc = param<double>("kino_astar.max_acc", linedrone_config_.robot_max_acceleration_ / std::sqrt(3.0));
        config.w_time = param<double>("kino_astar.w_time", 10.0);
        config.horizon = param<double>("kino_astar.horizon", (bounds.row(1) - bounds.row(0)).norm());
        config.resolution_astar = param<double>("kino_astar.resolution_astar", 0.1);
        config.lambda_heu = param<double>("kino_astar.lambda_heu", 5.0);
        config.allocate_num = static_cast<int>(param<int64_t>("kino_astar.allocate_num", 20000000));
        config.check_num = static_cast<int>(param<int64_t>("kino_astar.check_num", 5));
        config.origin = bounds.row(0).transpose();
        config.map_size = (bounds.row(1) - bounds.row(0)).transpose();
        a_result.rrt_range = config.resolution_astar;

        auto mapping = costmap_mapping_;
        KinodynamicAstar search(config, [mapping, clearance](const Eigen::Vector3d& p) { return !mapping->isClearance(p, clearance); });

        // kinodynamicReplan of Fast-Planner: from rest, to the goal at rest
        const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
        auto start_time = std::chrono::steady_clock::now();
        search.reset();
        int status = search.search(a_start, zero, zero, a_goal, zero, true);
        if (status == KinodynamicAstar::NO_PATH)
        {
            // Retry searching with discontinuous initial state
            search.reset();
            status = search.search(a_start, zero, zero, a_goal, zero, false);
        }
        a_result.initialization_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

        const char* names[] = {"", "reach horizon", "reach end", "no path", "near end"};
        RCLCPP_INFO(get_logger(), "Kinodynamic A*: %s in %.3f s (%d iterations, %d nodes)", names[status],
                    a_result.initialization_time, search.getNbOfIterations(), search.getNbOfNodes());
        if (status != KinodynamicAstar::REACH_END)
            return;

        NativeTrajectory trajectory = search.getTrajectory(param<double>("kino_astar.sampling_step", 0.05));
        a_result.solutions.push_back(scoreTrajectory(trajectory, true));
        a_result.nb_of_control_points = static_cast<int>(trajectory.size());
    }

}; // class KinoAstarNode

}; // namespace arena_benchmarks


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<arena_benchmarks::KinoAstarNode>(options));
    rclcpp::shutdown();
    return 0;
}
