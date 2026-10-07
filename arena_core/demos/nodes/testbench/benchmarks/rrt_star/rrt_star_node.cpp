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
 * RRT* benchmark planner: OMPL RRT* minimizing the path length, with the same state space, bounds and state validity
 * checker as the RRT initialization of ARENA. RRT* is anytime, it runs for rrt_star.time_budget seconds and returns
 * its best path. The nodes of the path are the control points of the NURBS curve evaluated like the trajectories of
 * ARENA, like the RRT paths that initialize ARENA.
 */

// Local
#include "common/benchmark_planner_node.h"
#include "arena_core/planning/OMPLPlanner.h"
#include "arena_core/geometry/ompl_state_validity_checker.h"

// External Libraries
// OMPL
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

// System
#include <chrono>
#include <memory>


namespace arena_benchmarks
{

class RRTStarNode : public BenchmarkPlannerNode
{
public:

    explicit RRTStarNode(const rclcpp::NodeOptions& a_options)
    : BenchmarkPlannerNode("linedrone_test_node", a_options)
    {
        // Same setup as the RRT of LinedroneTestNode, with RRT* and a path length objective
        ompl_planner_ = std::make_shared<arena_core::OMPLPlanner>();
        ompl_planner_->setProblemDimensions(3);
        ompl_planner_->getInitializer()->planner_ = std::make_shared<ompl::geometric::RRTstar>(ompl_planner_->getSpaceInformation());
        ompl_planner_->getInitializer()->state_validity_checker_ =
            std::make_shared<arena_core::OMPLStateValidityChecker>(ompl_planner_->getSpaceInformation(), costmap_mapping_);
        ompl_planner_->getInitializer()->optimization_objective_ =
            std::make_shared<ompl::base::PathLengthOptimizationObjective>(ompl_planner_->getSpaceInformation());
        ompl_planner_->getInitializer()->cost_to_go_heuristic_ = ompl::base::goalRegionCostToGo;
    }

protected:

    void plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result) override
    {
        const double range = param<double>("optimization.initialization.rrt_range", 5.0);
        const double time_budget = param<double>("rrt_star.time_budget", 1.0);
        const double speed = param<double>("benchmark.speed", linedrone_config_.robot_max_speed_);
        const double ramp_length = param<double>("benchmark.speed_ramp_length", 0.0);

        ompl_planner_->getInitializer()->planner_->as<ompl::geometric::RRTstar>()->setRange(range);
        ompl_planner_->setSolvingTimeout(time_budget);

        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        ompl_planner_->setBounds(bounds.row(0).transpose(), bounds.row(1).transpose());
        ompl_planner_->setStart(a_start);
        ompl_planner_->setGoal(a_goal);

        auto start_time = std::chrono::steady_clock::now();
        ompl::base::PlannerStatus status = ompl_planner_->plan();
        a_result.initialization_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        a_result.rrt_range = range;

        if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            RCLCPP_WARN(get_logger(), "RRT* found no path in %.2f s (%s)", time_budget, status.asString().c_str());
            return;
        }

        auto* path = ompl_planner_->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        Eigen::MatrixXd waypoints(3, path->getStateCount());
        for (size_t i = 0; i < path->getStateCount(); ++i)
        {
            const auto* state = path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            waypoints.col(i) = Eigen::Vector3d(state->values[0], state->values[1], state->values[2]);
        }
        RCLCPP_INFO(get_logger(), "RRT* path: %zu states, length %.2f m", path->getStateCount(), path->length());

        Eigen::MatrixXd control_points = scorer().toControlPoints(waypoints, speed, ramp_length);
        a_result.solutions.push_back(scorer().score(control_points));
        a_result.nb_of_control_points = static_cast<int>(control_points.cols());
    }

private:

    std::shared_ptr<arena_core::OMPLPlanner> ompl_planner_;

}; // class RRTStarNode

}; // namespace arena_benchmarks


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<arena_benchmarks::RRTStarNode>(options));
    rclcpp::shutdown();
    return 0;
}
