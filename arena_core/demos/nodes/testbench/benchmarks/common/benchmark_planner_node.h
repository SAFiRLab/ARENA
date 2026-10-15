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

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "arena_msgs/msg/optimizer_path_info.hpp"

// Local
#include "linedrone/costmap_mapping.h"
#include "linedrone/linedrone_nurbs_analyzer.h"
#include "testbench/benchmarks/common/baseline_path_scorer.h"
#include "arena_core/math/algorithm/voting_algorithm.h"

// System
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

/**
 * @brief Solutions of a planning of a benchmark planner and the values reported to the testbench.
 */
struct PlanningResult
{
    std::vector<ScoredPath> solutions; // Every trajectory found by the planner, feasible or not
    double initialization_time = 0.0; // s, reported in the initialization time of the testbench (e.g. graph search)
    double optimization_time = 0.0;   // s, reported in the optimization time of the testbench (e.g. NSGA-II)
    int nb_of_control_points = 0;
    int nb_of_generations = 0;
    int population_size = 0;
    double rrt_range = 0.0; // Resolution of the planner written in the RRT range column (RRT range, waypoint spacing)
};


/**
 * @brief Base node of the benchmark planners compared to ARENA.
 *
 * It has the same ROS interface as LinedroneTestNode, the testbench drives it without any change when it is launched
 * with the name and namespace of LinedroneTestNode (benchmark_planner_launch.py):
 *     - subscriptions: goal_pose, drone_pose, planning_activation, /navigation/inflated_octomap/full,
 *       /navigation/sdf_octomap/full
 *     - publications: nurbs_infos (arena_msgs/OptimizerPathInfo), path_planning_finished, arena_path, solution_set
 *     - parameters: robot.*, optimization.sample_size, optimization.evaluation_sample_size, optimization.voting.*,
 *       optimization.adaptive_costs_weights.* (and the parameters of every planner)
 *
 * The planners only return their trajectories (plan()), this node evaluates them with BaselinePathScorer, chooses one
 * with the voting algorithm of ARENA (optimization.voting) among the feasible and safe ones and reports them like ARENA.
 */
class BenchmarkPlannerNode : public rclcpp::Node
{
public:

    BenchmarkPlannerNode(const std::string& a_name, const rclcpp::NodeOptions& a_options);
    ~BenchmarkPlannerNode() override = default;

protected:

    /**
     * @brief Plans from a_start to a_goal and fills a_result with the evaluated trajectories (see scorer()).
     * Called when the planning is activated, a goal has been received and both octomaps are available.
     */
    virtual void plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result) = 0;

    /**
     * @brief Value of a parameter, declared with a_default if it isn't set (yaml file or testbench).
     */
    template <typename T>
    T param(const std::string& a_name, const T& a_default)
    {
        if (!this->has_parameter(a_name))
            this->declare_parameter<T>(a_name, a_default);
        return this->get_parameter(a_name).get_value<T>();
    }

    /**
     * @brief Coefficients (time, safety, energy) of the voting: computed by the voting algorithm from the risks with
     * optimization.voting.use_risks, the fixed coefficients (getFixedCoefficients()) otherwise.
     */
    arena_core::ObjectiveCoefficients getCostsWeights();

    /** @brief Whether the coefficients are computed from the risks (optimization.voting.use_risks, default false). */
    bool votingUsesRisks();

    /** @brief Risks of the mission (optimization.voting.risks.*), clamped to [0, 1]. */
    arena_core::OperationalRisks getOperationalRisks();

    /** @brief Fixed coefficients of the tests (optimization.adaptive_costs_weights.*). */
    arena_core::ObjectiveCoefficients getFixedCoefficients();

    /** @brief Settings of the voting algorithm (optimization.voting.*). */
    arena_core::VotingAlgorithmSettings getVotingSettings();

    /** @brief Scorer of the trajectories, rebuilt when the sample size parameter changes. */
    BaselinePathScorer& scorer() { return *scorer_; }

    /**
     * @brief Evaluates a trajectory of the planner as it is (BaselinePathScorer::scoreTrajectory). The samples are spaced
     * by optimization.evaluation_spacing (< 0: half the resolution of the octomap, so a longer trajectory has more samples),
     * otherwise benchmark.evaluation_samples samples (default: optimization.evaluation_sample_size, 250 if not set).
     * The time of this evaluation isn't part of the planning time.
     * @param a_check_dynamics Check the acceleration (planners with dynamics only).
     */
    ScoredPath scoreTrajectory(const NativeTrajectory& a_trajectory, bool a_check_dynamics);

    std::shared_ptr<arena_demos::CostmapMapping> costmap_mapping_;
    arena_demos::LinedroneNurbsAnalyzerConfig linedrone_config_;

private:

    void run();
    void updateScorer();

    /** @brief Analyzer of the reported costs, rebuilt when optimization.evaluation_sample_size changes (null: 0). */
    void updateEvaluation();

    /**
     * @brief Trajectories reported to the testbench with optimization.evaluation_sample_size samples (tests only), like
     * LinedroneTestNode::evaluationFitness: the samples of every feasible solution are interpolated up to this size
     * (BaselinePathScorer::interpolateTrajectory) and its costs evaluated again (the constraints aren't checked again). The choice of the trajectory is made
     * before, with the costs of the planner.
     */
    void evaluateForReport(PlanningResult& a_result);
    void publishPlanningInfos(const PlanningResult& a_result, int a_chosen_idx);

    /**
     * @brief benchmark.status_file (analysis only): one line per planning with its completion (the planner returned at
     * least one trajectory reaching the goal) and the number of trajectories respecting the constraints, the safety
     * check, and both, and the number of safe trajectories respecting all the physical constraints of the robot (the
     * acceleration included, ScoredPath::physically_feasible).
     */
    void writeStatus(const PlanningResult& a_result, int a_chosen_idx);
    void publishPaths(const PlanningResult& a_result, int a_chosen_idx);

    // ROS Subscriptions Callbacks
    void goalPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr a_msg);
    void dronePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr a_msg);
    void planningActivationCallback(const std_msgs::msg::Bool::SharedPtr a_msg);
    void inflatedOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr a_msg);
    void colorOctreeCallback(const octomap_msgs::msg::Octomap::SharedPtr a_msg);

    // ROS Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arena_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr solution_set_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_planning_finished_pub_;
    rclcpp::Publisher<arena_msgs::msg::OptimizerPathInfo>::SharedPtr nurbs_infos_pub_;

    // ROS Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr drone_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planning_activation_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr inflated_octomap_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr color_octomap_sub_;

    rclcpp::TimerBase::SharedPtr run_timer_;

    std::shared_ptr<BaselinePathScorer> scorer_;
    std::shared_ptr<arena_demos::LinedroneNurbsAnalyzer> evaluation_analyzer_;
    unsigned int evaluation_sample_size_ = 0;
    double evaluation_time_ = 0.0; // s, evaluation of the reported costs (tests only), not part of the planning time
    double scoring_time_ = 0.0;    // s, scoreTrajectory() during plan(), not part of the planning time
    std::chrono::steady_clock::time_point planning_start_time_;
    std::chrono::steady_clock::time_point planning_end_time_; // After the choice of the trajectory
    int export_counter_ = 0; // Plannings written in optimization.export_solutions_file
    int status_counter_ = 0; // Plannings written in benchmark.status_file
    bool planning_activated_ = false;
    bool goal_sent_ = false;
    bool octree_received_ = false;
    bool color_octree_received_ = false;
    Eigen::Vector3d goal_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d drone_position_ = Eigen::Vector3d(95.367, 15.637, 6.376); // Same default start as LinedroneTestNode
    std::string ros_namespace_;

}; // class BenchmarkPlannerNode

}; // namespace arena_benchmarks
