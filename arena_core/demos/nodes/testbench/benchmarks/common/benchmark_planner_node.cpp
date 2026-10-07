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

#include "common/benchmark_planner_node.h"

// ROS 2
#include "octomap_msgs/conversions.h"

// Local
#include "arena_core/math/algorithm/adaptive_voting_algorithm.h"

// External Libraries
// Pagmo
#include <pagmo/utils/multi_objective.hpp>

// System
#include <limits>


using namespace std::chrono_literals;


namespace arena_benchmarks
{

BenchmarkPlannerNode::BenchmarkPlannerNode(const std::string& a_name, const rclcpp::NodeOptions& a_options)
: Node(a_name, a_options), costmap_mapping_(std::make_shared<arena_demos::CostmapMapping>()), linedrone_config_(50)
{
    ros_namespace_ = this->get_namespace();
    if (ros_namespace_ != "/" && ros_namespace_.back() != '/')
        ros_namespace_ += "/";

    // ROS Publishers, same topics as LinedroneTestNode
    arena_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("arena_path", 10);
    solution_set_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("solution_set", 10);
    path_planning_finished_pub_ = this->create_publisher<std_msgs::msg::Bool>(ros_namespace_ + "path_planning_finished", 10);
    nurbs_infos_pub_ = this->create_publisher<arena_msgs::msg::OptimizerPathInfo>(ros_namespace_ + "nurbs_infos", 10);

    // ROS Subscriptions
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(ros_namespace_ + "goal_pose", 10,
        std::bind(&BenchmarkPlannerNode::goalPoseCallback, this, std::placeholders::_1));
    drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(ros_namespace_ + "drone_pose", 10,
        std::bind(&BenchmarkPlannerNode::dronePoseCallback, this, std::placeholders::_1));
    planning_activation_sub_ = this->create_subscription<std_msgs::msg::Bool>(ros_namespace_ + "planning_activation", 10,
        std::bind(&BenchmarkPlannerNode::planningActivationCallback, this, std::placeholders::_1));
    inflated_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/inflated_octomap/full", 10,
        std::bind(&BenchmarkPlannerNode::inflatedOctomapCallback, this, std::placeholders::_1));
    color_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/sdf_octomap/full", 10,
        std::bind(&BenchmarkPlannerNode::colorOctreeCallback, this, std::placeholders::_1));

    // Robot configuration, same parameters as LinedroneTestNode (linedrone_problem_params.yaml)
    linedrone_config_.robot_mass_ = param<double>("robot.mass", 20.0);
    linedrone_config_.robot_max_speed_ = param<double>("robot.max_speed", 0.5);
    linedrone_config_.robot_max_acceleration_ = param<double>("robot.max_acceleration", 2.2);
    linedrone_config_.robot_permanent_power_ascent_ = param<double>("robot.permanent_power.ascent", 3475.0);
    linedrone_config_.robot_permanent_power_roll_ = param<double>("robot.permanent_power.roll", 3102.0);
    linedrone_config_.robot_permanent_power_pitch_ = param<double>("robot.permanent_power.pitch", 3102.0);
    linedrone_config_.robot_permanent_power_descent_ = param<double>("robot.permanent_power.descent", 3080.0);
    linedrone_config_.solveQuadraticSurfaceCoefficients();
    updateScorer();

    // Main loop at 50 Hz, like LinedroneTestNode
    run_timer_ = this->create_wall_timer(20ms, std::bind(&BenchmarkPlannerNode::run, this));
}

std::vector<double> BenchmarkPlannerNode::getCostsWeights()
{
    return {param<double>("optimization.adaptive_costs_weights.time", 1.0),
            param<double>("optimization.adaptive_costs_weights.safety", 0.0),
            param<double>("optimization.adaptive_costs_weights.energy", 0.0)};
}

void BenchmarkPlannerNode::updateScorer()
{
    // The sample size can be changed by the testbench between two plannings
    unsigned int sample_size = static_cast<unsigned int>(param<int64_t>("optimization.sample_size", 50));
    int degree = static_cast<int>(param<int64_t>("benchmark.nurbs_degree", 5));
    if (scorer_ && scorer_->getSampleSize() == sample_size && scorer_->getDegree() == degree)
        return;

    linedrone_config_.base_config.sample_size = sample_size;
    scorer_ = std::make_shared<BaselinePathScorer>(costmap_mapping_, linedrone_config_, degree);
}

void BenchmarkPlannerNode::goalPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr a_msg)
{
    goal_point_ = Eigen::Vector3d(a_msg->point.x, a_msg->point.y, a_msg->point.z);
    goal_sent_ = true;
}

void BenchmarkPlannerNode::dronePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr a_msg)
{
    drone_position_ = Eigen::Vector3d(a_msg->point.x, a_msg->point.y, a_msg->point.z);
}

void BenchmarkPlannerNode::planningActivationCallback(const std_msgs::msg::Bool::SharedPtr a_msg)
{
    planning_activated_ = a_msg->data;

    // Only a deactivation cancels the goal (see LinedroneTestNode::planningActivationCallback)
    if (!planning_activated_)
        goal_sent_ = false;
}

void BenchmarkPlannerNode::inflatedOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr a_msg)
{
    auto octree = std::dynamic_pointer_cast<octomap::OcTree>(std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::fullMsgToMap(*a_msg)));
    if (!octree)
    {
        RCLCPP_ERROR(get_logger(), "Failed to convert Octomap message to OcTree.");
        return;
    }
    costmap_mapping_->setOctree(octree);
    octree_received_ = true;
}

void BenchmarkPlannerNode::colorOctreeCallback(const octomap_msgs::msg::Octomap::SharedPtr a_msg)
{
    auto color_octree = std::dynamic_pointer_cast<octomap::ColorOcTree>(std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::fullMsgToMap(*a_msg)));
    if (!color_octree)
    {
        RCLCPP_ERROR(get_logger(), "Failed to convert Octomap message to ColorOcTree.");
        return;
    }
    costmap_mapping_->setColorOctree(color_octree);
    color_octree_received_ = true;
}

void BenchmarkPlannerNode::run()
{
    // Both maps are needed: the inflated octomap for the collisions and the SDF octomap for the safety cost
    if (!octree_received_ || !color_octree_received_)
        return;

    if (!planning_activated_ || !goal_sent_)
        return;

    planning_start_time_ = std::chrono::steady_clock::now();
    updateScorer();

    PlanningResult result;
    try
    {
        plan(drone_position_, goal_point_, result);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Planning failed: %s", e.what());
        result.solutions.clear();
    }

    // Choose a trajectory among the feasible and safe ones with the adaptive voting algorithm, like ARENA
    std::vector<std::vector<double>> safe_fitness;
    std::vector<size_t> safe_indexes;
    for (size_t i = 0; i < result.solutions.size(); ++i)
    {
        if (result.solutions[i].feasible && result.solutions[i].safe)
        {
            safe_fitness.push_back(result.solutions[i].fitness);
            safe_indexes.push_back(i);
        }
    }

    int chosen_idx = -1;
    if (safe_indexes.size() == 1)
        chosen_idx = static_cast<int>(safe_indexes[0]);
    else if (safe_indexes.size() > 1)
        chosen_idx = static_cast<int>(safe_indexes[arena_core::adaptive_voting_algorithm::getBetterCandidateIndex(safe_fitness, getCostsWeights())]);

    RCLCPP_INFO(get_logger(), "Planning done: %zu solutions, %zu feasible and safe, %s", result.solutions.size(),
                safe_indexes.size(), chosen_idx >= 0 ? "a trajectory is chosen" : "no trajectory");

    planning_activated_ = false;
    goal_sent_ = false;

    publishPaths(result, chosen_idx);
    publishPlanningInfos(result, chosen_idx);
}

void BenchmarkPlannerNode::publishPlanningInfos(const PlanningResult& a_result, int a_chosen_idx)
{
    // Same report as LinedroneTestNode::publishPlanningInfos: the feasible solutions are the ones respecting the
    // constraints of the optimizer, only the safe ones can be chosen
    std::vector<std::vector<double>> feasible_fitness;
    std::vector<bool> safe;
    int chosen_feasible_idx = -1;
    for (size_t i = 0; i < a_result.solutions.size(); ++i)
    {
        if (!a_result.solutions[i].feasible)
            continue;
        if (static_cast<int>(i) == a_chosen_idx)
            chosen_feasible_idx = static_cast<int>(feasible_fitness.size());
        feasible_fitness.push_back(a_result.solutions[i].fitness);
        safe.push_back(a_result.solutions[i].safe);
    }

    const bool feasible = chosen_feasible_idx >= 0;
    const double max_cost = std::numeric_limits<double>::max();

    arena_msgs::msg::OptimizerPathInfo infos_msg;
    infos_msg.feasible = feasible;
    infos_msg.path.header.frame_id = "map";
    infos_msg.path.header.stamp = this->now();

    if (feasible)
    {
        const Eigen::MatrixXd& samples = a_result.solutions[a_chosen_idx].samples;
        for (int i = 0; i < samples.cols(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = infos_msg.path.header;
            pose.pose.position.x = samples(0, i);
            pose.pose.position.y = samples(1, i);
            pose.pose.position.z = samples(2, i);
            pose.pose.orientation.w = 1.0;
            infos_msg.path.poses.push_back(pose);
            infos_msg.velocities.push_back(samples(3, i));
        }
    }

    infos_msg.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - planning_start_time_).count();
    infos_msg.initialization_time = a_result.initialization_time;
    infos_msg.optimization_time = a_result.optimization_time;
    infos_msg.nb_of_control_points = a_result.nb_of_control_points;
    infos_msg.nb_of_generations = a_result.nb_of_generations;
    infos_msg.population_size = a_result.population_size;
    infos_msg.nurbs_sample_size = linedrone_config_.base_config.sample_size;
    infos_msg.rrt_range = a_result.rrt_range;

    std::vector<std::vector<double>> safe_fitness;
    for (size_t i = 0; i < feasible_fitness.size(); ++i)
    {
        if (safe[i])
            safe_fitness.push_back(feasible_fitness[i]);
    }
    infos_msg.nb_of_feasible_solutions = static_cast<int32_t>(feasible_fitness.size());
    infos_msg.nb_of_safe_solutions = static_cast<int32_t>(safe_fitness.size());

    infos_msg.best_time_cost = max_cost;
    infos_msg.best_security_cost = max_cost;
    infos_msg.best_energy_cost = max_cost;
    for (const auto& fitness : safe_fitness)
    {
        infos_msg.best_time_cost = std::min(infos_msg.best_time_cost, fitness[0]);
        infos_msg.best_security_cost = std::min(infos_msg.best_security_cost, fitness[1]);
        infos_msg.best_energy_cost = std::min(infos_msg.best_energy_cost, fitness[2]);
    }

    infos_msg.chosen_time_cost = feasible ? feasible_fitness[chosen_feasible_idx][0] : max_cost;
    infos_msg.chosen_security_cost = feasible ? feasible_fitness[chosen_feasible_idx][1] : max_cost;
    infos_msg.chosen_energy_cost = feasible ? feasible_fitness[chosen_feasible_idx][2] : max_cost;

    std::vector<double> weights = getCostsWeights();
    infos_msg.time_coefficient = weights[0];
    infos_msg.security_coefficient = weights[1];
    infos_msg.energy_coefficient = weights[2];

    // Non-dominated solutions (pagmo needs at least 2 points to sort them)
    auto firstFront = [](const std::vector<std::vector<double>>& fitness) -> std::vector<pagmo::pop_size_t>
    {
        if (fitness.size() < 2)
            return std::vector<pagmo::pop_size_t>(fitness.size(), 0);
        return std::get<0>(pagmo::fast_non_dominated_sorting(fitness))[0];
    };

    for (pagmo::pop_size_t idx : firstFront(feasible_fitness))
    {
        infos_msg.pareto_front_time_costs.push_back(feasible_fitness[idx][0]);
        infos_msg.pareto_front_security_costs.push_back(feasible_fitness[idx][1]);
        infos_msg.pareto_front_energy_costs.push_back(feasible_fitness[idx][2]);
        infos_msg.pareto_front_safe.push_back(safe[idx]);
    }

    for (pagmo::pop_size_t idx : firstFront(safe_fitness))
    {
        infos_msg.safe_front_time_costs.push_back(safe_fitness[idx][0]);
        infos_msg.safe_front_security_costs.push_back(safe_fitness[idx][1]);
        infos_msg.safe_front_energy_costs.push_back(safe_fitness[idx][2]);
    }

    nurbs_infos_pub_->publish(infos_msg);

    // Notify that the planning attempt is finished, its success is given by infos_msg.feasible
    std_msgs::msg::Bool finished_msg;
    finished_msg.data = true;
    path_planning_finished_pub_->publish(finished_msg);
}

void BenchmarkPlannerNode::publishPaths(const PlanningResult& a_result, int a_chosen_idx)
{
    auto makeMarker = [this](const Eigen::MatrixXd& samples, const std::string& ns, int id, float r, float g, float b, float a)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        for (int i = 0; i < samples.cols(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = samples(0, i);
            p.y = samples(1, i);
            p.z = samples(2, i);
            marker.points.push_back(p);
        }
        return marker;
    };

    // Feasible and safe solutions in white, the others in red
    visualization_msgs::msg::MarkerArray solution_set;
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    solution_set.markers.push_back(delete_all);
    for (size_t i = 0; i < a_result.solutions.size(); ++i)
    {
        const ScoredPath& solution = a_result.solutions[i];
        if (solution.samples.cols() == 0)
            continue;
        bool valid = solution.feasible && solution.safe;
        solution_set.markers.push_back(makeMarker(solution.samples, "solution_set", static_cast<int>(i),
                                                  1.0f, valid ? 1.0f : 0.0f, valid ? 1.0f : 0.0f, 0.5f));
    }
    solution_set_pub_->publish(solution_set);

    visualization_msgs::msg::MarkerArray chosen_path;
    chosen_path.markers.push_back(delete_all);
    if (a_chosen_idx >= 0)
        chosen_path.markers.push_back(makeMarker(a_result.solutions[a_chosen_idx].samples, "arena_path", 0, 0.0f, 1.0f, 0.0f, 1.0f));
    arena_path_pub_->publish(chosen_path);
}

}; // namespace arena_benchmarks
