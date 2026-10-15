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
#include "arena_core/math/algorithm/voting_algorithm.h"

// External Libraries
// Pagmo
#include <pagmo/utils/multi_objective.hpp>

// System
#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <fstream>


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

arena_core::ObjectiveCoefficients BenchmarkPlannerNode::getCostsWeights()
{
    if (!votingUsesRisks())
        return getFixedCoefficients();
    return arena_core::createVotingAlgorithm(getVotingSettings())->computeCoefficients(getOperationalRisks());
}

bool BenchmarkPlannerNode::votingUsesRisks()
{
    return param<bool>("optimization.voting.use_risks", false);
}

arena_core::OperationalRisks BenchmarkPlannerNode::getOperationalRisks()
{
    auto risk = [this](const std::string& name)
    { return std::clamp(param<double>("optimization.voting.risks." + name, 0.0), 0.0, 1.0); };
    return {risk("wind"), risk("communication"), risk("localization"), risk("battery")};
}

arena_core::ObjectiveCoefficients BenchmarkPlannerNode::getFixedCoefficients()
{
    return {param<double>("optimization.adaptive_costs_weights.time", 1.0),
            param<double>("optimization.adaptive_costs_weights.safety", 0.0),
            param<double>("optimization.adaptive_costs_weights.energy", 0.0)};
}

arena_core::VotingAlgorithmSettings BenchmarkPlannerNode::getVotingSettings()
{
    arena_core::VotingAlgorithmSettings settings;
    settings.type = arena_core::votingAlgorithmTypeFromString(param<std::string>("optimization.voting.algorithm",
                                                                                 "risk_weighted_tchebycheff"));
    settings.nominal_coefficients = {param<double>("optimization.voting.nominal_coefficients.time", 1.0 / 3.0),
                                     param<double>("optimization.voting.nominal_coefficients.safety", 1.0 / 3.0),
                                     param<double>("optimization.voting.nominal_coefficients.energy", 1.0 / 3.0)};
    settings.gain = param<double>("optimization.voting.risk_weighted_tchebycheff.gain", settings.gain);
    settings.augmentation = param<double>("optimization.voting.risk_weighted_tchebycheff.augmentation", settings.augmentation);
    return settings;
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

void BenchmarkPlannerNode::updateEvaluation()
{
    // Sample size of the costs reported to the testbench, 0: the costs of the planner
    unsigned int evaluation_sample_size = static_cast<unsigned int>(std::max<int64_t>(param<int64_t>("optimization.evaluation_sample_size", 0), 0));
    if (evaluation_sample_size == evaluation_sample_size_)
        return;

    evaluation_sample_size_ = evaluation_sample_size;
    evaluation_analyzer_ = nullptr;
    if (evaluation_sample_size_ == 0)
        return;

    arena_demos::LinedroneNurbsAnalyzerConfig evaluation_config = linedrone_config_;
    evaluation_config.base_config.sample_size = evaluation_sample_size_;
    evaluation_analyzer_ = std::make_shared<arena_demos::LinedroneNurbsAnalyzer>(costmap_mapping_, evaluation_config,
                                                                               std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>{});
    RCLCPP_INFO(get_logger(), "The costs of the reported solutions are evaluated with %u samples", evaluation_sample_size_);
}

void BenchmarkPlannerNode::evaluateForReport(PlanningResult& a_result)
{
    if (!evaluation_analyzer_)
        return;

    auto start_time = std::chrono::steady_clock::now();
    arena_core::EvalNurbsOutput output;
    output.fitness_size_ = 3;
    output.fitness_array_ = std::vector<double>(output.fitness_size_, std::numeric_limits<double>::max());
    output.constraint_size_ = 2;
    output.constraint_array_ = std::vector<double>(output.constraint_size_, std::numeric_limits<double>::max());

    for (ScoredPath& solution : a_result.solutions)
    {
        // The trajectories evaluated as they are (scoreTrajectory) already have their reported costs
        if (solution.native || !solution.feasible || solution.samples.cols() < 2 ||
            solution.samples.cols() >= static_cast<int>(evaluation_sample_size_))
            continue;

        // The acceleration of the interpolated trajectory is meaningless at the corners of the polyline, the segments
        // between the samples are checked for collisions by the safety check of the scorer
        solution.samples = BaselinePathScorer::interpolateTrajectory(solution.samples, static_cast<int>(evaluation_sample_size_));
        evaluation_analyzer_->eval(solution.samples, output);
        solution.fitness = output.fitness_array_;
    }
    evaluation_time_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
}

ScoredPath BenchmarkPlannerNode::scoreTrajectory(const NativeTrajectory& a_trajectory, bool a_check_dynamics)
{
    auto start_time = std::chrono::steady_clock::now();
    int nb_of_samples;
    const double spacing = costmap_mapping_->evaluationSpacing(param<double>("optimization.evaluation_spacing", 0.0));
    if (spacing > 0.0)
    {
        // Samples at the given spacing along the path (half the octomap resolution by default)
        nb_of_samples = std::max(3, static_cast<int>(std::ceil(trajectoryLength(a_trajectory) / spacing)) + 1);
    }
    else
    {
        int64_t evaluation_sample_size = param<int64_t>("optimization.evaluation_sample_size", 0);
        nb_of_samples = static_cast<int>(param<int64_t>("benchmark.evaluation_samples", evaluation_sample_size > 0 ? evaluation_sample_size : 250));
    }
    ScoredPath scored = scorer().scoreTrajectory(a_trajectory, a_check_dynamics, nb_of_samples);
    scoring_time_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    return scored;
}

void BenchmarkPlannerNode::writeStatus(const PlanningResult& a_result, int a_chosen_idx)
{
    const std::string status_file = param<std::string>("benchmark.status_file", "");
    if (status_file.empty())
        return;

    int nb_of_constrained = 0, nb_of_safe = 0, nb_of_valid = 0, nb_of_physical = 0;
    double min_acceleration = std::numeric_limits<double>::infinity();
    for (const ScoredPath& solution : a_result.solutions)
    {
        nb_of_constrained += solution.feasible;
        nb_of_safe += solution.safe;
        nb_of_valid += solution.feasible && solution.safe;
        nb_of_physical += solution.physically_feasible && solution.safe;
        min_acceleration = std::min(min_acceleration, solution.max_acceleration);
    }
    const bool chosen_physical = a_chosen_idx >= 0 && a_result.solutions[a_chosen_idx].physically_feasible;

    bool new_file = !std::ifstream(status_file).good();
    std::ofstream file(status_file, std::ios::out | std::ios::app);
    if (new_file)
        file << "Planning,Completed,Solutions,Constraints respected,Safe,Feasible and safe,Chosen,Planner time,"
                "Physically feasible and safe,Chosen physically feasible,Min max acceleration\n";
    file << status_counter_++ << "," << !a_result.solutions.empty() << "," << a_result.solutions.size() << ","
         << nb_of_constrained << "," << nb_of_safe << "," << nb_of_valid << "," << (a_chosen_idx >= 0) << ","
         << std::fixed << std::setprecision(6) << a_result.initialization_time + a_result.optimization_time << ","
         << nb_of_physical << "," << chosen_physical << "," << min_acceleration << "\n";
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
    evaluation_time_ = 0.0;
    scoring_time_ = 0.0;
    updateScorer();
    updateEvaluation();

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
    {
        // Same voting algorithm as ARENA (optimization.voting)
        const int voted_idx = arena_core::createVotingAlgorithm(getVotingSettings())->selectCandidate(safe_fitness, getCostsWeights());
        chosen_idx = static_cast<int>(safe_indexes[voted_idx]);
    }

    // End of the planning: the publications, the evaluation of the reported costs and the exports aren't part of it
    planning_end_time_ = std::chrono::steady_clock::now();

    // Costs used by the voting, kept for optimization.export_solutions_file (the reported costs are evaluated below)
    std::vector<std::vector<double>> vote_fitness = safe_fitness;

    RCLCPP_INFO(get_logger(), "Planning done: %zu solutions, %zu feasible and safe, %s", result.solutions.size(),
                safe_indexes.size(), chosen_idx >= 0 ? "a trajectory is chosen" : "no trajectory");

    planning_activated_ = false;
    goal_sent_ = false;

    publishPaths(result, chosen_idx);

    // The costs and the chosen trajectory reported to the testbench with optimization.evaluation_sample_size samples
    evaluateForReport(result);

    // optimization.export_solutions_file (analysis only): every feasible and safe solution with the costs used by the voting,
    // the reported costs and the clearance of its reported trajectory (evaluation sample size), like LinedroneTestNode
    const std::string export_file = param<std::string>("optimization.export_solutions_file", "");
    if (!export_file.empty())
    {
        bool new_file = !std::ifstream(export_file).good();
        std::ofstream file(export_file, std::ios::out | std::ios::app);
        if (new_file)
            file << "Planning,Solution,Vote time cost,Vote security cost,Vote energy cost,Time cost,Security cost,Energy cost,"
                    "Average Closest Obstacle Distance,Min Closest Obstacle Distance\n";
        for (size_t k = 0; k < safe_indexes.size(); ++k)
        {
            const ScoredPath& solution = result.solutions[safe_indexes[k]];
            double sum = 0.0, min_distance = std::numeric_limits<double>::max();
            for (int c = 0; c < solution.samples.cols(); ++c)
            {
                double d = costmap_mapping_->closestObstacleDistance(solution.samples.block<3, 1>(0, c));
                sum += d;
                min_distance = std::min(min_distance, d);
            }
            file << std::fixed << std::setprecision(6) << export_counter_ << "," << safe_indexes[k] << ","
                 << vote_fitness[k][0] << "," << vote_fitness[k][1] << "," << vote_fitness[k][2] << ","
                 << solution.fitness[0] << "," << solution.fitness[1] << "," << solution.fitness[2] << ","
                 << sum / solution.samples.cols() << "," << min_distance << "\n";
        }
        RCLCPP_INFO(get_logger(), "%zu safe solutions exported to %s", safe_indexes.size(), export_file.c_str());
        export_counter_++;
    }

    writeStatus(result, chosen_idx);
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

    // Planning time: from the activation to the choice of the trajectory, without the evaluation of the trajectories
    infos_msg.planning_time = std::chrono::duration<double>(planning_end_time_ - planning_start_time_).count() - scoring_time_;
    RCLCPP_INFO(get_logger(), "Planning time %.1f ms, evaluation of the trajectories %.1f ms (not in the planning time)",
                infos_msg.planning_time * 1.0e3, (scoring_time_ + evaluation_time_) * 1.0e3);
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

    const arena_core::ObjectiveCoefficients weights = getCostsWeights();
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
