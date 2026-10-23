/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2025, David-Alexandre Poissant, Université de Sherbrooke
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

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "arena_msgs/msg/optimizer_path_info.hpp"

// Local
#include "arena_core/math/nurbs.h"
#include "arena_core/math/control_point.h"
#include "arena_core/planning/OMPLPlanner.h"
#include "arena_core/geometry/ompl_state_validity_checker.h"
#include "arena_core/math/algorithm/voting_algorithm.h"
#include "linedrone/linedrone_nurbs_analyzer.h"
#include "linedrone/linedrone_problem.hpp"
#include "linedrone/costmap_mapping.h"
#include "linedrone/sdf_state_validity_checker.h"
#include "testbench/benchmarks/moar_3d/weighted_astar_3d.h"

// External Libraries
// Eigen
#include <Eigen/Dense>
// Pagmo
#include <pagmo/algorithms/nsga2.hpp>
#include <pagmo/utils/multi_objective.hpp>
// OMPL
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

// System
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <fstream>
#include <limits>


using namespace std::chrono_literals;


struct GoalStatus
{
    bool goal_sent_ = false;
    Eigen::Vector3d goal_;
}; // struct GoalStatus


class LinedroneTestNode : public rclcpp::Node
{
public:

    LinedroneTestNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~LinedroneTestNode() override = default;

    /******************* Callbacks *******************/
    /** @brief Callback for the timer that runs the main loop of the node.
     * 
     * This function is called periodically to execute the main logic of the node.
     */
    void run();
    
    /** @brief Callback for the fitness function used in the optimization process.
     * 
     * This function calculates the fitness of a given solution based on the current state of the Linedrone.
     * 
     * @param dv The decision vector representing the solution to evaluate.
     * @return A vector containing the fitness values for the solution.
     */
    pagmo::vector_double linedroneFitness(const pagmo::vector_double& dv);

    /** @brief Callback to publish the solution set at every generation of the optimization process.
     * 
     * This function publishes the solution set of the optimization process as a visualization marker.
     * 
     * @param pop The population containing the solutions to publish.
     */
    void publishSolutionSet(const pagmo::population& pop);

private:

    /******************* Getters *******************/
    /** @brief Checks if the current path planned path is safe.
     * 
     * This function checks if the current path is safe by checking for collisions with the environment.
     * 
     * @return true if the path is safe, false otherwise.
     */
    bool isCurrentPathSafe() const;

    /** @brief True if no segment of the path crosses an occupied voxel of the inflated octomap.
     * Ray tracing between every pair of consecutive samples, the optimizer only checks the samples themselves.
     * @param path Samples of the trajectory (rows x, y, z, ...).
     */
    bool isPathSafe(const Eigen::MatrixXd& path) const;

    /******************* User-Defined methods *******************/
    /** @brief Converts a pagmo vector_double to a vector of ControlPoint objects.
     * 
     * This function converts a pagmo vector_double to a vector of ControlPoint objects for further processing.
     * 
     * @param dv The pagmo vector_double to convert.
     * @return A vector of ControlPoint objects.
     */
    std::vector<arena_core::ControlPoint<double, 4>> pagmoDVToControlPoints(const pagmo::vector_double& dv) const;

    Eigen::MatrixXd addControlPointsToPath(const Eigen::MatrixXd& initial_control_points, const int nb_of_control_points) const;

    /** @brief Samples of a trajectory interpolated up to nb_of_samples samples, for the evaluation of the reported costs.
     * 
     * The points are inserted at equal distance on every segment between two samples (addControlPointsToPath). The speed
     * follows a constant acceleration on every segment (squared speed linear in the distance), the motion model of the
     * time cost (2 d / (v1 + v2) per segment): the time of the trajectory is unchanged. A linear speed would take longer
     * near a null speed (start and goal) and penalize the trajectories accelerating over few samples.
     * 
     * @param path Samples of the trajectory (rows x, y, z, speed).
     */
    Eigen::MatrixXd interpolateTrajectory(const Eigen::MatrixXd& path, const int nb_of_samples) const;

    /** @brief Costs of a trajectory evaluated with optimization.evaluation_sample_size samples (tests only).
     *
     * With few samples, the optimizer only sees the costs at the samples: a trajectory whose samples are far from the
     * obstacles gets a good safety cost even if the segments between them pass close to an obstacle. The samples are
     * interpolated (interpolateTrajectory: points at equal distance on every segment between two samples, constant
     * acceleration) up to the evaluation sample size and the costs are evaluated again, so that the Pareto fronts of
     * every sample size can be compared to the reference front.
     *
     * @param path Samples of the trajectory (rows x, y, z, velocity).
     * @param fitness Costs given by the optimizer, returned unchanged when the evaluation is disabled or the trajectory
     *                already has at least evaluation_sample_size samples.
     * @return The time, safety and energy costs of the interpolated trajectory.
     */
    pagmo::vector_double evaluationFitness(const Eigen::MatrixXd& path, const pagmo::vector_double& fitness);

    /**
     * @brief Final evaluation of a solution at a resolution of the map (optimization.evaluation_spacing, tests only): the
     * NURBS curve of the solution sampled every a_spacing meters along it (a longer trajectory has more samples), its
     * costs, its constraints (acceleration, occupancy) and the ray tracing between its samples. Not part of the planning
     * time and not used by the optimizer nor the voting: only the reports use it.
     *
     * Time and energy: instantaneous start and stop, the first segment of the optimizer is flown at the speed of its second
     * sample and the last one at the speed of its second last sample (the speed of the NURBS curve goes to 0 at the ends:
     * its travel time would grow without bound with the resolution), the NURBS speed elsewhere, constant acceleration
     * between two samples. The acceleration constraint is checked on the NURBS speeds (the instantaneous start and stop
     * only account for the time and the energy).
     *
     * @param a_dv Decision vector of the solution.
     * @param a_samples Samples of the curve (rows x, y, z, velocity).
     * @param a_fitness Time, safety and energy costs of the samples.
     * @return True if the samples respect the constraints and no segment between them crosses an occupied voxel.
     */
    bool resolutionEvaluation(const pagmo::vector_double& a_dv, double a_spacing, Eigen::MatrixXd& a_samples,
                              pagmo::vector_double& a_fitness);

    /** @brief Uses ompl_wrapper to plan initial paths for the ARENA optimization process
     * 
     * This function initializes the planning process for the Linedrone by generating initial paths, therefore generating intial control points.
     */
    void initializerPlanning();

    /** @brief Runs the ARENA optimization process.
     * 
     * This function executes the ARENA optimization algorithm to find the best path for the Linedrone according to the defined fitness function
     * and adaptive costs weights.
     */
    void ARENAOptimization();

    /** @brief Reads the optimization hyperparameters that can be changed between plannings.
     * 
     * This function updates the population size and the NURBS sample size from the ROS parameters.
     */
    void updateOptimizationParameters();

    /** @brief Builds the voting algorithm and computes its coefficients from the parameters (optimization.voting).
     *
     * optimization.voting.algorithm selects the voting algorithm (risk_weighted_tchebycheff or rank). With
     * optimization.voting.use_risks, the coefficients of the objectives are computed by the algorithm from the risks
     * (optimization.voting.risks.*), otherwise they are optimization.adaptive_costs_weights.* (fixed coefficients of the
     * tests).
     */
    void updateVotingAlgorithm();

    /** @brief Publishes the report of a planning attempt and notifies that the attempt is finished.
     * 
     * Every planning attempt is reported, feasible or not, so that the testbench can evaluate the feasibility rate.
     * 
     * @param feasible_fitness The fitness of the solutions respecting the constraints at the end of the optimization
     *                         (evaluated with optimization.evaluation_sample_size samples when it is set).
     * @param chosen_idx The index of the chosen solution in feasible_fitness, or -1 if no feasible solution has been found.
     */
    void publishPlanningInfos(const std::vector<pagmo::vector_double>& feasible_fitness, const std::vector<bool>& safe, int chosen_idx);

    /** @brief Publishes the report of a planning attempt that didn't find any feasible solution.
     */
    void publishInfeasiblePlanning()
    {
        planning_end_time_ = std::chrono::steady_clock::now();
        publishPlanningInfos({}, {}, -1);
    }

    // ROS Timers
    rclcpp::TimerBase::SharedPtr run_timer_;
    rclcpp::TimerBase::SharedPtr octomap_timer_;

    // ROS Timers Private Callbacks
    /** @brief Callback to publish the arena path as a visualization marker.
     * 
     * This function publishes the arena path as a visualization marker in the ROS environment.
     */
    void publishARENAPath();

    /** @brief Callback to publish the control points of the NURBS curve.
     * 
     * This function publishes the control points of the NURBS curve as a visualization marker in the ROS environment.
     */
    void publishControlPoints();

    /** @brief Callback to publish the OMPL planner paths.
     * 
     * This function publishes the paths generated by the OMPL planner as a visualization marker array in the ROS environment.
     */
    void publishOMPLPlannerPaths();

    // ROS Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arena_control_points_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arena_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ompl_planner_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr solution_set_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_planning_finished_pub_;
    rclcpp::Publisher<arena_msgs::msg::OptimizerPathInfo>::SharedPtr nurbs_infos_pub_;

    // ROS Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr drone_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planning_activation_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr inflated_octomap_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr color_octomap_sub_;

    // ROS Subscriptions Callbacks
    /** @brief Callback for the goal pose subscription.
     * This function is called when a new goal pose is received.
     * @param msg The message containing the goal pose.
     */
    void goalPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /** @brief Callback for the drone pose subscription.
     * This function is called when a new drone position is received. It is used as the start of the next planning.
     * @param msg The message containing the drone position.
     */
    void dronePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /** @brief Callback for the planning activation subscription.
     * This function is called when a planning activation message is received.
     * @param msg The message containing the planning activation status.
     */
    void planningActivationCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /** @brief Callback for the inflated octomap subscription.
     * This function is called when a new inflated octomap is received.
     * @param msg The message containing the inflated octomap.
     */
    void inflatedOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

    /** @brief Callback for the color octomap subscription.
     * This function is called when a new color octomap is received.
     * @param msg The message containing the color octomap.
     */
    void colorOctreeCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);


    /****************** User-Defined Attributes *******************/
    arena_demos::LinedroneNurbsAnalyzerConfig linedrone_config;
    arena_core::EvalNurbsOutput linedrone_output_;
    std::shared_ptr<arena_demos::LinedroneNurbsAnalyzer> linedrone_nurbs_analyzer_;
    // Costs reported to the testbench with optimization.evaluation_sample_size samples, null when it isn't set
    std::shared_ptr<arena_demos::LinedroneNurbsAnalyzer> evaluation_nurbs_analyzer_;
    unsigned int evaluation_sample_size_ = 0;
    std::shared_ptr<arena_demos::CostmapMapping> costmap_mapping_;
    std::shared_ptr<arena_core::Nurbs<4>> nurbs_;
    std::shared_ptr<Eigen::MatrixXd> arena_path_;
    Eigen::MatrixXd reported_path_; // Samples of the chosen trajectory of the final evaluation (optimization.evaluation_spacing)
    std::chrono::steady_clock::time_point planning_end_time_; // After the choice of the trajectory
    bool planning_activated_ = false;
    bool path_planned_ = false;
    bool octree_received_ = false;
    bool color_octree_received_ = false;
    GoalStatus planning_goal_;

    // For the OMPL planner
    std::shared_ptr<arena_core::OMPLPlanner> ompl_planner_;
    std::vector<Eigen::MatrixXd> initial_paths_;
    Eigen::Vector3d start_point_;
    Eigen::Vector3d drone_position_ = Eigen::Vector3d(95.367, 15.637, 6.376); // Start of the planning until a drone pose is received
    Eigen::Vector3d goal_point_;

    // For optimization
    size_t population_size_ = 0;
    std::chrono::steady_clock::time_point planning_start_time_;
    double initialization_time_ = 0.0; // s
    double optimization_time_ = 0.0;   // s
    // optimization.early_exit_on_constraint_violation (on by default): the analyzer of the optimizer stops at the first
    // constraint violation. State of the current analyzer, built without it in the constructor
    bool early_exit_ = false;
    // optimization.verify_early_exit: every fitness is also evaluated without early exit and compared (tests only)
    bool verify_early_exit_ = false;
    std::shared_ptr<arena_demos::LinedroneNurbsAnalyzer> reference_nurbs_analyzer_;
    size_t early_exit_mismatches_ = 0, early_exit_checks_ = 0;
    int export_counter_ = 0;           // Plannings written in optimization.export_solutions_file

    // Voting algorithm selecting the solution and its coefficients (time, safety, energy) for the current planning
    std::unique_ptr<arena_core::IVotingAlgorithm> voting_algorithm_;
    arena_core::ObjectiveCoefficients voting_coefficients_ = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
    double evaluation_time_ = 0.0;     // s, evaluation of the reported costs (tests only), not part of the planning time
    int nb_of_control_points_ = 0;
    
    // ROS namespace
    std::string ros_namespace_ = "linedrone_test_node";

}; // LinedroneTestNode


// ROS subscriptions callback implementations
void LinedroneTestNode::goalPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    planning_goal_.goal_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    planning_goal_.goal_sent_ = true;

    ompl_planner_->setGoal(planning_goal_.goal_);
    goal_point_ = planning_goal_.goal_;
}

void LinedroneTestNode::dronePoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    drone_position_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
}

void LinedroneTestNode::planningActivationCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    planning_activated_ = msg->data;

    // Only a deactivation cancels the goal. The goal and the activation can be processed in any order when they
    // are received while the planner is busy (the executor handles the queued messages topic by topic), resetting the
    // goal on activation would then lose it and the planner would wait forever.
    if (!planning_activated_)
        planning_goal_.goal_sent_ = false;
}

void LinedroneTestNode::inflatedOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    auto octree = std::dynamic_pointer_cast<octomap::OcTree>(std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::fullMsgToMap(*msg)));

    if (!octree)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to OcTree.");
        return;
    }

    // Set the octree in the costmap mapping
    if (costmap_mapping_)
    {
        costmap_mapping_->setOctree(octree);
        octree_received_ = true;
    }
}

void LinedroneTestNode::colorOctreeCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    auto color_octree = std::dynamic_pointer_cast<octomap::ColorOcTree>(std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::fullMsgToMap(*msg)));
    
    if (!color_octree)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to ColorOcTree.");
        return;
    }

    // Set the color octree in the costmap mapping
    if (costmap_mapping_)
    {
        costmap_mapping_->setColorOctree(color_octree);
        color_octree_received_ = true;
    }
}


// ROS publishers implementations
void LinedroneTestNode::publishARENAPath()
{
    if (!arena_path_ || arena_path_->cols() == 0)
        return;

    visualization_msgs::msg::MarkerArray arena_markers;
    visualization_msgs::msg::Marker path_marker, velocities_marker;

    path_marker.header.frame_id = "map";
    path_marker.header.stamp = this->now();
    path_marker.ns = "arena_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.1; // Width of the line
    path_marker.color.r = 0.0f;
    path_marker.color.g = 1.0f; // Green color for the path
    path_marker.color.b = 0.0f;
    path_marker.color.a = 1.0f; // Fully opaque

    velocities_marker.header = path_marker.header;
    velocities_marker.ns = "arena_velocities";
    velocities_marker.id = 1;
    velocities_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    velocities_marker.action = visualization_msgs::msg::Marker::ADD;
    velocities_marker.scale.x = 0.5; // Diameter of the spheres
    velocities_marker.scale.y = 0.5; // Diameter of the spheres
    velocities_marker.scale.z = 0.5; // Diameter of the spheres

    for (int i = 0; i < arena_path_->cols(); ++i)
    {
        geometry_msgs::msg::Point p;
        p.x = (*arena_path_)(0, i);
        p.y = (*arena_path_)(1, i);
        p.z = (*arena_path_)(2, i);

        path_marker.points.push_back(p);

        // Create a sphere for the velocity at this point
        visualization_msgs::msg::Marker velocity_sphere;
        velocity_sphere.header = velocities_marker.header;
        velocity_sphere.ns = "velocity_spheres";
        velocity_sphere.id = i;
        velocity_sphere.type = visualization_msgs::msg::Marker::SPHERE;
        velocity_sphere.action = visualization_msgs::msg::Marker::ADD;
        velocity_sphere.pose.position = p;
        velocity_sphere.pose.orientation.w = 1.0; // No rotation
        velocity_sphere.color.r = fabs((*arena_path_)(3, i) / linedrone_config.robot_max_speed_);
        velocity_sphere.color.g = 1.0 - fabs((*arena_path_)(3, i) / linedrone_config.robot_max_speed_);
        velocity_sphere.color.b = 0.0f;
        velocity_sphere.color.a = 1.0f; // Fully opaque

        velocities_marker.points.push_back(p);
        velocities_marker.colors.push_back(velocity_sphere.color);
    }
    arena_markers.markers.push_back(path_marker);
    arena_markers.markers.push_back(velocities_marker);

    arena_path_pub_->publish(arena_markers);
}

void LinedroneTestNode::publishControlPoints()
{
    if (!nurbs_)
        return;

    visualization_msgs::msg::Marker control_points_marker;
    control_points_marker.header.frame_id = "map";
    control_points_marker.header.stamp = this->now();
    control_points_marker.ns = "control_points";
    control_points_marker.id = 0;
    control_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    control_points_marker.action = visualization_msgs::msg::Marker::ADD;
    control_points_marker.scale.x = 0.5; // Diameter of the spheres
    control_points_marker.scale.y = 0.5; // Diameter of the spheres
    control_points_marker.scale.z = 0.5; // Diameter of the spheres
    control_points_marker.color.r = 0.0f; // Red color for the control points
    control_points_marker.color.g = 0.0f;
    control_points_marker.color.b = 1.0f;
    control_points_marker.color.a = 1.0f; // Fully opaque

    for (const auto& cp : nurbs_->getControlPoints())
    {
        geometry_msgs::msg::Point p;
        p.x = cp.getValues()(0);
        p.y = cp.getValues()(1);
        p.z = cp.getValues()(2);

        control_points_marker.points.push_back(p);
        control_points_marker.colors.push_back(control_points_marker.color);
    }

    arena_control_points_pub_->publish(control_points_marker);
}

void LinedroneTestNode::publishOMPLPlannerPaths()
{
    if (!ompl_planner_)
        return;

    if (initial_paths_.empty())
        return;

    visualization_msgs::msg::MarkerArray initial_paths_markers;

    for (size_t i = 0; i < initial_paths_.size(); ++i)
    {
        const auto& path = initial_paths_[i];
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->now();
        path_marker.ns = "ompl_planner_path_";
        path_marker.id = i;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.1; // Width of the line
        path_marker.color.r = 1.0f;
        path_marker.color.g = 1.0f;
        path_marker.color.b = 1.0f;
        path_marker.color.a = 1.0f; // Fully opaque

        for (int j = 0; j < path.cols(); ++j)
        {
            geometry_msgs::msg::Point p;
            p.x = path(0, j);
            p.y = path(1, j);
            p.z = path(2, j);

            path_marker.points.push_back(p);
        }

        initial_paths_markers.markers.push_back(path_marker);
    }

    ompl_planner_pub_->publish(initial_paths_markers);
}

void LinedroneTestNode::publishSolutionSet(const pagmo::population& pop)
{
    // Publish solution set as a marker array of paths
    visualization_msgs::msg::MarkerArray solution_set_marker_array;

    // Get the max fitness values and store them for normalization and path coloring
    double max_time = 0.0;
    double min_time = std::numeric_limits<double>::max();
    double max_safety = 0.0;
    double min_safety = std::numeric_limits<double>::max();
    double max_energy = 0.0;
    double min_energy = std::numeric_limits<double>::max();
    for (int i = 0; i < pop.size(); i++)
    {
        pagmo::vector_double fitness = pop.get_f()[i];
        if (fitness[0] > max_time)
            max_time = fitness[0];
        if (fitness[0] < min_time)
            min_time = fitness[0];

        if (fitness[1] > max_safety)
            max_safety = fitness[1];
        if (fitness[1] < min_safety)
            min_safety = fitness[1];

        if (fitness[2] > max_energy)
            max_energy = fitness[2];
        if (fitness[2] < min_energy)
            min_energy = fitness[2];
    }

    double min_value = 0.0;
    double max_value = std::max({max_time, max_safety, max_energy});

    // Create lambda function to rescale the fitness values to [min_value, max_value] according to the fitness function's own max value
    auto rescale_time = [min_value, max_value, min_time, max_time](double value) -> double
    {
        /*if (max_time == min_time)
            return min_value;*/
        //return min_value + (max_value - min_value) * (value - min_time) / (max_time - min_time);
        return (value - min_time) / (max_time - min_time);
    };

    auto rescale_safety = [min_value, max_value, min_safety, max_safety](double value) -> double
    {
        /*if (max_safety == min_safety)
            return min_value;*/
        //return min_value + (max_value - min_value) * (value - min_safety) / (max_safety - min_safety);
        return (value - min_safety) / (max_safety - min_safety);
    };

    auto rescale_energy = [min_value, max_value, min_energy, max_energy](double value) -> double
    {
        /*if (max_energy == min_energy)
            return min_value;*/
        //return min_value + (max_value - min_value) * (value - min_energy) / (max_energy - min_energy);
        return (value - min_energy) / (max_energy - min_energy);
    };

    for (int i = 0; i < pop.size(); i++)
    {
        visualization_msgs::msg::Marker pose_marker;
        pose_marker.header.frame_id = "map"; 
        pose_marker.header.stamp = this->now();
        pose_marker.ns = "solution_set";
        pose_marker.id = i;
        pose_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        pose_marker.action = visualization_msgs::msg::Marker::ADD;

        std::vector<arena_core::ControlPoint<double, 4>> control_points = pagmoDVToControlPoints(pop.get_x()[i]);
        nurbs_->setControlPoints(control_points);
        Eigen::MatrixXd curve_points = nurbs_->evaluate();
        
        for (int j = 0; j < curve_points.cols(); j++)
        {
            geometry_msgs::msg::Point a_point;
            a_point.x = curve_points(0, j);
            a_point.y = curve_points(1, j);
            a_point.z = curve_points(2, j);
            pose_marker.points.push_back(a_point);
        }

        pose_marker.scale.x = 0.3;
        pose_marker.scale.y = 0.3;
        pose_marker.scale.z = 0.3;
        pose_marker.color.a = 0.5;
        
        pagmo::vector_double fitness = pop.get_f()[i];
        // Set color based on fitness values (red:=time, green:=safety, blue:=curvature)
        // The lower the fitness value, the more intense the color
        pose_marker.color.r = 1 - rescale_time(fitness[0]);// / max_value;
        if (std::isinf(pose_marker.color.r) || std::isnan(pose_marker.color.r) || std::isinf(-pose_marker.color.r) || std::isnan(-pose_marker.color.r))
            pose_marker.color.r = 1.0;
        pose_marker.color.g = 1 - rescale_safety(fitness[1]);// / max_value;
        if (std::isinf(pose_marker.color.g) || std::isnan(pose_marker.color.g) || std::isinf(-pose_marker.color.g) || std::isnan(-pose_marker.color.g))
            pose_marker.color.g = 1.0;
        pose_marker.color.b = 1 - rescale_energy(fitness[2]);// / max_value;
        if (std::isinf(pose_marker.color.b) || std::isnan(pose_marker.color.b) || std::isinf(-pose_marker.color.b) || std::isnan(-pose_marker.color.b))
            pose_marker.color.b = 1.0;

        solution_set_marker_array.markers.push_back(pose_marker);
    }

    solution_set_pub_->publish(solution_set_marker_array);
}


std::vector<arena_core::ControlPoint<double, 4>> LinedroneTestNode::pagmoDVToControlPoints(const pagmo::vector_double& dv) const
{
    std::vector<arena_core::ControlPoint<double, 4>> control_points;
    // The constructor is ControlPoint(values, id, weight): the id must be given for the weight to be used
    unsigned int next_id = 0;

    // Push first control point
    Eigen::VectorXd start(4);
    start << start_point_(0), start_point_(1), start_point_(2), 0.0; // Assuming the 4th dimension is velocity
    control_points.push_back(arena_core::ControlPoint<double, 4>(start, next_id++, dv[0]));

    for (int i = 1; i < dv.size() - 1; i += 5)
    {
        Eigen::VectorXd cp(4);
        cp << dv[i], dv[i + 1], dv[i + 2], dv[i + 3]; // Assuming the 4th dimension is velocity
        control_points.push_back(arena_core::ControlPoint<double, 4>(cp, next_id++, dv[i + 4]));
    }

    // Push the last control point
    Eigen::VectorXd end(4);
    end << goal_point_(0), goal_point_(1), goal_point_(2), 0.0; // Assuming the 4th dimension is velocity
    control_points.push_back(arena_core::ControlPoint<double, 4>(end, next_id, dv[dv.size() - 1]));

    return control_points;
}

Eigen::MatrixXd LinedroneTestNode::addControlPointsToPath(const Eigen::MatrixXd& initial_control_points, const int nb_of_control_points) const
{
    int initial_size = initial_control_points.cols();

    if (initial_size < 2)
        throw std::invalid_argument("Initial control points must have at least 2 columns.");
    if (nb_of_control_points < initial_size)
        throw std::invalid_argument("New number of control points must be >= initial number.");
    if (nb_of_control_points == initial_size)
        return initial_control_points;

    int dim = initial_control_points.rows();
    Eigen::MatrixXd new_path(dim, nb_of_control_points);

    // Total segments between initial control points
    int segments = initial_size - 1;
    int points_to_add = nb_of_control_points - initial_size;

    // Compute how many points to insert between each segment
    std::vector<int> insert_counts(segments, 0);
    for (int i = 0; i < points_to_add; ++i)
        insert_counts[i % segments] += 1; // Distribute extra points as evenly as possible

    int new_index = 0;
    for (int i = 0; i < segments; ++i)
    {
        Eigen::VectorXd start = initial_control_points.col(i);
        Eigen::VectorXd end = initial_control_points.col(i + 1);

        // Always add the start point
        new_path.col(new_index++) = start;

        int inserts = insert_counts[i];
        for (int j = 1; j <= inserts; ++j)
        {
            double alpha = static_cast<double>(j) / (inserts + 1);
            Eigen::VectorXd interp = (1.0 - alpha) * start + alpha * end;
            new_path.col(new_index++) = interp;
        }
    }

    // Finally add the last original point
    new_path.col(new_index++) = initial_control_points.col(initial_size - 1);

    return new_path;
}

Eigen::MatrixXd LinedroneTestNode::interpolateTrajectory(const Eigen::MatrixXd& path, const int nb_of_samples) const
{
    Eigen::MatrixXd squared_speed_path = path;
    squared_speed_path.row(3) = path.row(3).array().square();
    Eigen::MatrixXd interpolated = addControlPointsToPath(squared_speed_path, nb_of_samples);
    interpolated.row(3) = interpolated.row(3).array().max(0.0).sqrt();
    return interpolated;
}

pagmo::vector_double LinedroneTestNode::evaluationFitness(const Eigen::MatrixXd& path, const pagmo::vector_double& fitness)
{
    if (!evaluation_nurbs_analyzer_ || path.cols() >= static_cast<int>(evaluation_sample_size_))
        return fitness;

    auto start_time = std::chrono::steady_clock::now();

    // Only the costs are kept: the constraints were checked by the optimizer on its own samples, and the acceleration
    // of the interpolated trajectory is meaningless at the corners of the polyline (direction change over a tiny segment).
    // The segments between the samples are checked for collisions by isPathSafe().
    arena_core::EvalNurbsOutput output = linedrone_output_;
    evaluation_nurbs_analyzer_->eval(interpolateTrajectory(path, evaluation_sample_size_), output);
    evaluation_time_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    return output.fitness_array_;
}

bool LinedroneTestNode::resolutionEvaluation(const pagmo::vector_double& a_dv, double a_spacing, Eigen::MatrixXd& a_samples,
                                             pagmo::vector_double& a_fitness)
{
    auto start_time = std::chrono::steady_clock::now();
    const int optimizer_sample_size = static_cast<int>(linedrone_config.base_config.sample_size);
    a_fitness.assign(3, std::numeric_limits<double>::max());

    // Dense sampling of the curve containing the samples of the optimizer (uniform in the curve parameter): optimizer
    // sample j is dense sample j * kDensity
    constexpr int kDensity = 40;
    nurbs_->setControlPoints(pagmoDVToControlPoints(a_dv));
    nurbs_->setSampleSize((optimizer_sample_size - 1) * kDensity + 1);
    const Eigen::MatrixXd dense = nurbs_->evaluate();
    nurbs_->setSampleSize(optimizer_sample_size);
    if (dense.cols() < 2 || !dense.allFinite())
    {
        evaluation_time_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        return false;
    }
    std::vector<double> distance(dense.cols(), 0.0);
    for (int i = 1; i < dense.cols(); ++i)
        distance[i] = distance[i - 1] + (dense.block<3, 1>(0, i) - dense.block<3, 1>(0, i - 1)).norm();
    const double length = distance.back();
    // Instantaneous start and stop: on the first segment of the optimizer the speed is the one of its second sample, on
    // the last segment the one of its second last sample (the NURBS speed goes to 0 at the ends, its travel time would
    // grow without bound with the sampling resolution)
    const double first_end = distance[kDensity], last_start = distance[dense.cols() - 1 - kDensity];
    const double first_speed = dense(3, kDensity), last_speed = dense(3, dense.cols() - 1 - kDensity);

    // Samples every a_spacing meters along the curve (a longer trajectory has more samples)
    const int nb_of_samples = std::max(5, static_cast<int>(std::ceil(length / a_spacing)) + 1);
    a_samples = Eigen::MatrixXd(4, nb_of_samples);
    std::vector<double> sample_distance(nb_of_samples);
    int k = 1;
    for (int i = 0; i < nb_of_samples; ++i)
    {
        const double s = length * i / (nb_of_samples - 1);
        while (k < dense.cols() - 1 && distance[k] < s)
            ++k;
        const double segment = distance[k] - distance[k - 1];
        const double alpha = segment > 1.0e-12 ? std::clamp((s - distance[k - 1]) / segment, 0.0, 1.0) : 1.0;
        a_samples.col(i) = (1.0 - alpha) * dense.col(k - 1) + alpha * dense.col(k);
        sample_distance[i] = s;
    }

    // Safety cost, occupancy and acceleration constraints with the analyzer of ARENA (no early exit) on the NURBS speeds
    arena_demos::LinedroneNurbsAnalyzerConfig config = linedrone_config;
    config.base_config.sample_size = static_cast<unsigned int>(nb_of_samples);
    config.early_exit_on_constraint_violation_ = false;
    arena_demos::LinedroneNurbsAnalyzer analyzer(costmap_mapping_, config, std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>{});
    arena_core::EvalNurbsOutput output = linedrone_output_;
    output.fitness_array_.assign(output.fitness_size_, std::numeric_limits<double>::max());
    output.constraint_array_.assign(output.constraint_size_, std::numeric_limits<double>::max());
    analyzer.eval(a_samples, output);

    // Time and energy with the instantaneous start and stop (same energy model as the analyzer: steady-state power of the
    // direction of motion times the duration, transient energy m / 2 |v_b^2 - v_a^2| of every change of speed)
    auto speedAt = [&](int i)
    {
        if (sample_distance[i] <= first_end)
            return first_speed;
        if (sample_distance[i] >= last_start)
            return last_speed;
        return a_samples(3, i);
    };
    const double half_mass = 0.5 * linedrone_config.robot_mass_;
    double time = 0.0;
    double energy = half_mass * (first_speed * first_speed + last_speed * last_speed); // Start and stop
    for (int i = 0; i + 1 < nb_of_samples; ++i)
    {
        const Eigen::Vector3d displacement = a_samples.block<3, 1>(0, i + 1) - a_samples.block<3, 1>(0, i);
        const double d = displacement.norm();
        const double v0 = speedAt(i), v1 = speedAt(i + 1);
        const double mean_speed = 0.5 * (v0 + v1); // Constant acceleration between two samples
        const double dt = d / std::max(mean_speed, 1.0e-6);
        time += dt;
        if (d > 1.0e-12)
            energy += analyzer.steadyStatePower(displacement / d, displacement.z() < 0.0) * dt;
        energy += half_mass * std::abs(v1 * v1 - v0 * v0);
    }
    for (int i = 0; i < nb_of_samples; ++i)
        a_samples(3, i) = speedAt(i); // Reported speeds

    a_fitness = {time, output.fitness_array_[1], energy};
    const bool valid = output.constraint_array_[0] <= linedrone_config.robot_max_acceleration_ &&
                       output.constraint_array_[1] <= linedrone_config.max_occupancy_ &&
                       std::isfinite(output.fitness_array_[1]) && output.fitness_array_[1] != std::numeric_limits<double>::max() &&
                       std::isfinite(time) && std::isfinite(energy) && isPathSafe(a_samples);
    evaluation_time_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    return valid;
}

void LinedroneTestNode::initializerPlanning()
{
    if (!ompl_planner_)
    {
        RCLCPP_ERROR(get_logger(), "OMPL Planner is not initialized.");
        return;
    }

    planning_start_time_ = std::chrono::steady_clock::now();
    initialization_time_ = 0.0;
    optimization_time_ = 0.0;
    early_exit_mismatches_ = early_exit_checks_ = 0;
    evaluation_time_ = 0.0;
    nb_of_control_points_ = 0;
    updateOptimizationParameters();

    path_planned_ = false;
    initial_paths_.clear();

    if (arena_path_)
    {
        if (arena_path_->cols() > 0)
        {
            // Publish empty path to clear previous visualizations
            visualization_msgs::msg::MarkerArray empty_path;
            visualization_msgs::msg::Marker empty_marker;
            empty_marker.header.frame_id = "map";
            empty_marker.header.stamp = this->now();

            arena_path_pub_->publish(empty_path);
        }
    }

    // Set the bounds for the OMPL planner
    Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
    Eigen::Vector3d min_bounds = bounds.row(0).transpose();
    Eigen::Vector3d max_bounds = bounds.row(1).transpose();
    // optimization.bounds_z_top_margin: the bounds (RRT, A* seeds and optimization) extend above the highest voxel of the
    // octomap by this margin (m), the space above the map is free with a null collision cost
    if (this->has_parameter("optimization.bounds_z_top_margin"))
        max_bounds(2) += this->get_parameter("optimization.bounds_z_top_margin").as_double();

    ompl_planner_->setBounds(min_bounds, max_bounds);

    // Set the start state for the OMPL planner
    start_point_ = drone_position_;
    ompl_planner_->setStart(drone_position_);

    // RRT states clear of the inflated octomap (default). With the SDF, the states also have an SDF collision cost at most
    // the highest cost of the start and the goal (initial paths in the safest area, outside the SDF when possible):
    // optimization.initialization.sdf_validity_ratio is the share of the initial paths planned with the SDF (0: none, 1:
    // all), optimization.initialization.sdf_validity = true is a ratio of 1
    double sdf_validity_ratio = this->has_parameter("optimization.initialization.sdf_validity_ratio") ?
        std::clamp(this->get_parameter("optimization.initialization.sdf_validity_ratio").as_double(), 0.0, 1.0) : 0.0;
    if (this->has_parameter("optimization.initialization.sdf_validity") &&
        this->get_parameter("optimization.initialization.sdf_validity").as_bool())
        sdf_validity_ratio = 1.0;
    auto default_checker = std::make_shared<arena_core::OMPLStateValidityChecker>(ompl_planner_->getSpaceInformation(), costmap_mapping_);
    std::shared_ptr<ompl::base::StateValidityChecker> sdf_checker;
    auto sdf = costmap_mapping_->getCapability<arena_demos::CostSDFCapability>();
    if (sdf_validity_ratio > 0.0 && sdf)
    {
        const double max_cost = std::max(sdf->getCollisionCost(drone_position_), sdf->getCollisionCost(planning_goal_.goal_));
        sdf_checker = std::make_shared<arena_demos::SdfStateValidityChecker>(ompl_planner_->getSpaceInformation(), costmap_mapping_, max_cost);
        RCLCPP_INFO(get_logger(), "RRT initialization: %.0f%% of the paths with the SDF (collision cost at most %.3f)",
                    100.0 * sdf_validity_ratio, max_cost);
    }
    const size_t nb_of_sdf_paths = sdf_checker ? static_cast<size_t>(std::lround(sdf_validity_ratio * population_size_)) : 0;

    // Seed speed of the RRT paths relative to the max speed (see below)
    const double initial_speed_ratio = this->has_parameter("optimization.initialization.initial_speed_ratio") ?
        this->get_parameter("optimization.initialization.initial_speed_ratio").as_double() : 1.0;

    int successive_failed_planning_attempts = 0;
    // Start timer for initialization process
    auto start_time = std::chrono::steady_clock::now();

    // optimization.initialization.astar_ratio: share of the initial paths given by a shortest path A* on a grid of
    // optimization.initialization.astar_resolution (same validity as the RRT), its points taken every RRT range along the
    // path (same spacing as the nodes of the RRT paths), the other paths by the RRT
    const double astar_ratio = this->has_parameter("optimization.initialization.astar_ratio") ?
        std::clamp(this->get_parameter("optimization.initialization.astar_ratio").as_double(), 0.0, 1.0) : 0.0;
    const size_t nb_of_astar_paths = static_cast<size_t>(std::lround(astar_ratio * population_size_));
    if (nb_of_astar_paths > 0)
    {
        arena_benchmarks::WeightedAStar3DConfig astar_config;
        astar_config.resolution = this->has_parameter("optimization.initialization.astar_resolution") ?
            this->get_parameter("optimization.initialization.astar_resolution").as_double() : 1.0;
        astar_config.min_bounds = min_bounds;
        astar_config.max_bounds = max_bounds;
        auto mapping = costmap_mapping_;
        arena_benchmarks::WeightedAStar3D astar(astar_config, [mapping](const Eigen::Vector3d& p) { return mapping->isClearance(p, 0.8); });
        auto length = [](const Eigen::Vector3d& from, const Eigen::Vector3d& to) { return (to - from).norm(); };
        std::vector<Eigen::Vector3d> grid_path;
        if (astar.search(drone_position_, planning_goal_.goal_, length, length, grid_path) && grid_path.size() >= 2)
        {
            // Points every RRT range along the path, the start and the goal kept
            const double spacing = this->get_parameter("optimization.initialization.rrt_range").as_double();
            std::vector<Eigen::Vector3d> points = {grid_path.front()};
            double since_last = 0.0;
            for (size_t i = 1; i < grid_path.size(); ++i)
            {
                const Eigen::Vector3d a = grid_path[i - 1], b = grid_path[i];
                const double segment = (b - a).norm();
                double position = 0.0;
                while (segment - position + since_last >= spacing)
                {
                    position += spacing - since_last;
                    points.push_back(a + (b - a) * (position / segment));
                    since_last = 0.0;
                }
                since_last += segment - position;
            }
            if (points.size() > 1 && (points.back() - grid_path.back()).norm() < 0.5 * spacing)
                points.pop_back();
            points.push_back(grid_path.back());

            Eigen::MatrixXd path_matrix(4, points.size());
            for (size_t i = 0; i < points.size(); ++i)
            {
                path_matrix.block<3, 1>(0, i) = points[i];
                // Same seed speeds as the RRT paths (see below)
                path_matrix(3, i) = (i > 0 && i + 1 < points.size()) ? linedrone_config.robot_max_speed_ * initial_speed_ratio : 0.0;
            }
            for (size_t k = 0; k < nb_of_astar_paths; ++k)
                initial_paths_.push_back(path_matrix);
            RCLCPP_INFO(get_logger(), "Initialization: %zu path(s) from the shortest path A* (%zu points)", nb_of_astar_paths, points.size());
        }
        else
            RCLCPP_WARN(get_logger(), "Initialization: the shortest path A* found no path, the RRT gives every path");
    }

    const size_t first_rrt_path = initial_paths_.size(); // After the A* paths
    while (initial_paths_.size() < population_size_)
    {
        // The first RRT paths in the safest area (SDF), the others with the default validity
        ompl_planner_->getInitializer()->state_validity_checker_ = initial_paths_.size() - first_rrt_path < nb_of_sdf_paths ?
            sdf_checker : std::static_pointer_cast<ompl::base::StateValidityChecker>(default_checker);
        if (ompl_planner_->plan() == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            successive_failed_planning_attempts = 0;

            ompl::geometric::PathGeometric* path = ompl_planner_->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();
            if (path)
            {
                Eigen::MatrixXd path_matrix(4, path->getStateCount());
                for (size_t i = 0; i < path->getStateCount(); ++i)
                {
                    const ompl::base::State* state = path->getState(i);
                    const auto* real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
                    path_matrix(0, i) = real_state->values[0];
                    path_matrix(1, i) = real_state->values[1];
                    path_matrix(2, i) = real_state->values[2];
                    if (i>0 && i < path->getStateCount() - 1)
                    {
                        // Seed speed of the interior control points (4th dimension). The time and energy costs both
                        // decrease with the speed and the safety cost doesn't depend on it, so the optimizer starts at the
                        // maximum speed and only lowers it where needed (acceleration constraint). Seeding all the
                        // individuals at the same lower speed (0.5 x max) left the speed genes without diversity:
                        // the crossover of identical genes gives the same genes and the mutation alone moved them too
                        // slowly, the speeds stayed near the seed.
                        path_matrix(3, i) = linedrone_config.robot_max_speed_ * initial_speed_ratio;
                    }
                    else
                        path_matrix(3, i) = 0.0; // No velocity at the start and end points
                }

                initial_paths_.push_back(path_matrix);
            }
        }
        else
            successive_failed_planning_attempts++;

        if (successive_failed_planning_attempts >= 10)
        {
            RCLCPP_WARN(get_logger(), "Too many failed planning attempts during initialization. Stopping the planning process.");
            planning_activated_ = false;
            initialization_time_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            publishInfeasiblePlanning();
            return;
        }
    }

    // Stop timer for initialization process
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    initialization_time_ = std::chrono::duration<double>(end_time - start_time).count();
    RCLCPP_INFO(get_logger(), "OMPL Planner initialized in %ld milliseconds with %zu paths generated.", duration, initial_paths_.size());

    // Publish the initial paths
    publishOMPLPlannerPaths();

    // Start optimization process
    ARENAOptimization();
}

pagmo::vector_double LinedroneTestNode::linedroneFitness(const pagmo::vector_double& dv)
{
    if (!linedrone_nurbs_analyzer_)
        throw std::runtime_error("linedroneFitness => Linedrone NURBS Analyzer is not initialized.");

    if (!nurbs_)
        throw std::runtime_error("linedroneFitness => NURBS is not initialized.");

    linedrone_output_.fitness_array_[0] = std::numeric_limits<double>::max();
    linedrone_output_.fitness_array_[1] = std::numeric_limits<double>::max();
    linedrone_output_.fitness_array_[2] = std::numeric_limits<double>::max();
    linedrone_output_.constraint_array_[0] = std::numeric_limits<double>::max();
    linedrone_output_.constraint_array_[1] = std::numeric_limits<double>::max();
    
    std::vector<arena_core::ControlPoint<double, 4>> control_points = pagmoDVToControlPoints(dv);
    nurbs_->setControlPoints(control_points);
    Eigen::MatrixXd pt = nurbs_->evaluate();

    // Degenerate curve (e.g. null weights on a whole span), rejected instead of giving NaN costs that crash NSGA-II
    if (pt.cols() != static_cast<int>(linedrone_config.base_config.sample_size) || !pt.allFinite())
        return linedrone_output_.fitness_array_;

    // Rejection of the solutions violating a constraint: every cost is the max double
    auto reject = [this](arena_core::EvalNurbsOutput& output)
    {
        // Reject the solution if it exceeds the maximum acceleration or if it's in collision with the environment
        if (output.constraint_array_[0] > linedrone_config.robot_max_acceleration_ ||
            output.constraint_array_[1] > linedrone_config.max_occupancy_)
            output.fitness_array_.assign(output.fitness_array_.size(), std::numeric_limits<double>::max());
    };

    // Evaluate the NURBS curve using the Linedrone NURBS Analyzer (stops at the first constraint violation with
    // optimization.early_exit_on_constraint_violation)
    linedrone_nurbs_analyzer_->eval(pt, linedrone_output_);
    reject(linedrone_output_);

    // optimization.verify_early_exit: same fitness as the evaluation of the whole curve
    if (verify_early_exit_ && reference_nurbs_analyzer_)
    {
        arena_core::EvalNurbsOutput reference = linedrone_output_;
        reference.fitness_array_.assign(reference.fitness_size_, std::numeric_limits<double>::max());
        reference.constraint_array_.assign(reference.constraint_size_, std::numeric_limits<double>::max());
        reference_nurbs_analyzer_->eval(pt, reference);
        reject(reference);
        early_exit_checks_++;
        if (reference.fitness_array_ != linedrone_output_.fitness_array_)
            early_exit_mismatches_++;
    }

    return linedrone_output_.fitness_array_;
}

void LinedroneTestNode::ARENAOptimization()
{
    // Start timer for the optimization process
    auto start_time = std::chrono::steady_clock::now();

    // Get minimum number of control points to set the amounnt of decision variables for the optimization
    int nb_of_control_points = initial_paths_[0].cols();
    for (size_t i = 1; i < initial_paths_.size(); ++i)
    {
        if (initial_paths_[i].cols() > nb_of_control_points)
            nb_of_control_points = initial_paths_[i].cols();
    }

    if (nb_of_control_points < nurbs_->getDegree() + 1)
        nb_of_control_points = nurbs_->getDegree() + 1;
    nb_of_control_points_ = nb_of_control_points;

    // Make sure all initial paths have the same number of control points
    for (size_t i = 0; i < initial_paths_.size(); ++i)
    {
        if (initial_paths_[i].cols() < nb_of_control_points)
        {
            // Add control points to the path to match the number of control points
            initial_paths_[i] = addControlPointsToPath(initial_paths_[i], nb_of_control_points);
        }
    }

    // Check if there is any inf or nan in the initial paths
    for (const auto& path : initial_paths_)
    {
        if (path.hasNaN() || path.array().isInf().any())
        {
            RCLCPP_ERROR(get_logger(), "ARENAOptimization => Initial path contains NaN or Inf values. Optimization cannot proceed.");
            publishInfeasiblePlanning();
            return;
        }
    }

    // Create pagmo problem
    std::function<pagmo::vector_double(const pagmo::vector_double &)> fitness_eval_cb = std::bind(&LinedroneTestNode::linedroneFitness, this, std::placeholders::_1);
    std::pair<Eigen::VectorXd, Eigen::VectorXd> bounds = ompl_planner_->getBounds();
    double* x_bounds = new double[2] {bounds.first(0), bounds.second(0)};
    double* y_bounds = new double[2] {bounds.first(1), bounds.second(1)};
    double* z_bounds = new double[2] {bounds.first(2), bounds.second(2)};
    pagmo::problem prob_linedrone{pagmo::linedrone_problem((nb_of_control_points - 2) * 4 + nb_of_control_points,
                                                           3u,
                                                           &fitness_eval_cb,
                                                           x_bounds,
                                                           y_bounds,
                                                           z_bounds,
                                                           linedrone_config.robot_max_speed_)};
    RCLCPP_INFO(get_logger(), "ARENAOptimization => Problem created");
    std::cout << prob_linedrone << std::endl;

    int nb_of_generations = this->get_parameter("optimization.NSGA-II.generations").as_int();
    std::function<void(const pagmo::population&)> show_set_callback = std::bind(&LinedroneTestNode::publishSolutionSet, this, std::placeholders::_1);
    pagmo::algorithm nsga2{pagmo::nsga2(nb_of_generations, 0.95, 10.0, 0.01, 50.0, pagmo::random_device::next(), nullptr)};
    pagmo::vector_double adaptive_matrix = pagmo::vector_double(prob_linedrone.get_nf(), 0.0);
    nsga2.set_adaptive_matrix(adaptive_matrix);

    pagmo::population pop_linedrone{prob_linedrone, population_size_};
    pop_linedrone.clear();

    for (size_t i = 0; i < initial_paths_.size(); ++i)
    {
        pagmo::vector_double dv;
        dv.push_back(1.0); // Weight of the first control point
        for (size_t j = 1; j < nb_of_control_points - 1; j++)
        {
            dv.push_back(initial_paths_[i](0, j)); // X coordinate
            dv.push_back(initial_paths_[i](1, j)); // Y coordinate
            dv.push_back(initial_paths_[i](2, j)); // Z coordinate
            dv.push_back(initial_paths_[i](3, j)); // Velocity
            dv.push_back(1.0); // Weight of the control point
        }
        dv.push_back(1.0); // Weight of the last control point
        pagmo::vector_double fitness = prob_linedrone.fitness(dv);
        pop_linedrone.push_back(dv, fitness);
    }

    pop_linedrone = nsga2.evolve(pop_linedrone);

    // End timer for the optimization process
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    optimization_time_ = std::chrono::duration<double>(end_time - start_time).count();
    RCLCPP_INFO(get_logger(), "ARENAOptimization => Optimization completed in %ld milliseconds.", duration);
    if (verify_early_exit_)
        RCLCPP_INFO(get_logger(), "ARENAOptimization => Early exit check: %zu mismatches in %zu fitness evaluations",
                    early_exit_mismatches_, early_exit_checks_);

    // Filter trajectories that doesn't respect the constraints
    std::vector<int> to_remove;
    for (int i = 0; i < pop_linedrone.size(); i++)
    {
        pagmo::vector_double fitness = pop_linedrone.get_f()[i];
        if (fitness[0] == std::numeric_limits<double>::max() || fitness[1] == std::numeric_limits<double>::max() || fitness[2] == std::numeric_limits<double>::max())
            to_remove.push_back(i);
    }

    if (to_remove.size() >= pop_linedrone.size())
    {
        RCLCPP_ERROR(get_logger(), "ARENAOptimization => No valid solution found.");
        delete[] x_bounds;
        delete[] y_bounds;
        delete[] z_bounds;
        planning_activated_ = false;
        planning_goal_.goal_sent_ = false;
        publishInfeasiblePlanning();
        return;
    }

    pagmo::population pop_linedrone_filtered(prob_linedrone, population_size_ - to_remove.size());
    int j = 0;
    for (int i = 0; i < pop_linedrone.size(); i++)
    {
        if (std::find(to_remove.begin(), to_remove.end(), i) == to_remove.end())
        {
            pagmo::vector_double dv = pop_linedrone.get_x()[i];
            pop_linedrone_filtered.set_x(j, dv);
            j++;
        }
    }

    // Safety check of every feasible solution: the optimizer only checks the samples of the trajectories, a segment
    // between two samples can still cross an obstacle. Only the safe solutions can be chosen.
    // The costs reported to the testbench are evaluated with optimization.evaluation_sample_size samples when it is set,
    // the chosen solution is still selected with the costs of the optimizer.
    std::vector<bool> safe(pop_linedrone_filtered.size(), false);
    std::vector<pagmo::vector_double> safe_fitness;
    std::vector<size_t> safe_indexes;
    std::vector<pagmo::vector_double> reported_fitness;
    // optimization.evaluation_spacing (tests only): final evaluation of every solution at a resolution of the map for the
    // reports, the optimizer and the voting keep their own evaluation
    const double evaluation_spacing = costmap_mapping_->evaluationSpacing(
        this->has_parameter("optimization.evaluation_spacing") ? this->get_parameter("optimization.evaluation_spacing").as_double() : 0.0);
    std::vector<bool> resolution_valid(pop_linedrone_filtered.size(), false);
    std::vector<Eigen::MatrixXd> resolution_samples(pop_linedrone_filtered.size());
    reported_path_ = Eigen::MatrixXd();
    for (size_t i = 0; i < pop_linedrone_filtered.size(); i++)
    {
        nurbs_->setControlPoints(pagmoDVToControlPoints(pop_linedrone_filtered.get_x()[i]));
        Eigen::MatrixXd path = nurbs_->evaluate();
        safe[i] = isPathSafe(path);
        if (evaluation_spacing > 0.0)
        {
            pagmo::vector_double fitness;
            resolution_valid[i] = resolutionEvaluation(pop_linedrone_filtered.get_x()[i], evaluation_spacing, resolution_samples[i], fitness);
            reported_fitness.push_back(fitness);
        }
        else
            reported_fitness.push_back(evaluationFitness(path, pop_linedrone_filtered.get_f()[i]));
        if (safe[i])
        {
            safe_fitness.push_back(pop_linedrone_filtered.get_f()[i]);
            safe_indexes.push_back(i);
        }
    }

    // Report of the final evaluation: only the solutions passing it, the chosen one must pass it to be reported feasible
    auto publishReport = [&](int chosen_idx, std::chrono::steady_clock::time_point end_time)
    {
        planning_end_time_ = end_time; // The publications aren't part of the planning
        if (evaluation_spacing <= 0.0)
        {
            publishPlanningInfos(reported_fitness, safe, chosen_idx);
            return;
        }
        std::vector<pagmo::vector_double> valid_fitness;
        int valid_chosen_idx = -1;
        for (size_t i = 0; i < reported_fitness.size(); ++i)
        {
            if (!resolution_valid[i])
                continue;
            if (static_cast<int>(i) == chosen_idx)
            {
                valid_chosen_idx = static_cast<int>(valid_fitness.size());
                reported_path_ = resolution_samples[i];
            }
            valid_fitness.push_back(reported_fitness[i]);
        }
        if (chosen_idx >= 0 && valid_chosen_idx < 0)
            RCLCPP_WARN(get_logger(), "ARENAOptimization => The chosen trajectory fails the final evaluation (%.3f m spacing)", evaluation_spacing);
        publishPlanningInfos(valid_fitness, std::vector<bool>(valid_fitness.size(), true), valid_chosen_idx);
    };
    RCLCPP_INFO(get_logger(), "ARENAOptimization => %zu / %zu feasible solutions are safe.", safe_indexes.size(), safe.size());

    // optimization.export_solutions_file (analysis only): every safe solution of the final population with the costs used
    // by the voting (optimizer), the reported costs and the clearance of its reported trajectory, to replay the voting
    // with other weights on the same population (e.g. the risk sweep of Fig. 4)
    const std::string export_file = this->has_parameter("optimization.export_solutions_file") ?
        this->get_parameter("optimization.export_solutions_file").as_string() : "";
    if (!export_file.empty())
    {
        auto export_start_time = std::chrono::steady_clock::now();
        bool new_file = !std::ifstream(export_file).good();
        std::ofstream file(export_file, std::ios::out | std::ios::app);
        if (new_file)
            file << "Planning,Solution,Vote time cost,Vote security cost,Vote energy cost,Time cost,Security cost,Energy cost,"
                    "Average Closest Obstacle Distance,Min Closest Obstacle Distance\n";
        // The solutions of the report: the ones passing the final evaluation at a resolution of the map
        // (optimization.evaluation_spacing, on its samples), the safe ones otherwise
        std::vector<size_t> exported_indexes;
        for (size_t i = 0; i < pop_linedrone_filtered.size(); ++i)
            if (evaluation_spacing > 0.0 ? static_cast<bool>(resolution_valid[i]) : static_cast<bool>(safe[i]))
                exported_indexes.push_back(i);
        for (size_t i : exported_indexes)
        {
            Eigen::MatrixXd path;
            if (evaluation_spacing > 0.0)
                path = resolution_samples[i];
            else
            {
                nurbs_->setControlPoints(pagmoDVToControlPoints(pop_linedrone_filtered.get_x()[i]));
                path = nurbs_->evaluate();
                if (evaluation_nurbs_analyzer_ && path.cols() >= 2 && path.cols() < static_cast<int>(evaluation_sample_size_))
                    path = interpolateTrajectory(path, evaluation_sample_size_);
            }
            const pagmo::vector_double& vote_fitness = pop_linedrone_filtered.get_f()[i];
            double sum = 0.0, min_distance = std::numeric_limits<double>::max();
            for (int c = 0; c < path.cols(); ++c)
            {
                double d = costmap_mapping_->closestObstacleDistance(path.block<3, 1>(0, c));
                sum += d;
                min_distance = std::min(min_distance, d);
            }
            file << std::fixed << std::setprecision(6) << export_counter_ << "," << i << ","
                 << vote_fitness[0] << "," << vote_fitness[1] << "," << vote_fitness[2] << ","
                 << reported_fitness[i][0] << "," << reported_fitness[i][1] << "," << reported_fitness[i][2] << ","
                 << sum / path.cols() << "," << min_distance << "\n";
        }
        RCLCPP_INFO(get_logger(), "ARENAOptimization => %zu solutions exported to %s", exported_indexes.size(), export_file.c_str());
        export_counter_++;
        evaluation_time_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - export_start_time).count();
    }

    if (safe_indexes.empty())
    {
        RCLCPP_ERROR(get_logger(), "ARENAOptimization => No safe solution found.");
        delete[] x_bounds;
        delete[] y_bounds;
        delete[] z_bounds;
        planning_activated_ = false;
        planning_goal_.goal_sent_ = false;
        publishReport(-1, std::chrono::steady_clock::now());
        return;
    }

    // Voting algorithm (optimization.voting) selecting the best safe solution with the coefficients of this planning
    const int voted_idx = voting_algorithm_->selectCandidate(safe_fitness, voting_coefficients_);
    int best_idx = static_cast<int>(safe_indexes[voted_idx]);
    const auto choice_time = std::chrono::steady_clock::now();
    pagmo::vector_double best_solution = pop_linedrone_filtered.get_x()[best_idx];

    // Convert the best solution to control points
    std::vector<arena_core::ControlPoint<double, 4>> best_control_points = pagmoDVToControlPoints(best_solution);
    nurbs_->setControlPoints(best_control_points);
    arena_path_ = std::make_shared<Eigen::MatrixXd>(nurbs_->evaluate());
    publishARENAPath();
    publishControlPoints();

    delete[] x_bounds;
    delete[] y_bounds;
    delete[] z_bounds;
    
    path_planned_ = true;
    planning_activated_ = false;
    planning_goal_.goal_sent_ = false;

    publishReport(best_idx, choice_time);
}

void LinedroneTestNode::updateOptimizationParameters()
{
    updateVotingAlgorithm();

    // Distance between the RRT nodes of the initialization
    if (ompl_planner_ && ompl_planner_->getInitializer()->planner_)
        ompl_planner_->getInitializer()->planner_->as<ompl::geometric::RRT>()->setRange(this->get_parameter("optimization.initialization.rrt_range").as_double());

    population_size_ = this->get_parameter("optimization.NSGA-II.population_size").as_int();
    // Make sure the population size is divisible by 4
    if (population_size_ % 4 != 0)
        population_size_ += 4 - (population_size_ % 4);

    // optimization.integral_safety_cost (off by default): safety cost integrated along the path instead of averaged over
    // the samples (LinedroneNurbsAnalyzerConfig::integral_safety_cost_), in every analyzer (optimizer and reports)
    const bool integral_safety_cost = this->has_parameter("optimization.integral_safety_cost") &&
                                      this->get_parameter("optimization.integral_safety_cost").as_bool();
    const bool safety_cost_changed = integral_safety_cost != linedrone_config.integral_safety_cost_;
    linedrone_config.integral_safety_cost_ = integral_safety_cost;

    // Sample size of the costs reported to the testbench, 0 (or not set): the costs of the optimizer
    const int64_t evaluation_sample_size = this->has_parameter("optimization.evaluation_sample_size") ?
        this->get_parameter("optimization.evaluation_sample_size").as_int() : 0;
    if (evaluation_sample_size != static_cast<int64_t>(evaluation_sample_size_) || safety_cost_changed)
    {
        evaluation_sample_size_ = static_cast<unsigned int>(std::max<int64_t>(evaluation_sample_size, 0));
        evaluation_nurbs_analyzer_ = nullptr;
        if (evaluation_sample_size_ > 0)
        {
            arena_demos::LinedroneNurbsAnalyzerConfig evaluation_config = linedrone_config;
            evaluation_config.base_config.sample_size = evaluation_sample_size_;
            evaluation_nurbs_analyzer_ = std::make_shared<arena_demos::LinedroneNurbsAnalyzer>(costmap_mapping_,
                                                                                                evaluation_config,
                                                                                                std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>{});
            RCLCPP_INFO(get_logger(), "The costs of the reported solutions are evaluated with %u samples", evaluation_sample_size_);
        }
    }

    // Rejection at the first constraint violation in the analyzer of the optimizer (same fitness, faster), and its check
    // (on by default)
    const bool early_exit = !this->has_parameter("optimization.early_exit_on_constraint_violation") ||
                            this->get_parameter("optimization.early_exit_on_constraint_violation").as_bool();
    const bool verify_early_exit = this->has_parameter("optimization.verify_early_exit") &&
                                   this->get_parameter("optimization.verify_early_exit").as_bool();

    unsigned int sample_size = this->get_parameter("optimization.sample_size").as_int();
    if (sample_size == linedrone_config.base_config.sample_size && nurbs_ && nurbs_->getSampleSize() == static_cast<int>(sample_size) &&
        early_exit == early_exit_ && verify_early_exit == verify_early_exit_ && !safety_cost_changed)
        return;

    early_exit_ = early_exit;
    verify_early_exit_ = verify_early_exit;
    linedrone_config.base_config.sample_size = sample_size;
    if (nurbs_)
        nurbs_->setSampleSize(sample_size);

    // The analyzer keeps its own copy of the configuration. linedrone_config keeps the early exit off: the analyzer of the
    // reported costs is built from it
    arena_demos::LinedroneNurbsAnalyzerConfig optimizer_config = linedrone_config;
    optimizer_config.early_exit_on_constraint_violation_ = early_exit_;
    linedrone_nurbs_analyzer_ = std::make_shared<arena_demos::LinedroneNurbsAnalyzer>(costmap_mapping_,
                                                                                      optimizer_config,
                                                                                      std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>{});
    reference_nurbs_analyzer_ = verify_early_exit_ ?
        std::make_shared<arena_demos::LinedroneNurbsAnalyzer>(costmap_mapping_, linedrone_config,
                                                              std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>{}) :
        nullptr;
    RCLCPP_INFO(get_logger(), "Analyzer of the optimizer: early exit on constraint violation %s%s, safety cost %s", early_exit_ ? "on" : "off",
                verify_early_exit_ ? ", every fitness checked against the full evaluation" : "",
                linedrone_config.integral_safety_cost_ ? "integrated along the path" : "mean over the samples");
}

void LinedroneTestNode::updateVotingAlgorithm()
{
    auto doubleParam = [this](const std::string& name, double default_value)
    { return this->has_parameter(name) ? this->get_parameter(name).as_double() : default_value; };

    arena_core::VotingAlgorithmSettings settings;
    settings.nominal_coefficients = {doubleParam("optimization.voting.nominal_coefficients.time", 1.0 / 3.0),
                                     doubleParam("optimization.voting.nominal_coefficients.safety", 1.0 / 3.0),
                                     doubleParam("optimization.voting.nominal_coefficients.energy", 1.0 / 3.0)};
    settings.gain = doubleParam("optimization.voting.risk_weighted_tchebycheff.gain", settings.gain);
    settings.augmentation = doubleParam("optimization.voting.risk_weighted_tchebycheff.augmentation", settings.augmentation);
    const std::string algorithm = this->has_parameter("optimization.voting.algorithm") ?
        this->get_parameter("optimization.voting.algorithm").as_string() : "risk_weighted_tchebycheff";
    try
    {
        settings.type = arena_core::votingAlgorithmTypeFromString(algorithm);
        voting_algorithm_ = arena_core::createVotingAlgorithm(settings);
    }
    catch (const std::invalid_argument& e)
    {
        RCLCPP_ERROR(get_logger(), "Invalid voting parameters (%s), the default risk weighted Tchebycheff voting is used", e.what());
        voting_algorithm_ = arena_core::createVotingAlgorithm(arena_core::VotingAlgorithmSettings());
    }

    const bool use_risks = this->has_parameter("optimization.voting.use_risks") ?
        this->get_parameter("optimization.voting.use_risks").as_bool() : true;
    if (use_risks)
    {
        // Risks of the mission, in [0, 1]
        auto risk = [&doubleParam](const std::string& name) { return std::clamp(doubleParam("optimization.voting.risks." + name, 0.0), 0.0, 1.0); };
        arena_core::OperationalRisks risks{risk("wind"), risk("communication"), risk("localization"), risk("battery")};
        voting_coefficients_ = voting_algorithm_->computeCoefficients(risks);
        RCLCPP_INFO(get_logger(), "Voting %s, risks (wind %.2f, communication %.2f, localization %.2f, battery %.2f) => "
                    "coefficients (time %.3f, safety %.3f, energy %.3f)", voting_algorithm_->name().c_str(), risks.wind,
                    risks.communication, risks.localization, risks.battery, voting_coefficients_[0],
                    voting_coefficients_[1], voting_coefficients_[2]);
    }
    else
    {
        // Fixed coefficients (tests)
        voting_coefficients_ = {doubleParam("optimization.adaptive_costs_weights.time", 1.0),
                                doubleParam("optimization.adaptive_costs_weights.safety", 0.0),
                                doubleParam("optimization.adaptive_costs_weights.energy", 0.0)};
        RCLCPP_INFO(get_logger(), "Voting %s with fixed coefficients (time %.3f, safety %.3f, energy %.3f)",
                    voting_algorithm_->name().c_str(), voting_coefficients_[0], voting_coefficients_[1], voting_coefficients_[2]);
    }
}

void LinedroneTestNode::publishPlanningInfos(const std::vector<pagmo::vector_double>& feasible_fitness, const std::vector<bool>& safe, int chosen_idx)
{
    const bool feasible = chosen_idx >= 0 && chosen_idx < static_cast<int>(feasible_fitness.size());
    const double max_cost = std::numeric_limits<double>::max();

    arena_msgs::msg::OptimizerPathInfo infos_msg;
    infos_msg.feasible = feasible;
    infos_msg.path.header.frame_id = "map";
    infos_msg.path.header.stamp = this->now();

    if (feasible && arena_path_)
    {
        // Same samples as the reported costs, the testbench measures the clearance of the trajectory on them
        Eigen::MatrixXd reported_path = *arena_path_;
        if (reported_path_.cols() > 0)
            reported_path = reported_path_; // Final evaluation at a resolution of the map
        else if (evaluation_nurbs_analyzer_ && reported_path.cols() >= 2 && reported_path.cols() < static_cast<int>(evaluation_sample_size_))
            reported_path = interpolateTrajectory(reported_path, evaluation_sample_size_);

        for (int i = 0; i < reported_path.cols(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = infos_msg.path.header;
            pose.pose.position.x = reported_path(0, i);
            pose.pose.position.y = reported_path(1, i);
            pose.pose.position.z = reported_path(2, i);
            pose.pose.orientation.w = 1.0;
            infos_msg.path.poses.push_back(pose);
            infos_msg.velocities.push_back(reported_path(3, i));
        }
    }

    // Planning time: from the activation to the choice of the trajectory, without the evaluations of the reports
    infos_msg.planning_time = std::chrono::duration<double>(planning_end_time_ - planning_start_time_).count() - evaluation_time_;
    if (evaluation_time_ > 0.0)
        RCLCPP_INFO(get_logger(), "Evaluation of the reported costs: %.1f ms (not in the planning time)", evaluation_time_ * 1.0e3);
    infos_msg.initialization_time = initialization_time_;
    infos_msg.optimization_time = optimization_time_;
    infos_msg.nb_of_control_points = nb_of_control_points_;
    infos_msg.nb_of_generations = this->get_parameter("optimization.NSGA-II.generations").as_int();
    infos_msg.population_size = population_size_;
    infos_msg.nurbs_sample_size = linedrone_config.base_config.sample_size;
    infos_msg.rrt_range = this->get_parameter("optimization.initialization.rrt_range").as_double();

    // Safe solutions, the only ones that can be chosen
    std::vector<pagmo::vector_double> safe_fitness;
    for (size_t i = 0; i < feasible_fitness.size(); i++)
    {
        if (i < safe.size() && safe[i])
            safe_fitness.push_back(feasible_fitness[i]);
    }
    infos_msg.nb_of_feasible_solutions = static_cast<int32_t>(feasible_fitness.size());
    infos_msg.nb_of_safe_solutions = static_cast<int32_t>(safe_fitness.size());

    // Best value reached for every objective by the safe solutions
    infos_msg.best_time_cost = max_cost;
    infos_msg.best_security_cost = max_cost;
    infos_msg.best_energy_cost = max_cost;
    for (const auto& fitness : safe_fitness)
    {
        infos_msg.best_time_cost = std::min(infos_msg.best_time_cost, fitness[0]);
        infos_msg.best_security_cost = std::min(infos_msg.best_security_cost, fitness[1]);
        infos_msg.best_energy_cost = std::min(infos_msg.best_energy_cost, fitness[2]);
    }

    infos_msg.chosen_time_cost = feasible ? feasible_fitness[chosen_idx][0] : max_cost;
    infos_msg.chosen_security_cost = feasible ? feasible_fitness[chosen_idx][1] : max_cost;
    infos_msg.chosen_energy_cost = feasible ? feasible_fitness[chosen_idx][2] : max_cost;

    // Coefficients used by the voting algorithm for this planning
    infos_msg.time_coefficient = voting_coefficients_[0];
    infos_msg.security_coefficient = voting_coefficients_[1];
    infos_msg.energy_coefficient = voting_coefficients_[2];

    // Non-dominated solutions (pagmo needs at least 2 points to sort them)
    auto firstFront = [](const std::vector<pagmo::vector_double>& fitness) -> std::vector<pagmo::pop_size_t>
    {
        if (fitness.size() < 2)
            return std::vector<pagmo::pop_size_t>(fitness.size(), 0);
        return std::get<0>(pagmo::fast_non_dominated_sorting(fitness))[0];
    };

    // Pareto front approximation: non-dominated feasible solutions of the final population, with their safety
    for (pagmo::pop_size_t idx : firstFront(feasible_fitness))
    {
        infos_msg.pareto_front_time_costs.push_back(feasible_fitness[idx][0]);
        infos_msg.pareto_front_security_costs.push_back(feasible_fitness[idx][1]);
        infos_msg.pareto_front_energy_costs.push_back(feasible_fitness[idx][2]);
        infos_msg.pareto_front_safe.push_back(idx < safe.size() && safe[idx]);
    }

    // Non-dominated safe solutions, they can be dominated by unsafe solutions of the Pareto front above
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

bool LinedroneTestNode::isCurrentPathSafe() const
{
    if (!path_planned_)
        return true;
    
    if (!arena_path_)
        return false;

    if (arena_path_->cols() == 0)
        return false;
    
    if (arena_path_->cols() != linedrone_config.base_config.sample_size)
    {
        RCLCPP_WARN(get_logger(), "Current path size does not match the sample size. Expected: %u, Got: %zu",
                    linedrone_config.base_config.sample_size, arena_path_->cols());
        return false; // Path size mismatch
    }

    if (!isPathSafe(*arena_path_))
    {
        RCLCPP_WARN(get_logger(), "Collision detected along the current path.");
        return false;
    }

    return true; // No collisions detected along the path
}

bool LinedroneTestNode::isPathSafe(const Eigen::MatrixXd& path) const
{
    if (!costmap_mapping_)
        return false;

    // Check if every segment between two consecutive samples is free in the octree
    for (int i = 0; i < path.cols() - 1; ++i)
    {
        Eigen::Vector3d start = path.block<3, 1>(0, i);
        Eigen::Vector3d end = path.block<3, 1>(0, i + 1);

        if (costmap_mapping_->isOccupiedRayTracing(start, end))
            return false;
    }

    return true;
}

// Main logic of the node
void LinedroneTestNode::run()
{
    if (!linedrone_nurbs_analyzer_)
    {
        RCLCPP_ERROR(get_logger(), "Linedrone NURBS Analyzer is not initialized.");
        return;
    }

    // Wait for both maps before planning, the planning request is kept until then.
    // The inflated octomap is used for collision checking and the SDF octomap for the safety cost,
    // planning without the latter gives NaN safety costs that crash NSGA-II.
    if (!octree_received_ || !color_octree_received_)
        return;

    // Perform the main logic of the node
    if ((planning_activated_ && planning_goal_.goal_sent_) || !isCurrentPathSafe())
        initializerPlanning();
}

LinedroneTestNode::LinedroneTestNode(rclcpp::NodeOptions options)
: Node("linedrone_test_node", options), linedrone_config(50), costmap_mapping_(std::make_shared<arena_demos::CostmapMapping>()),
  linedrone_nurbs_analyzer_(nullptr), ompl_planner_(nullptr), nurbs_(nullptr), arena_path_(nullptr)
{
    // ROS Publisher Initialization
    arena_control_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("arena_control_points", 10);
    arena_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("arena_path", 10);
    ompl_planner_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ompl_planner_paths", 10);
    solution_set_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("solution_set", 10);

    ros_namespace_ = this->get_namespace();

    if (ros_namespace_ != "/" && ros_namespace_.back() != '/')
        ros_namespace_ += "/";

    path_planning_finished_pub_ = this->create_publisher<std_msgs::msg::Bool>(ros_namespace_ + "path_planning_finished", 10);
    nurbs_infos_pub_ = this->create_publisher<arena_msgs::msg::OptimizerPathInfo>(ros_namespace_ + "nurbs_infos", 10);

    // ROS Subscription Initialization
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(ros_namespace_ + "goal_pose", 10, std::bind(&LinedroneTestNode::goalPoseCallback, this, std::placeholders::_1));
    drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(ros_namespace_ + "drone_pose", 10, std::bind(&LinedroneTestNode::dronePoseCallback, this, std::placeholders::_1));
    planning_activation_sub_ = this->create_subscription<std_msgs::msg::Bool>(ros_namespace_ + "planning_activation", 10, std::bind(&LinedroneTestNode::planningActivationCallback, this, std::placeholders::_1));
    inflated_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/inflated_octomap/full", 10, std::bind(&LinedroneTestNode::inflatedOctomapCallback, this, std::placeholders::_1));
    color_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/sdf_octomap/full", 10, std::bind(&LinedroneTestNode::colorOctreeCallback, this, std::placeholders::_1));

    // Define the nurbs_analyzer configurations with ros parameters
    linedrone_config.robot_mass_ = this->get_parameter("robot.mass").as_double(); // kg
    linedrone_config.robot_max_speed_ = this->get_parameter("robot.max_speed").as_double(); // m/s
    linedrone_config.robot_max_acceleration_ = this->get_parameter("robot.max_acceleration").as_double(); // m/s^2
    linedrone_config.robot_permanent_power_ascent_ = this->get_parameter("robot.permanent_power.ascent").as_double(); // Watts
    linedrone_config.robot_permanent_power_roll_ = this->get_parameter("robot.permanent_power.roll").as_double(); // Watts
    linedrone_config.robot_permanent_power_pitch_ = this->get_parameter("robot.permanent_power.pitch").as_double(); // Watts
    linedrone_config.robot_permanent_power_descent_ = this->get_parameter("robot.permanent_power.descent").as_double(); // Watts
    linedrone_config.solveQuadraticSurfaceCoefficients();

    linedrone_nurbs_analyzer_ = std::make_shared<arena_demos::LinedroneNurbsAnalyzer>(costmap_mapping_,
                                                                                      linedrone_config, 
                                                                                      std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>{});
    
    // Initialize the nurbs output structure
    linedrone_output_.fitness_size_ = 3;
    linedrone_output_.fitness_array_ = std::vector<double>(linedrone_output_.fitness_size_, std::numeric_limits<double>::max()); // We are optimizing to minimize the fitness values
    linedrone_output_.constraint_size_ = 2;
    linedrone_output_.constraint_array_ = std::vector<double>(linedrone_output_.constraint_size_, std::numeric_limits<double>::max());

    // Initialize the OMPL planner
    ompl_planner_ = std::make_shared<arena_core::OMPLPlanner>();
    double timeout = this->get_parameter("optimization.initialization.timeout").as_double();
    ompl_planner_->setSolvingTimeout(timeout);
    ompl_planner_->setProblemDimensions(3);                                                                                  
    ompl_planner_->getInitializer()->planner_ = std::make_shared<ompl::geometric::RRT>(ompl_planner_->getSpaceInformation());
    double rrt_range = this->get_parameter("optimization.initialization.rrt_range").as_double();
    ompl_planner_->getInitializer()->planner_->as<ompl::geometric::RRT>()->setRange(rrt_range);
    ompl_planner_->getInitializer()->state_validity_checker_ = std::make_shared<arena_core::OMPLStateValidityChecker>(ompl_planner_->getSpaceInformation(), costmap_mapping_);
    ompl_planner_->getInitializer()->optimization_objective_ = nullptr;

    // Initialize the NURBS API
    // The sample size from the parameters is applied by updateOptimizationParameters() so the analyzer gets it too
    nurbs_ = std::make_shared<arena_core::Nurbs<4>>(std::vector<arena_core::ControlPoint<double, 4>>(), linedrone_config.base_config.sample_size);
    updateOptimizationParameters();

    // ROS Timer Initialization
    // Timer for the main loop at 50 Hz
    run_timer_ = this->create_wall_timer(20ms, std::bind(&LinedroneTestNode::run, this));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<LinedroneTestNode>(options));
    rclcpp::shutdown();
    return 0;
}
