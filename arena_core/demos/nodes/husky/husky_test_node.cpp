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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <grid_map_msgs/msg/grid_map.hpp>

// Local
#include "arena_core/math/nurbs.h"
#include "arena_core/math/control_point.h"
#include "arena_core/planning/OMPLPlanner.h"
#include "arena_core/geometry/ompl_state_validity_checker.h"
#include "arena_core/math/algorithm/adaptive_voting_algorithm.h"
#include "husky/husky_nurbs_analyzer.h"
#include "husky/husky_problem.hpp"

// External Libraries
// Eigen
#include <Eigen/Dense>
// Pagmo
#include <pagmo/algorithms/nsga2.hpp>
// OMPL
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// Grid_Map
#include <grid_map_pcl/grid_map_pcl.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// System
#include <numeric>
#include <limits>


using namespace std::chrono_literals;


struct GoalStatus
{
    bool goal_sent_ = false;
    Eigen::Vector2d goal_;
}; // struct GoalStatus


class HuskyTestNode : public rclcpp::Node
{
public:

    HuskyTestNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~HuskyTestNode() override = default;

    /******************* Callbacks *******************/
    /** @brief Callback for the timer that runs the main loop of the node.
     * 
     * This function is called periodically to execute the main logic of the node.
     */
    void run();
    
    /** @brief Callback for the fitness function used in the optimization process.
     * 
     * This function calculates the fitness of a given solution based on the current state of the Husky.
     * 
     * @param dv The decision vector representing the solution to evaluate.
     * @return A vector containing the fitness values for the solution.
     */
    pagmo::vector_double huskyFitness(const pagmo::vector_double& dv);

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

    /******************* User-Defined methods *******************/
    /** @brief Converts a pagmo vector_double to a vector of ControlPoint objects.
     * 
     * This function converts a pagmo vector_double to a vector of ControlPoint objects for further processing.
     * 
     * @param dv The pagmo vector_double to convert.
     * @return A vector of ControlPoint objects.
     */
    std::vector<arena_core::ControlPoint<double, 3>> pagmoDVToControlPoints(const pagmo::vector_double& dv) const;

    Eigen::MatrixXd addControlPointsToPath(const Eigen::MatrixXd& initial_control_points, const int nb_of_control_points) const;

    /** @brief Uses ompl_wrapper to plan initial paths for the ARENA optimization process
     * 
     * This function initializes the planning process for the Husky by generating initial paths, therefore generating intial control points.
     */
    void initializerPlanning();

    /** @brief Runs the ARENA optimization process.
     * 
     * This function executes the ARENA optimization algorithm to find the best path for the Husky according to the defined fitness function
     * and adaptive costs weights.
     */
    void ARENAOptimization();

    // ROS Timers
    rclcpp::TimerBase::SharedPtr run_timer_;

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

    // ROS Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planning_activation_sub_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr global_grid_map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;

    // ROS namespace
    std::string ros_namespace_ = "husky_test_node";

    // ROS Subscriptions Callbacks
    /** @brief Callback for the goal pose subscription.
     * This function is called when a new goal pose is received.
     * @param msg The message containing the goal pose.
     */
    void goalPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /** @brief Callback for the planning activation subscription.
     * This function is called when a planning activation message is received.
     * @param msg The message containing the planning activation status.
     */
    void planningActivationCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /** @brief Callback for the global grid_map subscription.
     * This function is called when a new global grid_map is received.
     * @param msg The message containing the global grid_map.
     */
    void globalGridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

    /** @brief Callback for the robot pose subscription.
    * This function is called when a new robot pose is received.
    * @param msg The message containing the robot pose.
    */
    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /****************** User-Defined Attributes *******************/
    arena_demos::HuskyNurbsAnalyzerConfig husky_config;
    arena_core::EvalNurbsOutput husky_output_;
    std::shared_ptr<arena_demos::HuskyNurbsAnalyzer> husky_nurbs_analyzer_;
    std::shared_ptr<arena_demos::TraversabilityCostmap> traversability_mapping_;
    std::shared_ptr<arena_core::Nurbs<3>> nurbs_;
    std::shared_ptr<Eigen::MatrixXd> arena_path_;
    bool planning_activated_ = false;
    bool path_planned_ = false;
    GoalStatus planning_goal_;

    // For the OMPL planner
    std::shared_ptr<arena_core::OMPLPlanner> ompl_planner_;
    std::vector<Eigen::MatrixXd> initial_paths_;
    Eigen::Vector2d start_point_;
    Eigen::Vector3d current_robot_pose_;
    Eigen::Vector2d goal_point_;

    // For optimization
    size_t population_size_ = 0;

}; // HuskyTestNode


// ROS subscriptions callback implementations
void HuskyTestNode::goalPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    planning_goal_.goal_ = Eigen::Vector2d(msg->point.x, msg->point.y);
    planning_goal_.goal_sent_ = true;

    ompl_planner_->setGoal(planning_goal_.goal_);
    goal_point_ = planning_goal_.goal_;
}

void HuskyTestNode::planningActivationCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    planning_activated_ = msg->data;
    planning_goal_.goal_sent_ = false;
}

void HuskyTestNode::globalGridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
    auto global_grid_map = std::make_shared<grid_map::GridMap>();
    grid_map::GridMapRosConverter::fromMessage(*msg, *global_grid_map);

    if (traversability_mapping_)
        traversability_mapping_->setGlobalMap(global_grid_map);
}

void HuskyTestNode::robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_robot_pose_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    start_point_ = current_robot_pose_.head<2>();
}

// ROS publishers implementations
void HuskyTestNode::publishARENAPath()
{
    if (!arena_path_ || arena_path_->cols() == 0)
        return;

    visualization_msgs::msg::MarkerArray arena_markers;
    visualization_msgs::msg::Marker path_marker, velocities_marker;

    path_marker.header.frame_id = "world";
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
        Eigen::Vector2d point_2d = Eigen::Vector2d(p.x, p.y);
        
        // Get the elevation from the traversability mapping
        auto cap = traversability_mapping_->getCapability<arena_demos::TraversabilityCostCapability>();
        double elevation = cap->getElevationAt(point_2d);
        if (!std::isfinite(elevation))
        {
            p.z = 0.0;
        }
        else
            p.z = elevation;

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
        velocity_sphere.color.r = fabs((*arena_path_)(2, i) / husky_config.robot_max_speed_);
        velocity_sphere.color.g = 1.0 - fabs((*arena_path_)(2, i) / husky_config.robot_max_speed_);
        velocity_sphere.color.b = 0.0f;
        velocity_sphere.color.a = 1.0f; // Fully opaque

        velocities_marker.points.push_back(p);
        velocities_marker.colors.push_back(velocity_sphere.color);
    }
    arena_markers.markers.push_back(path_marker);
    arena_markers.markers.push_back(velocities_marker);

    arena_path_pub_->publish(arena_markers);
}

void HuskyTestNode::publishControlPoints()
{
    if (!nurbs_)
        return;

    visualization_msgs::msg::Marker control_points_marker;
    control_points_marker.header.frame_id = "world";
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
        Eigen::Vector2d point_2d = Eigen::Vector2d(p.x, p.y);
        
        // Get the elevation from the traversability mapping
        auto cap = traversability_mapping_->getCapability<arena_demos::TraversabilityCostCapability>();
        double elevation = cap->getElevationAt(point_2d);
        if (!std::isfinite(elevation))
        {
            p.z = 0.0;
        }
        else
            p.z = elevation;

        control_points_marker.points.push_back(p);
        control_points_marker.colors.push_back(control_points_marker.color);
    }

    arena_control_points_pub_->publish(control_points_marker);
}

void HuskyTestNode::publishOMPLPlannerPaths()
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
        path_marker.header.frame_id = "world";
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
            Eigen::Vector2d point_2d = Eigen::Vector2d(p.x, p.y);

            // Get the elevation from the traversability mapping
            auto cap = traversability_mapping_->getCapability<arena_demos::TraversabilityCostCapability>();
            double elevation = cap->getElevationAt(point_2d);
            if (!std::isfinite(elevation))
            {
                p.z = 0.0;
            }
            else
                p.z = elevation;

            path_marker.points.push_back(p);
        }

        initial_paths_markers.markers.push_back(path_marker);
    }

    ompl_planner_pub_->publish(initial_paths_markers);
}

void HuskyTestNode::publishSolutionSet(const pagmo::population& pop)
{
    // Publish solution set as a marker array of paths
    visualization_msgs::msg::MarkerArray solution_set_marker_array;

    // Get the max fitness values and store them for normalization and path coloring
    double max_time = 0.0;
    double min_time = std::numeric_limits<double>::max();
    double max_safety = 0.0;
    double min_safety = std::numeric_limits<double>::max();
    /*double max_energy = 0.0;
    double min_energy = std::numeric_limits<double>::max();*/
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

        /*if (fitness[2] > max_energy)
            max_energy = fitness[2];
        if (fitness[2] < min_energy)
            min_energy = fitness[2];*/
    }

    double min_value = 0.0;
    double max_value = std::max({max_time, max_safety});//, max_energy});

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

    /*auto rescale_energy = [min_value, max_value, min_energy, max_energy](double value) -> double
    {
        /*if (max_energy == min_energy)
            return min_value;
        //return min_value + (max_value - min_value) * (value - min_energy) / (max_energy - min_energy);
        return (value - min_energy) / (max_energy - min_energy);
    };*/

    for (int i = 0; i < pop.size(); i++)
    {
        visualization_msgs::msg::Marker pose_marker;
        pose_marker.header.frame_id = "world"; 
        pose_marker.header.stamp = this->now();
        pose_marker.ns = "solution_set";
        pose_marker.id = i;
        pose_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        pose_marker.action = visualization_msgs::msg::Marker::ADD;

        std::vector<arena_core::ControlPoint<double, 3>> control_points = pagmoDVToControlPoints(pop.get_x()[i]);
        nurbs_->setControlPoints(control_points);
        Eigen::MatrixXd curve_points = nurbs_->evaluate();
        
        for (int j = 0; j < curve_points.cols(); j++)
        {
            geometry_msgs::msg::Point a_point;
            a_point.x = curve_points(0, j);
            a_point.y = curve_points(1, j);

            Eigen::Vector2d point_2d = Eigen::Vector2d(a_point.x, a_point.y);
            
            // Get the elevation from the traversability mapping
            auto cap = traversability_mapping_->getCapability<arena_demos::TraversabilityCostCapability>();
            double elevation = cap->getElevationAt(point_2d);
            if (!std::isfinite(elevation))
            {
                a_point.z = 0.0;
            }
            else
                a_point.z = elevation;
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
        /*pose_marker.color.b = 1 - rescale_energy(fitness[2]);// / max_value;
        if (std::isinf(pose_marker.color.b) || std::isnan(pose_marker.color.b) || std::isinf(-pose_marker.color.b) || std::isnan(-pose_marker.color.b))
            pose_marker.color.b = 1.0;*/
        pose_marker.color.b = 0.0;

        solution_set_marker_array.markers.push_back(pose_marker);
    }

    solution_set_pub_->publish(solution_set_marker_array);
}

std::vector<arena_core::ControlPoint<double, 3>> HuskyTestNode::pagmoDVToControlPoints(const pagmo::vector_double& dv) const
{
    std::vector<arena_core::ControlPoint<double, 3>> control_points;

    // Push first control point
    Eigen::VectorXd start(3);
    start << start_point_(0), start_point_(1), 0.0; // Assuming the 3th dimension is velocity
    control_points.push_back(arena_core::ControlPoint<double, 3>(start, dv[0]));

    for (int i = 1; i < dv.size() - 1; i += 4)
    {
        Eigen::VectorXd cp(3);
        cp << dv[i], dv[i + 1], dv[i + 2]; // Assuming the 3th dimension is velocity
        control_points.push_back(arena_core::ControlPoint<double, 3>(cp, dv[i + 3]));
    }

    // Push the last control point
    Eigen::VectorXd end(3);
    end << goal_point_(0), goal_point_(1), 0.0; // Assuming the 3th dimension is velocity
    control_points.push_back(arena_core::ControlPoint<double, 3>(end, dv[dv.size() - 1]));

    return control_points;
}

Eigen::MatrixXd HuskyTestNode::addControlPointsToPath(const Eigen::MatrixXd& initial_control_points, const int nb_of_control_points) const
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

void HuskyTestNode::initializerPlanning()
{
    if (!ompl_planner_)
    {
        RCLCPP_ERROR(get_logger(), "OMPL Planner is not initialized.");
        return;
    }

    path_planned_ = false;
    initial_paths_.clear();

    if (arena_path_)
    {
        if (arena_path_->cols() > 0)
        {
            // Publish empty path to clear previous visualizations
            visualization_msgs::msg::MarkerArray empty_path;
            visualization_msgs::msg::Marker empty_marker;
            empty_marker.header.frame_id = "world";
            empty_marker.header.stamp = this->now();

            arena_path_pub_->publish(empty_path);
        }
    }

    // Set the bounds for the OMPL planner
    Eigen::MatrixXd bounds = traversability_mapping_->getMapBounds();
    Eigen::Vector2d min_bounds = bounds.row(0).transpose();
    Eigen::Vector2d max_bounds = bounds.row(1).transpose();

    ompl_planner_->setBounds(min_bounds, max_bounds);

    // Set the start state for the OMPL planner
    ompl_planner_->setStart(start_point_);

    int successive_failed_planning_attempts = 0;
    // Start timer for initialization process
    auto start_time = std::chrono::steady_clock::now();
    while (initial_paths_.size() < population_size_)
    {
        if (ompl_planner_->plan() == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            successive_failed_planning_attempts = 0;

            ompl::geometric::PathGeometric* path = ompl_planner_->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();
            if (path)
            {
                Eigen::MatrixXd path_matrix(3, path->getStateCount());
                for (size_t i = 0; i < path->getStateCount(); ++i)
                {
                    const ompl::base::State* state = path->getState(i);
                    const auto* real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
                    path_matrix(0, i) = real_state->values[0];
                    path_matrix(1, i) = real_state->values[1];
                    if (i > 0 && i < path->getStateCount() - 1)
                    {
                        path_matrix(2, i) = husky_config.robot_max_speed_ * 0.25; // Assuming the 3th dimension is velocity
                    }
                    else
                        path_matrix(2, i) = 0.0; // No velocity at the start and end points
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
            return;
        }
    }

    // Stop timer for initialization process
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(get_logger(), "OMPL Planner initialized in %ld milliseconds with %zu paths generated.", duration, initial_paths_.size());

    // Publish the initial paths
    publishOMPLPlannerPaths();

    // Start optimization process
    ARENAOptimization();
}

pagmo::vector_double HuskyTestNode::huskyFitness(const pagmo::vector_double& dv)
{
    if (!husky_nurbs_analyzer_)
        throw std::runtime_error("huskyFitness => Husky NURBS Analyzer is not initialized.");

    if (!nurbs_)
        throw std::runtime_error("huskyFitness => NURBS is not initialized.");

    husky_output_.fitness_array_[0] = std::numeric_limits<double>::max();
    husky_output_.fitness_array_[1] = std::numeric_limits<double>::max();
    husky_output_.constraint_array_[0] = std::numeric_limits<double>::max();
    
    std::vector<arena_core::ControlPoint<double, 3>> control_points = pagmoDVToControlPoints(dv);
    nurbs_->setControlPoints(control_points);
    Eigen::MatrixXd pt = nurbs_->evaluate();

    // Evaluate the NURBS curve using the Husky NURBS Analyzer
    husky_nurbs_analyzer_->eval(pt, husky_output_);
    
    if (husky_output_.constraint_array_[0] > 0.5)
    {
        // Reject the solution if it's in collision with the environment
        husky_output_.fitness_array_[0] = std::numeric_limits<double>::max();
        husky_output_.fitness_array_[1] = std::numeric_limits<double>::max();
    }

    return husky_output_.fitness_array_;
}

void HuskyTestNode::ARENAOptimization()
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
            return;
        }
    }

    // Create pagmo problem
    std::function<pagmo::vector_double(const pagmo::vector_double &)> fitness_eval_cb = std::bind(&HuskyTestNode::huskyFitness, this, std::placeholders::_1);
    std::pair<Eigen::VectorXd, Eigen::VectorXd> bounds = ompl_planner_->getBounds();
    double* x_bounds = new double[2] {bounds.first(0), bounds.second(0)};
    double* y_bounds = new double[2] {bounds.first(1), bounds.second(1)};
    pagmo::problem prob_husky{pagmo::husky_problem((nb_of_control_points - 2) * 3 + nb_of_control_points,
                                                           2u,
                                                           &fitness_eval_cb,
                                                           x_bounds,
                                                           y_bounds,
                                                           husky_config.robot_max_speed_)};
    RCLCPP_INFO(get_logger(), "ARENAOptimization => Problem created");
    std::cout << prob_husky << std::endl;

    int nb_of_generations = this->get_parameter("optimization.NSGA-II.generations").as_int();
    std::function<void(const pagmo::population&)> show_set_callback = std::bind(&HuskyTestNode::publishSolutionSet, this, std::placeholders::_1);
    pagmo::algorithm nsga2{pagmo::nsga2(nb_of_generations, 0.95, 10.0, 0.01, 50.0, pagmo::random_device::next(), &show_set_callback)};
    pagmo::vector_double adaptive_matrix = pagmo::vector_double(prob_husky.get_nf(), 0.0);
    nsga2.set_adaptive_matrix(adaptive_matrix);

    pagmo::population pop_husky{prob_husky, population_size_};
    pop_husky.clear();

    for (size_t i = 0; i < initial_paths_.size(); ++i)
    {
        pagmo::vector_double dv;
        dv.push_back(1.0); // Weight of the first control point
        for (size_t j = 1; j < nb_of_control_points - 1; j++)
        {
            dv.push_back(initial_paths_[i](0, j)); // X coordinate
            dv.push_back(initial_paths_[i](1, j)); // Y coordinate
            dv.push_back(initial_paths_[i](2, j)); // Velocity
            dv.push_back(1.0); // Weight of the control point
        }
        dv.push_back(1.0); // Weight of the last control point
        pagmo::vector_double fitness = prob_husky.fitness(dv);
        pop_husky.push_back(dv, fitness);
    }

    // Make sure we're up to date with sample size
    husky_config.base_config.sample_size = this->get_parameter("optimization.sample_size").as_int();

    pop_husky = nsga2.evolve(pop_husky);

    // End timer for the optimization process
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(get_logger(), "ARENAOptimization => Optimization completed in %ld milliseconds.", duration);

    // Filter trajectories that doesn't respect the constraints
    std::vector<int> to_remove;
    for (int i = 0; i < pop_husky.size(); i++)
    {
        pagmo::vector_double fitness = pop_husky.get_f()[i];
        if (fitness[0] == std::numeric_limits<double>::max() || fitness[1] == std::numeric_limits<double>::max())
            to_remove.push_back(i);
    }

    pagmo::population pop_husky_filtered(prob_husky, population_size_ - to_remove.size());
    int j = 0;
    for (int i = 0; i < pop_husky.size(); i++)
    {
        if (std::find(to_remove.begin(), to_remove.end(), i) == to_remove.end())
        {
            pagmo::vector_double dv = pop_husky.get_x()[i];
            pop_husky_filtered.set_x(j, dv);
            j++;
        }
    }

    // Apply adaptive voting algorithm to select the best solution
    std::vector<double> costs_weights;
    costs_weights.push_back(this->get_parameter("optimization.adaptive_costs_weights.time").as_double());
    costs_weights.push_back(this->get_parameter("optimization.adaptive_costs_weights.safety").as_double());
    //costs_weights.push_back(this->get_parameter("optimization.adaptive_costs_weights.energy").as_double());
    int best_idx = arena_core::adaptive_voting_algorithm::getBetterCandidateIndex(pop_husky_filtered.get_f(), costs_weights);
    pagmo::vector_double best_solution = pop_husky_filtered.get_x()[best_idx];

    // Convert the best solution to control points
    std::vector<arena_core::ControlPoint<double, 3>> best_control_points = pagmoDVToControlPoints(best_solution);
    nurbs_->setControlPoints(best_control_points);
    arena_path_ = std::make_shared<Eigen::MatrixXd>(nurbs_->evaluate());
    publishARENAPath();
    publishControlPoints();

    path_planned_ = true;
    planning_activated_ = false;
    planning_goal_.goal_sent_ = false;
}

bool HuskyTestNode::isCurrentPathSafe() const
{
    if (!path_planned_)
        return true;
    
    if (!arena_path_)
        return false;

    if (arena_path_->cols() == 0)
        return false;
    
    if (arena_path_->cols() != husky_config.base_config.sample_size)
    {
        RCLCPP_WARN(get_logger(), "Current path size does not match the sample size. Expected: %u, Got: %zu",
                    husky_config.base_config.sample_size, arena_path_->cols());
        return false; // Path size mismatch
    }

    // Check if the path is safe by checking for collisions with the octree
    for (int i = 0; i < arena_path_->cols() - 1; ++i)
    {
        Eigen::Vector2d start = arena_path_->col(i).head(2);
        Eigen::Vector2d end = arena_path_->col(i + 1).head(2);

        if (traversability_mapping_->isOccupiedRayTracing(start, end))
        {
            RCLCPP_WARN(get_logger(), "Collision detected along the path segment from (%f, %f) to (%f, %f).",
                        start.x(), start.y(), end.x(), end.y());
            return false; // Collision detected
        }
    }
    
    return true; // No collisions detected along the path
}

// Main logic of the node
void HuskyTestNode::run()
{
    if (!husky_nurbs_analyzer_)
    {
        RCLCPP_ERROR(get_logger(), "Husky NURBS Analyzer is not initialized.");
        return;
    }

    // Perform the main logic of the node
    if ((planning_activated_ && planning_goal_.goal_sent_) || !isCurrentPathSafe())
        initializerPlanning();
}

HuskyTestNode::HuskyTestNode(rclcpp::NodeOptions options)
: Node("husky_test_node", options), husky_config(50), traversability_mapping_(std::make_shared<arena_demos::TraversabilityCostmap>()),
  husky_nurbs_analyzer_(nullptr), ompl_planner_(nullptr), nurbs_(nullptr), arena_path_(nullptr)
{
    // ROS Publisher Initialization
    arena_control_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(ros_namespace_ + "/arena_control_points", 10);
    arena_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(ros_namespace_ + "/arena_path", 10);
    ompl_planner_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(ros_namespace_ + "/ompl_planner_paths", 10);
    solution_set_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(ros_namespace_ + "/solution_set", 10);

    // ROS Subscription Initialization
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(ros_namespace_ + "/goal_pose", 10, std::bind(&HuskyTestNode::goalPoseCallback, this, std::placeholders::_1));
    planning_activation_sub_ = this->create_subscription<std_msgs::msg::Bool>(ros_namespace_ + "/planning_activation", 10, std::bind(&HuskyTestNode::planningActivationCallback, this, std::placeholders::_1));
    global_grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>("/global_grid_map", 10, std::bind(&HuskyTestNode::globalGridMapCallback, this, std::placeholders::_1));
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/groundTruth/poseStamped", 10, std::bind(&HuskyTestNode::robotPoseCallback, this, std::placeholders::_1));
    
    population_size_ = this->get_parameter("optimization.NSGA-II.population_size").as_int();
    // Make sure the population size is divisible by 4
    if (population_size_ % 4 != 0)
        population_size_ += 4 - (population_size_ % 4);

    // Define the nurbs_analyzer configurations with ros parameters
    husky_config.robot_mass_ = this->get_parameter("robot.mass").as_double(); // kg
    husky_config.robot_max_speed_ = this->get_parameter("robot.max_speed").as_double(); // m/s
    husky_config.robot_max_acceleration_ = this->get_parameter("robot.max_acceleration").as_double(); // m/s^2

    husky_nurbs_analyzer_ = std::make_shared<arena_demos::HuskyNurbsAnalyzer>(traversability_mapping_, husky_config);
    
    // Initialize the nurbs output structure
    husky_output_.fitness_size_ = 2;
    husky_output_.fitness_array_ = std::vector<double>(husky_output_.fitness_size_, std::numeric_limits<double>::max()); // We are optimizing to minimize the fitness values
    husky_output_.constraint_size_ = 1;
    husky_output_.constraint_array_ = std::vector<double>(husky_output_.constraint_size_, std::numeric_limits<double>::max());

    // Initialize the OMPL planner
    ompl_planner_ = std::make_shared<arena_core::OMPLPlanner>();
    double timeout = this->get_parameter("optimization.initialization.timeout").as_double();
    ompl_planner_->setSolvingTimeout(timeout);
    ompl_planner_->setProblemDimensions(2);
    ompl_planner_->getInitializer()->planner_ = std::make_shared<ompl::geometric::RRT>(ompl_planner_->getSpaceInformation());
    double rrt_range = this->get_parameter("optimization.initialization.rrt_range").as_double();
    ompl_planner_->getInitializer()->planner_->as<ompl::geometric::RRT>()->setRange(rrt_range);
    // TODO
    ompl_planner_->getInitializer()->state_validity_checker_ = std::make_shared<arena_core::OMPLStateValidityChecker>(ompl_planner_->getSpaceInformation(), traversability_mapping_);
    ompl_planner_->getInitializer()->optimization_objective_ = nullptr;

    // Initialize the NURBS API
    husky_config.base_config.sample_size = this->get_parameter("optimization.sample_size").as_int();
    nurbs_ = std::make_shared<arena_core::Nurbs<3>>(std::vector<arena_core::ControlPoint<double, 3>>(), husky_config.base_config.sample_size);

    // ROS Timer Initialization
    // Timer for the main loop at 50 Hz
    run_timer_ = this->create_wall_timer(20ms, std::bind(&HuskyTestNode::run, this));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<HuskyTestNode>(options));
    rclcpp::shutdown();
    return 0;
}
