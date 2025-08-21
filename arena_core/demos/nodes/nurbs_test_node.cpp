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

// Local
#include "arena_core/math/nurbs.h"
#include "arena_core/math/control_point.h"
#include "arena_core/planning/OMPLPlanner.h"
#include "arena_core/geometry/ompl_state_validity_checker.h"
#include "linedrone/linedrone_nurbs_analyzer.h"
#include "linedrone/costmap_mapping.h"
// OLD locals
#include "linedrone/nurbs_analyzer_old.h"
#include "linedrone/old_nurbs.h"

// External Libraries
// Eigen
#include <Eigen/Dense>
// OMPL
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>


using namespace std::chrono_literals;

class NurbsVisualizerNode : public rclcpp::Node
{
public:
    NurbsVisualizerNode()
    : Node("nurbs_visualizer_node"), linedrone_config(50), costmap_mapping_(std::make_shared<arena_demos::CostmapMapping>()),
    color_octree_(std::make_shared<octomap::ColorOcTree>(0.5)),
    linedrone_nurbs_analyzer_(nullptr), ompl_planner_(std::make_shared<arena_core::OMPLPlanner>())
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("nurbs_marker", 10);
        color_octree_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("color_octree_marker", 10);
        fitted_curve_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fitted_curve_marker", 10);
        control_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("control_points_marker", 10);
        old_nurbs_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("old_nurbs_marker", 10);
        initializer_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("initializer_marker", 10);

        if (color_octree_->readBinary(map_file_path_))
        {
            RCLCPP_INFO(get_logger(), "Octomap loaded successfully from %s", map_file_path_.c_str());
        }
        else
            RCLCPP_ERROR(get_logger(), "Failed to load octomap from %s", map_file_path_.c_str());

        costmap_mapping_->setColorOctree(color_octree_);

        std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(0.5);
        if (octree->readBinary(map_file_path_))
        {
            RCLCPP_INFO(get_logger(), "Octree loaded successfully from %s", map_file_path_.c_str());
        }
        else
            RCLCPP_ERROR(get_logger(), "Failed to load octree from %s", map_file_path_.c_str());

        costmap_mapping_->setOctree(octree);

        // Define the nurbs_analyzer configurations
        linedrone_config.robot_mass_ = 25.0; // Example mass in kg
        linedrone_config.robot_permanent_power_ascent_ = 3475.0; // Example power in Watts
        linedrone_config.robot_permanent_power_roll_ = 3102.0; // Example power in Watts
        linedrone_config.robot_permanent_power_pitch_ = 3102.0; // Example power in Watts
        linedrone_config.robot_permanent_power_descent_ = 3080.0; // Example power in Watts

        linedrone_config.solveQuadraticSurfaceCoefficients();

        linedrone_nurbs_analyzer_ = std::shared_ptr<arena_demos::LinedroneNurbsAnalyzer>(
            new arena_demos::LinedroneNurbsAnalyzer(costmap_mapping_, linedrone_config, {}));

        // Old NURBS analyzer setup
        pipeline_config_old_.drone_speed_ = 2.0; // Example speed in m/s
        pipeline_config_old_.drone_acceleration_ = 2.2; // Example acceleration in m/s^2
        pipeline_config_old_.drone_mass_ = 25.0; // Example mass in kg
        pipeline_config_old_.drone_permanent_power_.roll_ = 3102.0; // Example power in Watts
        pipeline_config_old_.drone_permanent_power_.pitch_ = 3102.0; // Example power in Watts
        pipeline_config_old_.drone_permanent_power_.ascent_ = 3475.0; // Example power in Watts
        pipeline_config_old_.drone_permanent_power_.descent_ = 3080.0; // Example power in Watts
        pipeline_config_old_.rrt_rope_delta_ = 5.0; // Example rope delta in meters
        pipeline_config_old_.sample_size_ = 50; // Example sample size for NUR
        pipeline_config_old_.population_size_ = 40; // Example population size for optimization
        pipeline_config_old_.max_generations_ = 1000; // Example max generations for optimization
        pipeline_config_old_.mutation_distribution_index_ = 20.0; // Example mutation distribution index
        pipeline_config_old_.crossover_distribution_index_ = 20.0; // Example crossover distribution index
        pipeline_config_old_.crossover_probability_ = 0.9; // Example crossover probability
        pipeline_config_old_.final_cost_time_ = 0.5; // Example final cost time weight
        pipeline_config_old_.final_cost_security_ = 0.3; // Example final cost security weight
        pipeline_config_old_.final_cost_energy_ = 0.2; // Example final cost energy weight
        pipeline_config_old_.solveQuadraticSurfaceCoefficients();

        // Example NURBS control points
        Eigen::VectorXd p1(4);
        p1 << 0, 0, 0, 0;
        Eigen::VectorXd p2(4);
        p2 << 1, 2, 3, 0.5;
        Eigen::VectorXd p3(4);
        p3 << 2, 0, 3, 0.5;
        Eigen::VectorXd p4(4);
        p4 << 3, 2, -5, 0.5;
        Eigen::VectorXd p5(4);
        p5 << 4, 0, 2, 0.5;
        Eigen::VectorXd p6(4);
        p6 << 5, 2, 1, 0.5;
        Eigen::VectorXd p7(4);
        p7 << 6, 0, 0, 0.5;
        Eigen::VectorXd p8(4);
        p8 << 7, 2, -1, 0.5;
        Eigen::VectorXd p9(4);
        p9 << 8, 0, -2, 0.5;
        Eigen::VectorXd p10(4);
        p10 << 9, 2, -3, 0.5;
        Eigen::VectorXd p11(4);
        p11 << 10, 0, -4, 0.5;
        Eigen::VectorXd p12(4);
        p12 << 11, 2, -5, 0.5;
        Eigen::VectorXd p13(4);
        p13 << 12, 0, 10, 0;

        control_points_.push_back(arena_core::ControlPoint<double, 4>(p1));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p2));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p3));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p4));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p5));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p6));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p7));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p8));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p9));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p10));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p11));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p12));
        control_points_.push_back(arena_core::ControlPoint<double, 4>(p13));

        std::vector<Eigen::VectorXd> control_points_vec;
        control_points_vec.push_back(p1);
        control_points_vec.push_back(p2);
        control_points_vec.push_back(p3);
        control_points_vec.push_back(p4);
        control_points_vec.push_back(p5);
        control_points_vec.push_back(p6);
        control_points_vec.push_back(p7);
        control_points_vec.push_back(p8);
        control_points_vec.push_back(p9);
        control_points_vec.push_back(p10);
        control_points_vec.push_back(p11);
        control_points_vec.push_back(p12);
        control_points_vec.push_back(p13);

        // Example weights
        control_points_[1].setW(5.0);
        control_points_[2].setW(10.0);
        control_points_[3].setW(7.0);

        int degree = 5;
        sample_size_ = 50;
        nurbs_ = std::make_shared<arena_core::Nurbs<4>>(control_points_, sample_size_, degree);

        // Initialize the old NURBS
        std::vector<double> weights = { 1.0, 5.0, 10.0, 7.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        old_nurbs_ = std::make_shared<arena_demos::OldNurbs>(control_points_vec, sample_size_, weights, degree);

        // Initialize the OMPL planner
        ompl_planner_->setSolvingTimeout(1.0);
        ompl_planner_->setProblemDimensions(3);
        ompl_planner_->getInitializer()->planner_ = std::make_shared<ompl::geometric::RRT>(ompl_planner_->getSpaceInformation());
        ompl_planner_->getInitializer()->planner_->as<ompl::geometric::RRT>()->setRange(2.0);
        ompl_planner_->getInitializer()->state_validity_checker_ = std::make_shared<arena_core::OMPLStateValidityChecker>(ompl_planner_->getSpaceInformation(), costmap_mapping_);
        ompl_planner_->getInitializer()->optimization_objective_ = nullptr;

        timer_ = this->create_wall_timer(500ms, std::bind(&NurbsVisualizerNode::publishAll, this));
        //derivatives_timer_ = this->create_wall_timer(1000ms, std::bind(&NurbsVisualizerNode::publishDerivatives, this));
        octomap_timer_ = this->create_wall_timer(1000ms, std::bind(&NurbsVisualizerNode::publishColorOctree, this));
    }

private:
    void publish_marker()
    {
        visualization_msgs::msg::Marker marker, old_nurbs_marker, fitted_curve_marker;
        marker.header.frame_id = "map";
        fitted_curve_marker.header.frame_id = "map";
        old_nurbs_marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        fitted_curve_marker.header.stamp = this->now();
        old_nurbs_marker.header.stamp = this->now();
        marker.ns = "nurbs";
        fitted_curve_marker.ns = "fitted_curve";
        old_nurbs_marker.ns = "old_nurbs";
        marker.id = 0;
        fitted_curve_marker.id = 0;
        old_nurbs_marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        fitted_curve_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        old_nurbs_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        fitted_curve_marker.action = visualization_msgs::msg::Marker::ADD;
        old_nurbs_marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05;
        fitted_curve_marker.scale.x = 0.05;
        old_nurbs_marker.scale.x = 0.05;
        fitted_curve_marker.color.r = 1.0;
        old_nurbs_marker.color.r = 0.0;
        fitted_curve_marker.color.g = 0.0;
        old_nurbs_marker.color.g = 0.0;
        fitted_curve_marker.color.b = 0.0;
        old_nurbs_marker.color.b = 1.0;
        fitted_curve_marker.color.a = 1.0;
        old_nurbs_marker.color.a = 1.0;

        // Evalute the time it takes to generate the NURBS curve
        auto start = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd pt = nurbs_->evaluate();
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
        RCLCPP_INFO(this->get_logger(), "NURBS generation took: %f ms", elapsed_ms.count());

        // Evaluate the time it takes to evaluate the old NURBS
        start = std::chrono::high_resolution_clock::now();
        Eigen::VectorXd* old_pt = old_nurbs_->evaluate();
        end = std::chrono::high_resolution_clock::now();
        elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
        RCLCPP_INFO(this->get_logger(), "Old NURBS generation took: %f ms", elapsed_ms.count());

        // Evaluate the time it takes to evaluate the NURBS curve
        start = std::chrono::high_resolution_clock::now();
        arena_core::EvalNurbsOutput* eval_output = new arena_core::EvalNurbsOutput();
        eval_output->fitness_size_ = 3;
        eval_output->fitness_array_ = std::vector<double>(eval_output->fitness_size_, 0.0);
        eval_output->constraint_size_ = 2;
        eval_output->constraint_array_ = std::vector<double>(eval_output->constraint_size_, 0.0);
        linedrone_nurbs_analyzer_->eval(pt, *eval_output);
        end = std::chrono::high_resolution_clock::now();
        elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
        RCLCPP_INFO(this->get_logger(), "NURBS evaluation took: %f ms", elapsed_ms.count());
        

        // Evaluate the time it takes to evaluate the old NURBS
        start = std::chrono::high_resolution_clock::now();
        arena_demos::EvalNurbsOutput old_eval_output;
        old_eval_output = arena_demos::getEvalNurbs(old_pt, color_octree_.get(), sample_size_, pipeline_config_old_, {});
        end = std::chrono::high_resolution_clock::now();
        elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
        RCLCPP_INFO(this->get_logger(), "Old NURBS evaluation took: %f ms", elapsed_ms.count());
        double safety_cost = (old_eval_output.safety_output_.total_collision_cost_ / old_eval_output.safety_output_.nb_of_collision_checks_) + old_eval_output.safety_output_.max_collision_cost_;

        RCLCPP_INFO(this->get_logger(), "NURBS fitness: %f, %f, %f", eval_output->fitness_array_[0], eval_output->fitness_array_[1], eval_output->fitness_array_[2]);
        RCLCPP_INFO(this->get_logger(), "NURBS constraints: %f, %f", eval_output->constraint_array_[0], eval_output->constraint_array_[1]);
        RCLCPP_INFO(this->get_logger(), "Old NURBS fitness: %f, %f, %f", old_eval_output.time_output_, safety_cost, old_eval_output.energy_output_.total_energy_);
        RCLCPP_INFO(this->get_logger(), "Old NURBS constraints: %f, %f", old_eval_output.energy_output_.max_acceleration_, old_eval_output.safety_output_.max_occupancy_);

        // Fit a polynomial curve on the points
        start = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd fitted_curve_coeffs = nurbs_->fitPolynomialCurve(pt);
        end = std::chrono::high_resolution_clock::now();
        elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
        //RCLCPP_INFO(this->get_logger(), "Polynomial curve fitting took: %f ms", elapsed_ms.count());
        RCLCPP_INFO(this->get_logger(), "*************************************************");

        // Sample the curve and add points and check if the points are in collision with the octree
        for (int i = 0; i < sample_size_; ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = pt(0, i);
            p.y = pt(1, i);
            p.z = pt(2, i);
            marker.points.push_back(p);

            // Check if the point is in collision with the octree
            octomap::point3d point(p.x, p.y, p.z);
            octomap::ColorOcTreeNode* node = color_octree_->search(point);
            if (node)
            {
                if (color_octree_->isNodeOccupied(node))
                {
                    std_msgs::msg::ColorRGBA color;
                    color.r = 1.0;
                    color.g = 0.0;
                    color.b = 0.0;
                    color.a = 1.0;
                    marker.colors.push_back(color);
                    continue;
                }
            }

            std_msgs::msg::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;
            marker.colors.push_back(color);
        }

        // Add points from the old NURBS
        for (int i = 0; i < sample_size_; ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = old_pt[i][0];
            p.y = old_pt[i][1];
            p.z = old_pt[i][2];
            old_nurbs_marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
        old_nurbs_pub_->publish(old_nurbs_marker);

        delete[] old_pt;

        // Sample the fitted curve and add points
        Eigen::VectorXd fitted_curve_sample = nurbs_->estimateParamByArcLength(pt);
        for (int i = 0; i < fitted_curve_sample.size(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = nurbs_->evaluatePolynomial(fitted_curve_coeffs, fitted_curve_sample(i))[0];
            p.y = nurbs_->evaluatePolynomial(fitted_curve_coeffs, fitted_curve_sample(i))[1];
            p.z = nurbs_->evaluatePolynomial(fitted_curve_coeffs, fitted_curve_sample(i))[2];
            fitted_curve_marker.points.push_back(p);
        }

        //delete[] pt;
        //delete eval_output;

        fitted_curve_pub_->publish(fitted_curve_marker);

        // OMPL planner test
        // First 3 dimensions of the NURBS points
        Eigen::Map<const Eigen::Vector3d> start_point(pt.col(2).data());
        Eigen::Map<const Eigen::Vector3d> goal(pt.col(sample_size_ - 1).data());
        ompl_planner_->setStart(start_point);
        ompl_planner_->setGoal(goal);

        // Set the bounds for the OMPL planner
        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        Eigen::Vector3d min_bounds = bounds.row(0).transpose();
        Eigen::Vector3d max_bounds = bounds.row(1).transpose();

        ompl_planner_->setBounds(min_bounds, max_bounds);
        if (ompl_planner_->plan() == ompl::base::PlannerStatus::EXACT_SOLUTION || ompl_planner_->plan() == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
        {
            RCLCPP_INFO(this->get_logger(), "OMPL planner found an exact solution");
            ompl::geometric::PathGeometric* path = ompl_planner_->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>();
            visualization_msgs::msg::Marker ompl_marker;
            ompl_marker.header.frame_id = "map";
            ompl_marker.header.stamp = this->now();
            ompl_marker.ns = "ompl_path";
            ompl_marker.id = 0;
            ompl_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            ompl_marker.action = visualization_msgs::msg::Marker::ADD;
            ompl_marker.scale.x = 0.05;
            ompl_marker.color.r = 0.0;
            ompl_marker.color.g = 0.0;
            ompl_marker.color.b = 1.0;
            ompl_marker.color.a = 1.0;

            for (const auto& state : path->getStates())
            {
                geometry_msgs::msg::Point p;
                p.x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
                p.y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
                p.z = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
                ompl_marker.points.push_back(p);
            }

            initializer_pub_->publish(ompl_marker);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "OMPL planner failed to find a solution");
        }
    }

    void publishControlPoints()
    {
        visualization_msgs::msg::Marker control_points_marker;
        control_points_marker.header.frame_id = "map";
        control_points_marker.header.stamp = this->now();
        control_points_marker.ns = "control_points";
        control_points_marker.id = 1;
        control_points_marker.type = visualization_msgs::msg::Marker::POINTS;
        control_points_marker.action = visualization_msgs::msg::Marker::ADD;

        control_points_marker.scale.x = 0.25;
        control_points_marker.scale.y = 0.25;
        control_points_marker.scale.z = 0.25;
        control_points_marker.color.r = 1.0;
        control_points_marker.color.g = 0.0;
        control_points_marker.color.b = 0.0;
        control_points_marker.color.a = 1.0;

        for (const auto& cp : control_points_) {
            geometry_msgs::msg::Point p;
            p.x = cp[0];
            p.y = cp[1];
            p.z = cp[2];
            control_points_marker.points.push_back(p);
        }

        control_points_pub_->publish(control_points_marker);
    }

    void publishAll()
    {
        publish_marker();
        publishControlPoints();
    }

    void publishDerivatives()
    {
        // Evaluate the NURBS curve and its derivatives
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<std::vector<std::vector<double>>> ders = nurbs_->derivatives(3);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        RCLCPP_INFO(this->get_logger(), "NURBS derivative evaluation took: %f ms", elapsed.count());
    }

    void publishColorOctree()
    {
        visualization_msgs::msg::Marker octree_marker;
        octree_marker.header.frame_id = "map";
        octree_marker.header.stamp = this->now();
        octree_marker.ns = "octomap";
        octree_marker.id = 2;
        octree_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        octree_marker.action = visualization_msgs::msg::Marker::ADD;

        octree_marker.scale.x = color_octree_->getResolution();
        octree_marker.scale.y = color_octree_->getResolution();
        octree_marker.scale.z = color_octree_->getResolution();

        for (octomap::ColorOcTree::leaf_iterator it = color_octree_->begin_leafs(), end = color_octree_->end_leafs(); it != end; ++it)
        {
            if (!color_octree_->isNodeOccupied(*it)) {
                continue; // Skip empty nodes
            }

            geometry_msgs::msg::Point p;
            p.x = it.getX();
            p.y = it.getY();
            p.z = it.getZ();

            octree_marker.points.push_back(p);

            std_msgs::msg::ColorRGBA color;
            color.r = it->getColor().r / 255.0;
            color.g = it->getColor().g / 255.0;
            color.b = it->getColor().b / 255.0;
            color.a = 1.0;

            octree_marker.colors.push_back(color);
        }

        color_octree_pub_->publish(octree_marker);
    }

    // ROS Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr color_octree_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fitted_curve_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_points_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr old_nurbs_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr initializer_pub_;

    // ROS Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr derivatives_timer_;
    rclcpp::TimerBase::SharedPtr octomap_timer_;

    // User-defined attributes
    std::vector<arena_core::ControlPoint<double, 4>> control_points_;
    std::shared_ptr<arena_core::Nurbs<4>> nurbs_;
    arena_demos::LinedroneNurbsAnalyzerConfig linedrone_config;
    arena_demos::LinedroneEvalNurbsOutput linedrone_output_;
    std::shared_ptr<arena_demos::LinedroneNurbsAnalyzer> linedrone_nurbs_analyzer_;
    std::shared_ptr<arena_demos::CostmapMapping> costmap_mapping_;
    std::shared_ptr<octomap::ColorOcTree> color_octree_;

    std::string map_file_path_ = "/home/dev_ws/src/arena_core/demos/ressources/saved_octomaps/CL_map_res_50cm.bt";
    int sample_size_;

    // For the OMPL planner
    std::shared_ptr<arena_core::OMPLPlanner> ompl_planner_;

    // For the old NURBS analyzer
    std::shared_ptr<arena_demos::OldNurbs> old_nurbs_;
    arena_demos::PipelineConfig pipeline_config_old_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NurbsVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
