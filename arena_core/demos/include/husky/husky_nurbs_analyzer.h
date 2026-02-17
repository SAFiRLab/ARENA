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

#pragma once

// Local
#include "arena_core/math/optimization/evaluation/INurbsAnalyzer.h"
#include "husky/mapping/traversability_costmap.hpp"

// System
#include <memory>
#include <unordered_map>
#include <limits>

// Eternal Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_demos
{

/**
 * @brief Configuration structure for HuskyNurbsAnalyzer.
 * This structure holds the configuration parameters for the HuskyNurbsAnalyzer.
 */
struct HuskyNurbsAnalyzerConfig
{
    arena_core::NurbsAnalyzerConfig base_config; // Base configuration for NURBS analyzers

    double robot_mass_; // Mass of the robot in kg

    double robot_max_speed_; // Maximum speed of the robot in m/s
    double robot_max_acceleration_; // Maximum acceleration of the robot in m/s^2

    HuskyNurbsAnalyzerConfig(unsigned int a_sample_size)
        : base_config(a_sample_size), robot_mass_(0.0)
    {}

}; // HuskyNurbsAnalyzerConfig


struct HuskyEvalNurbsOutput
{
    // Time cost outputs
    double distance_output_; // Total distance covered by the NURBS curve
    double time_output_; // Total time taken for the NURBS curve

    // Safety cost outputs
    double total_collision_cost_; // Total collision cost
    double max_collision_cost_; // Maximum collision cost
    double max_occupancy_; // Maximum occupancy detected in the environment
    double nb_of_collision_checks_; // Number of collision checks

    // Energy cost outputs
    double total_energy_; // Total energy consumed by the NURBS curve
    double max_energy_; // Maximum energy consumed by the NURBS curve
    double max_acceleration_; // Maximum acceleration along the NURBS curve
    
    HuskyEvalNurbsOutput() :
        distance_output_(0.0), time_output_(0.0),
        total_collision_cost_(0.0), max_collision_cost_(0.0), max_occupancy_(0.0), nb_of_collision_checks_(0.0),
        total_energy_(0.0), max_energy_(0.0), max_acceleration_(0.0)
        {}
}; // HuskyEvalNurbsOutput


/**
 * @brief HuskyNurbsAnalyzer class for evaluating NURBS curves.
 * This class implements the INurbsAnalyzer interface and provides methods to evaluate NURBS curves
 * with specific configurations and safety checks for the Husky problem.
 */
class HuskyNurbsAnalyzer : public arena_core::INurbsAnalyzer
{
public:
    /**
     * @brief Default constructor for HuskyNurbsAnalyzer.
     */
    HuskyNurbsAnalyzer(std::shared_ptr<arena_demos::TraversabilityCostmap> a_traversability_costmap,
                       const HuskyNurbsAnalyzerConfig& a_config);

    /**
     * @brief Destructor for HuskyNurbsAnalyzer.
     */
    ~HuskyNurbsAnalyzer() final override = default;

    /************* User-defined methods *************/
    /**
     * @brief Evaluate the NURBS curve.
     * This method implements the evaluation of the NURBS curve and populates the output structure with the results.
     *
     * @param a_output Reference to an EvalNurbsOutput structure where results will be stored.
     * @param a_curve_points Matrix of control points for the NURBS curve.
     */
    void eval(const Eigen::MatrixXd& a_curve_points, arena_core::EvalNurbsOutput& a_output) final override;

    /**
     * @brief Evaluate the time cost based on distance and velocity.
     * This method computes the time cost for a given distance and velocity.
     *
     * @param a_distance The distance to evaluate.
     * @param a_velocity The velocity at which the distance is covered.
     */
    void evalTimeCost(const double a_distance, const double a_velocity);
    
    //void evalInsertionCost(Eigen::Vector3d& a_point1, )

    /**
     * @brief Evaluate the safety cost based on the NURBS curve and environment.
     * This method computes the safety cost for the NURBS curve by checking against the environment.
     *
     * @param a_point1 The first point of the NURBS curve segment.
     */
    void evalSafetyCost(const Eigen::Vector2d& a_point1);

private:

    /************* User-defined methods *************/

    /************* User-defined attributes *************/
    std::shared_ptr<arena_demos::TraversabilityCostmap> traversability_mapping_; // Pointer to the costmap mapping for collision checks
    HuskyNurbsAnalyzerConfig husky_config_; // Configuration for the analyzer
    HuskyEvalNurbsOutput husky_output_; // Output structure for NURBS evaluation results

}; // class HuskyNurbsAnalyzer

}; // namespace arena_demos
