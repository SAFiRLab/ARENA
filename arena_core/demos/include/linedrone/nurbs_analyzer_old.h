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

// System
#include <unordered_map>
#include <vector>
#include <string>

// Eigen
#include <Eigen/Dense>

// Octomap 
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>


namespace arena_core
{
struct OrientedBoundingBoxWrapper;
}; // namespace arena_core

namespace arena_demos
{

/**
 * \brief Pipeline config container
 */
struct PipelineConfig
{
    // Drone configs
    float drone_speed_;
    float drone_acceleration_;
    float drone_mass_;

    struct DronePermanentPower
    {
        float pitch_;
        float roll_;
        float ascent_;
        float descent_;

        Eigen::VectorXd quadratic_surface_coefficients_;
    } drone_permanent_power_;

    // RRT configs
    float rrt_rope_delta_;

    // Nurbs configs
    int sample_size_;

    // Optimization configs
    int population_size_;
    int max_generations_;
    float mutation_distribution_index_;
    float crossover_probability_;
    float crossover_distribution_index_;
    float final_cost_time_;
    float final_cost_security_;
    float final_cost_energy_;

    void parseConfig(std::string config_file_path);
    void solveQuadraticSurfaceCoefficients()
    {
        Eigen::MatrixXd A(6, 6);
        Eigen::VectorXd b(6);
    
        // We define the A matrix according to the quadratic surface equation
        // 0 = a*x^2 + b*y^2 + c*z^2 + d*x*y + e*x*z + f*y*z + g*x + h*y + j*z + k
        // The way we take our permanenet regime data, we can eliminate the d, e and f coefficients
        // We then set k = 1, as we are only interested in the coefficients of the quadratic surface and this is a scaling factor
        // So the equation becomes 0 = a*x^2 + b*y^2 + c*z^2 + g*x + h*y + j*z + 1
        // We then have the following system of equations:
    
        // A, B, C, G, H, J are the coefficients of the quadratic surface equation
        // Using points (drone_permanent_power_.roll_, 0, 0), (0, drone_permanent_power_.pitch_, 0), (0, 0, drone_permanent_power_.ascent_)
        // (-drone_permanent_power_.roll_, 0, 0), (0, -drone_permanent_power_.pitch_, 0), (0, 0, -drone_permanent_power_.descent_)
    
        A << 0, pow(drone_permanent_power_.pitch_, 2.0), 0, 0, drone_permanent_power_.pitch_, 0,
             pow(drone_permanent_power_.roll_, 2.0), 0, 0, drone_permanent_power_.roll_, 0, 0,
             0, 0, pow(drone_permanent_power_.ascent_, 2.0), 0, 0, drone_permanent_power_.ascent_,
             0, pow(drone_permanent_power_.pitch_, 2.0), 0, 0, -drone_permanent_power_.pitch_, 0,
             pow(drone_permanent_power_.roll_, 2.0), 0, 0, -drone_permanent_power_.roll_, 0, 0,
             0, 0, pow(drone_permanent_power_.descent_, 2.0), 0, 0, -drone_permanent_power_.descent_;
    
        b << -1, -1, -1, -1, -1, -1;
    
        drone_permanent_power_.quadratic_surface_coefficients_ = A.colPivHouseholderQr().solve(b);
    }    
};


struct EvalEnergyOutput
{
    double total_smoothness_;
    double max_smoothness_;
    double max_acceleration_;

    double total_energy_;
    double max_energy_;

    EvalEnergyOutput() : total_smoothness_(0.0), max_smoothness_(0.0), max_acceleration_(0.0),
                         total_energy_(0.0), max_energy_(0.0) {}
}; // struct EvalEnergyOutput

struct EvalSafetyOutput
{
    // Collision cost
    double total_collision_cost_;
    double max_collision_cost_;
    double max_occupancy_;
    double nb_of_collision_checks_;

    // Insertion cost
    bool insertion_cost_computed_;
    double total_insertion_cost_;
    double max_insertion_cost_;
    double nb_of_insertion_checks_;

    EvalSafetyOutput() : total_collision_cost_(0.0), max_collision_cost_(0.0), max_occupancy_(0.0), nb_of_collision_checks_(0.0),
                         total_insertion_cost_(0.0), max_insertion_cost_(0.0), nb_of_insertion_checks_(0.0), insertion_cost_computed_(false) {}
}; // struct EvalSafetyOutput

struct EvalNurbsOutput
{
    double time_output_;
    double distance_output_;
    EvalEnergyOutput energy_output_;
    EvalSafetyOutput safety_output_;

    EvalNurbsOutput() : time_output_(0.0), distance_output_(0.0) {}
}; // struct EvalNurbsOutput


//EvalCurvatureOutput getEvalCurvature(std::vector<std::vector<std::vector<double>>> ck_matrix);
EvalNurbsOutput getEvalNurbs(Eigen::Vector3d* curve_points, octomap::ColorOcTree* color_octree, 
                             int sample_size, arena_core::OrientedBoundingBoxWrapper* obb = nullptr);
EvalNurbsOutput getEvalNurbs(Eigen::VectorXd* curve_points, octomap::ColorOcTree* color_octree, 
                             int sample_size, const PipelineConfig& pipeline_config, 
                             const std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>& obb);

void evalTimeCost(double distance, double velocity, double& time_output);
void evalInsertionCost(Eigen::Vector3d point1, octomap::ColorOcTree* color_octree, 
                       const std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>& obb,
                       EvalSafetyOutput& output);
void evalCollisionCost(Eigen::Vector3d point1, Eigen::Vector3d point2, octomap::ColorOcTree* color_octree, 
                       const std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>& obb,
                       EvalSafetyOutput& output);
void evalSmoothnessCost(Eigen::Vector3d point_i_m_1, Eigen::Vector3d point_i, Eigen::Vector3d point_i_p_1, 
                         Eigen::Vector3d point_i_p_2, Eigen::Vector3d point_i_p_3,
                         double velocity_i_m_1, double velocity_i, double velocity_i_p_1, double velocity_i_p_2, 
                         EvalEnergyOutput& output);
void evalEnergyCost(Eigen::Vector3d point_i_m_1, Eigen::Vector3d point_i, Eigen::Vector3d point_i_p_1,
                    double velocity_i, double velocity_i_p_1, PipelineConfig pipeline_config, EvalEnergyOutput& output);

}; // namespace arena_demos

