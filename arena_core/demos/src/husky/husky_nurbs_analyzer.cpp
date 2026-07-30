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

#include "husky/husky_nurbs_analyzer.h"

// System
#include <vector>
#include <iostream>


namespace arena_demos
{

HuskyNurbsAnalyzer::HuskyNurbsAnalyzer(std::shared_ptr<arena_demos::TraversabilityCostmap> a_traversability_costmap,
                                       const HuskyNurbsAnalyzerConfig& a_config)
    : traversability_mapping_(a_traversability_costmap),
      husky_config_(a_config),
      husky_output_(),
      arena_core::INurbsAnalyzer(a_config.base_config)
{}

void HuskyNurbsAnalyzer::evalTimeCost(const double a_distance, const double a_velocity)
{
    if (a_velocity <= 1.0e-6)
    {
        husky_output_.time_output_ += a_distance / (a_velocity + 1.0e-6); // Prevent division by zero or very small velocity
    }
    else
        husky_output_.time_output_ += a_distance / a_velocity; // Normal case
}

void HuskyNurbsAnalyzer::evalSafetyCost(const Eigen::Vector2d& a_point1)
{
    auto capability = traversability_mapping_->getCapability<arena_demos::TraversabilityCostCapability>();
    if (capability)
    {
        double node_collision_cost = capability->getTraversabilityCost(a_point1);
        husky_output_.total_collision_cost_ += node_collision_cost;
        if (node_collision_cost > husky_output_.max_collision_cost_)
            husky_output_.max_collision_cost_ = node_collision_cost;
        
        double occupancy = capability->getOccupancy(a_point1);
        if (occupancy > husky_output_.max_occupancy_)
            husky_output_.max_occupancy_ = occupancy;

        husky_output_.nb_of_collision_checks_++;
    }
    else
        std::cerr << "Traversability mapping cannot provide TraversabilityCostCapability, collision cost evaluation is skipped." << std::endl;
}

void HuskyNurbsAnalyzer::eval(const Eigen::MatrixXd& a_curve_points, arena_core::EvalNurbsOutput& a_output)
{
    if (a_output.fitness_array_.empty())
        throw std::runtime_error("Fitness array is empty. Please initialize it before evaluation.");
    
    if (a_output.fitness_array_.size() != a_output.fitness_size_)
        throw std::runtime_error("Fitness array size does not match the expected fitness output size.");

    if (a_output.constraint_size_ > 0 && a_output.constraint_array_.empty())
        throw std::runtime_error("Constraint array is empty but constraint size is greater than zero. Please initialize it before evaluation.");

    if (a_output.constraint_size_ > 0 && a_output.constraint_array_.size() != a_output.constraint_size_)
        throw std::runtime_error("Constraint array size does not match the expected constraint output size.");

    for (unsigned int i = 0; i < husky_config_.base_config.sample_size - 1; i++)
    {
        Eigen::Map<const Eigen::Vector2d> point1(a_curve_points.col(i).data());
        Eigen::Map<const Eigen::Vector2d> point2(a_curve_points.col(i + 1).data());

        // Evaluate time cost between point1 and point2
        double velocity_i = a_curve_points(2, i); // Assuming the 3th column is velocity
        double velocity_i_p1 = a_curve_points(2, i + 1);
        double distance = (point2 - point1).norm();
        double velocity_DUA_i = (velocity_i + velocity_i_p1) / 2.0; // Assuming the 3th column is velocity
        evalTimeCost(distance, velocity_DUA_i);

        // Evaluate safety cost between point1 and point2
        evalSafetyCost(point1);

        // Evaluate acceleration constraint and energy cost
        /*if (i > 0)
        {
            Eigen::Map<const Eigen::Vector3d> point0(a_curve_points.col(i - 1).data());
            Eigen::Map<const Eigen::Vector3d> point4(a_curve_points.col(i + 3).data());

            double velocity_i_m_1 = a_curve_points(3, i - 1);
            double velocity_i_p2 = a_curve_points(3, i + 2);

            evalAccelerationConstraint(point0, point1, point2, point3, point4,
                                       velocity_i_m_1, velocity_i, velocity_i_p1, velocity_i_p2);
            evalEnergyCost(point0, point1, point2, velocity_i, velocity_i_p1);
        }
        else
            evalEnergyCost(point1, point1, point2, velocity_i, velocity_i_p1);*/
    }

    // Handle the last segments
    /*for (unsigned int i = linedrone_config.base_config.sample_size - 3; i < linedrone_config.base_config.sample_size - 1; i++)
    {
        Eigen::Map<const Eigen::Vector3d> point1(a_curve_points.col(i).data());
        Eigen::Map<const Eigen::Vector3d> point2(a_curve_points.col(i + 1).data());

        // Evaluate time cost for the last segment
        double velocity_i = a_curve_points(3, i);
        double velocity_i_p1 = a_curve_points(3, i + 1);
        double distance = (point2 - point1).norm();
        double velocity_DUA_i = (velocity_i + velocity_i_p1) / 2.0; // Assuming the 4th column is velocity
        evalTimeCost(distance, velocity_DUA_i);

        // Evaluate collision cost for the last segment
        evalCollisionCost(point1);

        // Evaluate insertion cost (if needed, currently not implemented)
        //if (!obbs_.empty())
            //evalInsertionCost(point1, point2); // Placeholder for insertion cost evaluation

        // Evaluate energy cost
        if (i < linedrone_config.base_config.sample_size - 2)
        {
            if (i > 0)
            {
                Eigen::Map<const Eigen::Vector3d> point0(a_curve_points.col(i - 1).data());
                evalEnergyCost(point0, point1, point2, velocity_i, velocity_i_p1);
            }
            else
                evalEnergyCost(point1, point1, point2, velocity_i, velocity_i_p1);
            
        }
        else
        {
            if (i > 0)
            {
                Eigen::Map<const Eigen::Vector3d> point0(a_curve_points.col(i - 1).data());
                evalEnergyCost(point0, point1, point2, velocity_i, velocity_i_p1);
            }
        }
    }*/

    Eigen::Map<const Eigen::Vector2d> last_point(a_curve_points.col(husky_config_.base_config.sample_size - 1).data());

    evalSafetyCost(last_point);

    // if (!obbs_.empty())
    //    evalInsertionCost(last_point); // Placeholder for insertion cost evaluation

    // Set the output values
    a_output.fitness_array_[0] = husky_output_.time_output_;
    if (husky_output_.nb_of_collision_checks_ > 0)
    {
        a_output.fitness_array_[1] = (husky_output_.total_collision_cost_ / husky_output_.nb_of_collision_checks_) + husky_output_.max_collision_cost_;
    }
    else
        a_output.fitness_array_[1] = std::numeric_limits<double>::max(); // If no collision checks were performed, set safety cost to max to penalize the solution

    //a_output.fitness_array_[2] = linedrone_output_.total_energy_;

    //a_output.constraint_array_[0] = linedrone_output_.max_acceleration_;
    a_output.constraint_array_[0] = husky_output_.max_occupancy_;

    husky_output_ = HuskyEvalNurbsOutput();
}



}; // arena_demos
