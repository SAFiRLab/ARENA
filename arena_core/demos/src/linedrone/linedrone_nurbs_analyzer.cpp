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

#include "linedrone/linedrone_nurbs_analyzer.h"

// System
#include <vector>


namespace arena_demos
{

LinedroneNurbsAnalyzer::LinedroneNurbsAnalyzer(octomap::ColorOcTree* a_color_octree,
                                               const LinedroneNurbsAnalyzerConfig& a_config,
                                               const std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>& a_obbs)
    : color_octree_(a_color_octree),
      linedrone_config(a_config),
      obbs_(a_obbs), 
      linedrone_output_(),
      arena_core::INurbsAnalyzer(a_config.base_config)
{
    /*if (!color_octree_)
        throw std::runtime_error("Color octree pointer is null.");

    if (!a_config.base_config.sample_size > 0)
        throw std::runtime_error("Sample size must be greater than zero.");

    // Check if the parameters are valid
    if (linedrone_config.robot_mass_ <= 0.0)
        throw std::runtime_error("Robot mass must be greater than zero.");

    if (linedrone_config.robot_permanent_power_roll_ < 0.0 || linedrone_config.robot_permanent_power_pitch_ < 0.0)
        throw std::runtime_error("Robot permanent power must be non-negative.");

    if (linedrone_config.robot_permanent_power_quadric_surface_coefficients_.empty())
        throw std::runtime_error("Robot permanent power quadric surface coefficients must not be empty.");

    if (linedrone_config.robot_permanent_power_quadric_surface_coefficients_.size() != 6)
        throw std::runtime_error("Robot permanent power quadric surface coefficients must have exactly 6 elements.");

    // Check if at least one robot power coefficient is non-zero
    if (std::all_of(linedrone_config.robot_permanent_power_quadric_surface_coefficients_.begin(),
                   linedrone_config.robot_permanent_power_quadric_surface_coefficients_.end(),
                   [](double coeff) { return coeff == 0.0; }))
        throw std::runtime_error("At least one robot permanent power quadric surface coefficient must be non-zero.");
    
    if (linedrone_config.C <= 0.0)
        throw std::runtime_error("Coefficient C must be greater than zero for discriminant evaluation.");*/
}

void LinedroneNurbsAnalyzer::evalTimeCost(const double a_distance, const double a_velocity)
{
    if (a_velocity <= 1.0e-6)
    {
        linedrone_output_.time_output_ += a_distance / (a_velocity + 1.0e-6); // Prevent division by zero or very small velocity
    }
    else
        linedrone_output_.time_output_ += a_distance / a_velocity; // Normal case
}

void LinedroneNurbsAnalyzer::evalCollisionCost(const Eigen::Vector3d& a_point1, const Eigen::Vector3d& a_point2)
{
    double tree_depth = color_octree_->getTreeDepth();
    octomap::point3d origin = octomap::point3d(a_point1.x(), a_point1.y(), a_point1.z());
    
    octomap::ColorOcTreeNode* node = color_octree_->search(origin, tree_depth);

    if (node)
    {
        octomap::ColorOcTreeNode::Color node_color = node->getColor();
        double blue = double(uint8_t(node_color.b));
        double node_collision_cost = 1.0 - (blue / 255.0); // Assuming blue channel indicates collision risk
        linedrone_output_.total_collision_cost_ += node_collision_cost;
        if (node_collision_cost > linedrone_output_.max_collision_cost_)
            linedrone_output_.max_collision_cost_ = node_collision_cost;

        double occupancy = node->getOccupancy();
        if (occupancy > linedrone_output_.max_occupancy_)
            linedrone_output_.max_occupancy_ = occupancy;

            linedrone_output_.nb_of_collision_checks_++;
    }
    else
    {
        // If the node is not found, we assume no collision cost but still increment the check count
        linedrone_output_.nb_of_collision_checks_++;
    }

    /*octomap::point3d end_point = octomap::point3d(a_point2.x(), a_point2.y(), a_point2.z());
    octomap::point3d_collection ray_traced_points;
    if (color_octree_->computeRay(origin, end_point, ray_traced_points))
    {
        for (const auto& point : ray_traced_points)
        {
            octomap::ColorOcTreeNode* ray_node = color_octree_->search(point, tree_depth);

            if (ray_node)
            {
                double occupancy = ray_node->getOccupancy();
                if (occupancy > linedrone_output_.max_occupancy_)
                    linedrone_output_.max_occupancy_ = occupancy;
            }
        }
    }*/
}

void LinedroneNurbsAnalyzer::evalEnergyCost(const Eigen::Vector3d& a_point_i_m_1, const Eigen::Vector3d& a_point_i, 
                                         const Eigen::Vector3d& a_point_i_p1, const double a_velocity_i, const double a_velocity_i_p1)
{
    Eigen::Vector3d displacement_i_m_1 = a_point_i - a_point_i_m_1;
    Eigen::Vector3d displacement_i = a_point_i_p1 - a_point_i;

    Eigen::Vector3d velocity_i_m_1_vector = Eigen::Vector3d(displacement_i_m_1.normalized() * a_velocity_i);
    Eigen::Vector3d velocity_i_vector = Eigen::Vector3d(displacement_i.normalized() * a_velocity_i_p1);

    // ------------- Transient regime ------------- //
    const double mass_half = 0.5 * linedrone_config.robot_mass_;
    const Eigen::Vector3d velocity_diff = velocity_i_m_1_vector.cwiseAbs2() - velocity_i_vector.cwiseAbs2();
    double transient_energy = (mass_half * velocity_diff).norm();
    // ----------------------------------------- //

    // ------------- Steady regime ------------- //
    const double velocity_i_p_1_safe = (std::abs(a_velocity_i_p1) > 1e-6) ? a_velocity_i_p1 : 1e-6;
    double roll_power = (velocity_i_vector.x() / velocity_i_p_1_safe) * linedrone_config.robot_permanent_power_roll_;
    double pitch_power = (velocity_i_vector.y() / velocity_i_p_1_safe) * linedrone_config.robot_permanent_power_pitch_;

    double linedrone_coeff_F_squared = linedrone_config.F * linedrone_config.F;
    double sqrt_discriminant = sqrt(linedrone_coeff_F_squared - 4.0 * linedrone_config.C * 
                               (1 + linedrone_config.A * (roll_power * roll_power) + linedrone_config.B * (pitch_power * pitch_power)));
    double denominator = 2.0 * linedrone_config.C;                               
    double z1_power = (-linedrone_config.F + sqrt_discriminant) / denominator;
    double z2_power = (-linedrone_config.F - sqrt_discriminant) / denominator;

    double z_power = 0.0;
    if ((a_point_i_p1 - a_point_i)[2] < 0.0)
    {
        z_power = std::min(fabs(z1_power), fabs(z2_power)); // Choose the minimum power for downward motion
    }
    else
        z_power = std::max(fabs(z1_power), fabs(z2_power)); // Choose the maximum power for upward motion

    double total_steady_power = Eigen::Vector3d(roll_power, pitch_power, z_power).norm();

    double velocity_i_safe = (std::abs(a_velocity_i) > 1e-6) ? a_velocity_i : velocity_i_p_1_safe; // Prevent division by zero or very small velocity
    double steady_energy = total_steady_power * displacement_i.norm() / velocity_i_safe; // Normal case
    // ----------------------------------------- //

    double total_energy = transient_energy + steady_energy;

    if (total_energy > linedrone_output_.max_energy_)
        linedrone_output_.max_energy_ = total_energy;

    linedrone_output_.total_energy_ += total_energy;
}

void LinedroneNurbsAnalyzer::evalAccelerationConstraint(const Eigen::Vector3d& a_point_i_m_1, const Eigen::Vector3d& a_point_i, 
                                                     const Eigen::Vector3d& a_point_i_p_1, const Eigen::Vector3d& a_point_i_p_2,
                                                     const Eigen::Vector3d& a_point_i_p_3, const double a_velocity_i_m_1,
                                                     const double a_velocity_i, const double a_velocity_i_p_1, const double a_velocity_i_p_2)
{
    double velocity_DUA_i = (a_velocity_i + a_velocity_i_p_1) / 2.0; // Average velocity between current and next point
    double velocity_DUA_i_p_1 = (a_velocity_i_p_1 + a_velocity_i_p_2) / 2.0; // Average velocity between next and next-next point

    Eigen::Vector3d displacement_i_m_1 = a_point_i - a_point_i_m_1;
    Eigen::Vector3d displacement_i = a_point_i_p_1 - a_point_i;
    Eigen::Vector3d displacement_i_p_1 = a_point_i_p_2 - a_point_i_p_1;
    Eigen::Vector3d displacement_i_p_2 = a_point_i_p_3 - a_point_i_p_2;

    double delta_t0 = displacement_i.norm() / velocity_DUA_i;
    double delta_t1 = displacement_i_p_1.norm() / velocity_DUA_i_p_1;

    // Compute the velocity vectors from the displacements and velocities
    Eigen::Vector3d velocity_i_m_1_vector = displacement_i_m_1.normalized() * a_velocity_i_m_1;
    Eigen::Vector3d velocity_i_vector = displacement_i.normalized() * a_velocity_i;
    Eigen::Vector3d velocity_i_p_1_vector = displacement_i_p_1.normalized() * a_velocity_i_p_1;
    Eigen::Vector3d velocity_i_p_2_vector = displacement_i_p_2.normalized() * a_velocity_i_p_2;

    // Get mean velocity vectors around points
    Eigen::Vector3d mean_velocity_i = (velocity_i_m_1_vector + velocity_i_vector + velocity_i_p_1_vector) / 3.0;
    Eigen::Vector3d mean_velocity_i_p_1 = (velocity_i_vector + velocity_i_p_1_vector + velocity_i_p_2_vector) / 3.0;

    // Compute the acceleration vectors
    Eigen::Vector3d acceleration_i = (mean_velocity_i_p_1 - mean_velocity_i) / delta_t0;

    double acceleration_magnitude = acceleration_i.norm();
    if (acceleration_magnitude > linedrone_output_.max_acceleration_)
        linedrone_output_.max_acceleration_ = acceleration_magnitude;
}

void LinedroneNurbsAnalyzer::eval(const Eigen::MatrixXd& a_curve_points, arena_core::EvalNurbsOutput& a_output)
{
    if (a_output.fitness_array_.empty())
        throw std::runtime_error("Fitness array is empty. Please initialize it before evaluation.");
    
    if (a_output.fitness_array_.size() != a_output.fitness_size_)
        throw std::runtime_error("Fitness array size does not match the expected fitness output size.");

    if (a_output.constraint_size_ > 0 && a_output.constraint_array_.empty())
        throw std::runtime_error("Constraint array is empty but constraint size is greater than zero. Please initialize it before evaluation.");

    if (a_output.constraint_size_ > 0 && a_output.constraint_array_.size() != a_output.constraint_size_)
        throw std::runtime_error("Constraint array size does not match the expected constraint output size.");
    
    for (unsigned int i = 0; i < linedrone_config.base_config.sample_size - 3; i++)
    {
        Eigen::Map<const Eigen::Vector3d> point1(a_curve_points.col(i).data());
        Eigen::Map<const Eigen::Vector3d> point2(a_curve_points.col(i + 1).data());
        Eigen::Map<const Eigen::Vector3d> point3(a_curve_points.col(i + 2).data());

        // Evaluate time cost between point1 and point2
        double velocity_i = a_curve_points(3, i); // Assuming the 4th column is velocity
        double velocity_i_p1 = a_curve_points(3, i + 1);
        double distance = (point2 - point1).norm();
        double velocity_DUA_i = (velocity_i + velocity_i_p1) / 2.0; // Assuming the 4th column is velocity
        evalTimeCost(distance, velocity_DUA_i);

        // Evaluate collision cost between point1 and point2
        evalCollisionCost(point1, point2);

        // Evaluate insertion cost (if needed, currently not implemented)
        //if (!obbs_.empty())
            //evalInsertionCost(point1, point2); // Placeholder for insertion cost evaluation

        // Evaluate acceleration constraint and energy cost
        if (i > 0)
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
            evalEnergyCost(point1, point1, point2, velocity_i, velocity_i_p1);
    }

    // Handle the last segments
    for (unsigned int i = linedrone_config.base_config.sample_size - 3; i < linedrone_config.base_config.sample_size - 1; i++)
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
        evalCollisionCost(point1, point2);

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
    }

    Eigen::Map<const Eigen::Vector3d> last_point(a_curve_points.col(linedrone_config.base_config.sample_size - 1).data());

    double tree_depth = color_octree_->getTreeDepth();
    octomap::ColorOcTreeNode* node = color_octree_->search(octomap::point3d(last_point.x(), last_point.y(), last_point.z()), tree_depth);

    if (node)
    {
        octomap::ColorOcTreeNode::Color node_color = node->getColor();
        double blue = double(uint8_t(node_color.b));
        double node_collision_cost = 1.0 - double(blue / 255.0); // Assuming blue channel indicates collision risk
        linedrone_output_.total_collision_cost_ += node_collision_cost;
        if (node_collision_cost > linedrone_output_.max_collision_cost_)
            linedrone_output_.max_collision_cost_ = node_collision_cost;

        linedrone_output_.nb_of_collision_checks_++;
    }
    else
        linedrone_output_.nb_of_collision_checks_++;

    // if (!obbs_.empty())
    //    evalInsertionCost(last_point); // Placeholder for insertion cost evaluation

    // Set the output values
    a_output.fitness_array_[0] = linedrone_output_.time_output_;

    if (linedrone_output_.total_insertion_cost_ > 0.0)
    {
        a_output.fitness_array_[1] = ((linedrone_output_.total_collision_cost_ / linedrone_output_.nb_of_collision_checks_) + linedrone_output_.max_collision_cost_) + 
        ((linedrone_output_.total_insertion_cost_ / linedrone_output_.nb_of_insertion_checks_) + linedrone_output_.max_insertion_cost_);
    }
    else
        a_output.fitness_array_[1] = (linedrone_output_.total_collision_cost_ / linedrone_output_.nb_of_collision_checks_) + linedrone_output_.max_collision_cost_;

    a_output.fitness_array_[2] = linedrone_output_.total_energy_;

    a_output.constraint_array_[0] = linedrone_output_.max_acceleration_;
    a_output.constraint_array_[1] = linedrone_output_.max_occupancy_;

    linedrone_output_ = LinedroneEvalNurbsOutput();
}



}; // arena_demos
