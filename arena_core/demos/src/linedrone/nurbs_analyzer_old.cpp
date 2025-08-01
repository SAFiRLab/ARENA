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

#include "linedrone/nurbs_analyzer_old.h"

// LineDrone_navigation
#include "arena_core/math/utils/linalg.h"
#include "arena_core/geometry/oriented_bounding_box_wrapper.h"

// System
#include <limits>
#include <math.h>


namespace arena_demos
{

/*EvalEnergyOutput getEvalCurvature(std::vector<std::vector<std::vector<double>>> ck_matrix)
{
    // Compute as the sum of all the second derivatives of the curve
    double total_curvature = 0.0;
    std::vector<double> curvatures;

    // Add the 2 first curvatures
    curvatures.push_back(0.0);
    curvatures.push_back(0.0);

    double max_curvature = 0.0;
    for (size_t i = 2; i < ck_matrix.size() - 2; i++)
    {
        // https://openstax.org/books/calculus-volume-3/pages/3-3-arc-length-and-curvature#fs-id1169738214420
        double denominator = pow(linalg::norm(ck_matrix[i][1]), 3);
        double numerator = linalg::norm(linalg::cross(ck_matrix[i][1], ck_matrix[i][2]));

        double temp_curvature = 0;
        // Verify division by 0 and too large values
        if (denominator == 0.0 || numerator / denominator > 1.0e6)
        {
            temp_curvature = 1.0e6;
        }
        else
        {
            temp_curvature = numerator / denominator;
        }

        if (temp_curvature >= max_curvature)
            max_curvature = temp_curvature;

        curvatures.push_back(temp_curvature);
        total_curvature += temp_curvature;
    }

    // Add the 2 last curvatures
    curvatures.push_back(0.0);
    curvatures.push_back(0.0);

    EvalEnergyOutput curvature_out;
    curvature_out.total_curvature_ = total_curvature;
    curvature_out.max_curvature_ = max_curvature;
    curvature_out.curvatures_ = curvatures;
    return curvature_out;
}*/

void evalTimeCost(double distance, double velocity, double& time_output)
{
    if (velocity <= 0.0 + 1.0e-6)
        velocity += 1.0e6;
    
    time_output += distance / velocity;
}

void evalInsertionCost(Eigen::Vector3d point1, octomap::ColorOcTree* color_octree, const std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>& obb,
                       EvalSafetyOutput& output)
{
    if (!obb.empty())
    {
        if (!output.insertion_cost_computed_)
                output.insertion_cost_computed_ = true;
            
        double insertion_cost = 0.0;
        for (auto it: obb)
        {
            Eigen::Vector3d closest_point_on_obb;
            double distance = it.second.distanceFromOBB(point1, closest_point_on_obb);
            double temp_insertion_cost = 0.0;
            if (0.0 < distance && distance < 2.0)
            {
                temp_insertion_cost = 1.0 - distance / 2.0;
            }
            else if (distance == 0.0)
                temp_insertion_cost = 1.0;

            insertion_cost += temp_insertion_cost;
        }

        if (insertion_cost > output.max_insertion_cost_)
            output.max_insertion_cost_ = insertion_cost;
        
        output.total_insertion_cost_ += insertion_cost;
        output.nb_of_insertion_checks_ += 1.0;
    }
}

void evalCollisionCost(Eigen::Vector3d point1, Eigen::Vector3d point2, octomap::ColorOcTree* color_octree, const std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>& obb,
                       EvalSafetyOutput& output)
{
    double tree_depth = color_octree->getTreeDepth();
    octomap::ColorOcTreeNode* octNode = color_octree->search(point1.x(), 
                                                             point1.y(),
                                                             point1.z(),
                                                             tree_depth);
    if (octNode != nullptr)
    {
        octomap::ColorOcTreeNode::Color nodeColor = octNode->getColor();
        double blue = double(uint8_t(nodeColor.b));
        double node_collision_cost = 1 - double(blue / 255.0);
        output.total_collision_cost_ += node_collision_cost;
        if (node_collision_cost > output.max_collision_cost_)
            output.max_collision_cost_ = node_collision_cost;

        output.nb_of_collision_checks_ += 1.0;
    } // 0 if out of octree -> probably empty
    else
        output.nb_of_collision_checks_ += 1.0;

    octomap::point3d origin = octomap::point3d(point1.x(), 
                                               point1.y(),
                                               point1.z());
    octomap::point3d end = octomap::point3d(point2.x(),
                                            point2.y(),
                                            point2.z());
    octomap::point3d_collection collection_ray;
    if (color_octree->computeRay(origin, end, collection_ray))
    {
        for (size_t j = 0; j < collection_ray.size(); j++)
        {
            octomap::ColorOcTreeNode* temp_oct_node = color_octree->search(collection_ray[j], tree_depth);

            if (temp_oct_node != nullptr)
            {
                double occupancy = temp_oct_node->getOccupancy();
                if (occupancy > output.max_occupancy_)
                    output.max_occupancy_ = occupancy;
            }
        }
    }
}

void evalSmoothnessCost(Eigen::Vector3d point_i_m_1, Eigen::Vector3d point_i, Eigen::Vector3d point_i_p_1, 
                         Eigen::Vector3d point_i_p_2, Eigen::Vector3d point_i_p_3,
                         double velocity_i_m_1, double velocity_i, double velocity_i_p_1, double velocity_i_p_2, 
                         EvalEnergyOutput& output)
{
    double velocity_DUA_i = (velocity_i + velocity_i_p_1) / 2;
    double velocity_DUA_i_p_1 = (velocity_i_p_1 + velocity_i_p_2) / 2;

    Eigen::Vector3d displacement_i_m_1 = point_i - point_i_m_1;
    Eigen::Vector3d displacement_i = point_i_p_1 - point_i;
    Eigen::Vector3d displacement_i_p_1 = point_i_p_2 - point_i_p_1;
    Eigen::Vector3d displacement_i_p_2 = point_i_p_3 - point_i_p_2;

    double delta_t0 = displacement_i.norm() / velocity_DUA_i;
    double delta_t1 = displacement_i_p_1.norm() / velocity_DUA_i_p_1;

    // Compute the velocity vectors from the displacements and the velocities
    Eigen::Vector3d velocity_i_m_1_vec = Eigen::Vector3d(displacement_i_m_1.normalized() * velocity_i_m_1);
    Eigen::Vector3d velocity_i_vec = Eigen::Vector3d(displacement_i.normalized() * velocity_i);
    Eigen::Vector3d velocity_i_p_1_vec = Eigen::Vector3d(displacement_i_p_1.normalized() * velocity_i_p_1);
    Eigen::Vector3d velocity_i_p_2_vec = Eigen::Vector3d(displacement_i_p_2.normalized() * velocity_i_p_2);

    // Get mean velocity vectors around points
    Eigen::Vector3d mean_velocity_i = (velocity_i_m_1_vec + velocity_i_vec + velocity_i_p_1_vec) / 3;
    Eigen::Vector3d mean_velocity_i_p_1 = (velocity_i_vec + velocity_i_p_1_vec + velocity_i_p_2_vec) / 3;

    // Compute the acceleration vectors
    Eigen::Vector3d acceleration_i = (mean_velocity_i_p_1 - mean_velocity_i) / delta_t0;
    Eigen::Vector3d acceleration_i_p_1 = (velocity_i_p_2_vec - mean_velocity_i_p_1) / delta_t1;

    if (acceleration_i.norm() > output.max_acceleration_)
        output.max_acceleration_ = acceleration_i.norm();

    // Compute jerk
    Eigen::Vector3d jerk = (acceleration_i_p_1 - acceleration_i) / delta_t0;

    // Compute the smoothness
    double smoothness = pow(acceleration_i.norm(), 2) + pow(jerk.norm(), 2);

    if (smoothness > output.max_smoothness_)
        output.max_smoothness_ = smoothness;

    output.total_smoothness_ += smoothness;
}

void evalEnergyCost(Eigen::Vector3d point_i_m_1, Eigen::Vector3d point_i, Eigen::Vector3d point_i_p_1, 
                    double velocity_i, double velocity_i_p_1, PipelineConfig pipeline_config, EvalEnergyOutput& output)
{
    Eigen::Vector3d displacement_i_m_1 = point_i - point_i_m_1;
    Eigen::Vector3d displacement_i = point_i_p_1 - point_i;

    Eigen::Vector3d velocity_i_m_1_vec = Eigen::Vector3d(displacement_i_m_1.normalized() * velocity_i);
    Eigen::Vector3d velocity_i_vec = Eigen::Vector3d(displacement_i.normalized() * velocity_i_p_1);

    // ------------- Transient regime ------------- //
    double z_transient_energy = pipeline_config.drone_mass_ / 2.0 * (pow(velocity_i_m_1_vec[2], 2.0) - pow(velocity_i_vec[2], 2.0));
    double x_transient_energy = pipeline_config.drone_mass_ / 2.0 * (pow(velocity_i_m_1_vec[0], 2.0) - pow(velocity_i_vec[0], 2.0));
    double y_transient_energy = pipeline_config.drone_mass_ / 2.0 * (pow(velocity_i_m_1_vec[1], 2.0) - pow(velocity_i_vec[1], 2.0));

    double transient_energy = sqrt(pow(z_transient_energy, 2.0) + pow(x_transient_energy, 2.0) + pow(y_transient_energy, 2.0));
    // ----------------------------------------- //
    

    // ------------- Steady state regime ------------- //
    double roll_power = (velocity_i_vec[0] / velocity_i_p_1) * pipeline_config.drone_permanent_power_.roll_;
    double pitch_power = (velocity_i_vec[1] / velocity_i_p_1) * pipeline_config.drone_permanent_power_.pitch_;

    // Z = (-solution[J] +- sqrt(solution[J]**2.0 - 4.0 * solution[C]*(1 + solution[A] * X**2.0 + solution[B] * Y**2.0))) / (2.0 * solution[C])
    double z1_power = (-pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[5] + 
                       sqrt(pow(pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[5], 2.0) - 
                            4.0 * pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[2] * 
                            (1 + pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[0] * pow(roll_power, 2.0) + 
                                 pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[1] * pow(pitch_power, 2.0)))) / 
                       (2.0 * pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[2]);

    double z2_power = (-pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[5] -
                       sqrt(pow(pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[5], 2.0) - 
                            4.0 * pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[2] * 
                            (1 + pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[0] * pow(roll_power, 2.0) + 
                                 pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[1] * pow(pitch_power, 2.0)))) / 
                       (2.0 * pipeline_config.drone_permanent_power_.quadratic_surface_coefficients_[2]);
    
    double z_power = 0.0;
    if ((point_i_p_1 - point_i)[2] < 0.0)
    {
        z_power = std::min(fabs(z1_power), fabs(z2_power));
    }
    else
        z_power = std::max(fabs(z1_power), fabs(z2_power));

    double total_power = sqrt(pow(roll_power, 2.0) + pow(pitch_power, 2.0) + pow(z_power, 2.0));

    double steady_state_energy = 0.0;
    if (fabs(velocity_i) <= 1.0e-6)
    {
        steady_state_energy = total_power * displacement_i.norm() / velocity_i_p_1;
    }
    else
        steady_state_energy = total_power * displacement_i.norm() / velocity_i;
    // ----------------------------------------- //

    double total_energy = (steady_state_energy) + transient_energy;

    /*if (total_energy > 1e3)
        total_energy /= 1e3;*/

    //ROS_INFO("Steady state energy: %f, Transient energy: %f", steady_state_energy, transient_energy);

    if (total_energy > output.max_energy_)
        output.max_energy_ = total_energy;

    output.total_energy_ += total_energy;
}

EvalNurbsOutput getEvalNurbs(Eigen::VectorXd* curve_points, octomap::ColorOcTree* color_octree, 
                             int sample_size, const PipelineConfig& pipeline_config, const std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>& obb)
{
    EvalNurbsOutput eval_nurbs;

    for (int i = 0; i < sample_size - 3; i++)
    {
        Eigen::Vector3d point1 = Eigen::Vector3d(curve_points[i][0], curve_points[i][1], curve_points[i][2]);
        Eigen::Vector3d point2 = Eigen::Vector3d(curve_points[i + 1][0], curve_points[i + 1][1], curve_points[i + 1][2]);
        Eigen::Vector3d point3 = Eigen::Vector3d(curve_points[i + 2][0], curve_points[i + 2][1], curve_points[i + 2][2]);
        
        double distance = (point1 - point2).norm();
        eval_nurbs.distance_output_ += distance;
        double velocity_DUA_i = (curve_points[i][3] + curve_points[i + 1][3]) / 2;
        evalTimeCost(distance, velocity_DUA_i, eval_nurbs.time_output_);

        evalCollisionCost(point1, point2, color_octree, obb, eval_nurbs.safety_output_);

        if (!obb.empty())
            evalInsertionCost(point1, color_octree, obb, eval_nurbs.safety_output_);

        if (i > 0)
        {
            Eigen::Vector3d point0 = Eigen::Vector3d(curve_points[i - 1][0], curve_points[i - 1][1], curve_points[i - 1][2]);
            Eigen::Vector3d point4 = Eigen::Vector3d(curve_points[i + 3][0], curve_points[i + 3][1], curve_points[i + 3][2]);

            evalSmoothnessCost(point0, point1, point2, point3, point4, 
                                curve_points[i - 1][3], curve_points[i][3], curve_points[i + 1][3], curve_points[i + 2][3], 
                                eval_nurbs.energy_output_);
            evalEnergyCost(point0, point1, point2, curve_points[i][3], curve_points[i + 1][3], pipeline_config, eval_nurbs.energy_output_);
        }
        else
            evalEnergyCost(point1, point1, point2, curve_points[i + 1][3], curve_points[i + 1][3], pipeline_config, eval_nurbs.energy_output_);
    }

    for (int i = sample_size - 3; i < sample_size - 1; i++)
    {
        Eigen::Vector3d point1 = Eigen::Vector3d(curve_points[i][0], curve_points[i][1], curve_points[i][2]);
        Eigen::Vector3d point2 = Eigen::Vector3d(curve_points[i + 1][0], curve_points[i + 1][1], curve_points[i + 1][2]);
        
        double distance = (point1 - point2).norm();
        eval_nurbs.distance_output_ += distance;

        double velocity_DUA_i = (curve_points[i][3] + curve_points[i + 1][3]) / 2;
        evalTimeCost(distance, velocity_DUA_i, eval_nurbs.time_output_);

        evalCollisionCost(point1, point2, color_octree, obb, eval_nurbs.safety_output_);

        if (!obb.empty())
            evalInsertionCost(point1, color_octree, obb, eval_nurbs.safety_output_);

        if (i < sample_size - 2)
        {
            if (i > 0)
            {
                Eigen::Vector3d point0 = Eigen::Vector3d(curve_points[i - 1][0], curve_points[i - 1][1], curve_points[i - 1][2]);
                evalEnergyCost(point0, point1, point2, curve_points[i][3], curve_points[i + 1][3], pipeline_config, eval_nurbs.energy_output_);
            }
            else
                evalEnergyCost(point1, point1, point2, curve_points[i + 1][3], curve_points[i + 1][3], pipeline_config, eval_nurbs.energy_output_);
        }
        else
        {
            if (i > 0)
            {
                Eigen::Vector3d point0 = Eigen::Vector3d(curve_points[i - 1][0], curve_points[i - 1][1], curve_points[i - 1][2]);
                evalEnergyCost(point0, point1, point2, curve_points[i][3], curve_points[i][3], pipeline_config, eval_nurbs.energy_output_);
            }
        }
            
    }

    Eigen::Vector3d last_point = Eigen::Vector3d(curve_points[sample_size - 1][0], 
                                                 curve_points[sample_size - 1][1], 
                                                 curve_points[sample_size - 1][2]);

    double tree_depth = color_octree->getTreeDepth();
    octomap::ColorOcTreeNode* end_oct_node = color_octree->search(last_point.x(),
                                                                  last_point.y(),
                                                                  last_point.z(),
                                                                  tree_depth);
    if (end_oct_node != nullptr)
    {
        octomap::ColorOcTreeNode::Color nodeColor = end_oct_node->getColor();
        double blue = double(uint8_t(nodeColor.b));
        double node_collision_cost = 1 - double(blue / 255.0);
        eval_nurbs.safety_output_.total_collision_cost_ += node_collision_cost;
        if (node_collision_cost > eval_nurbs.safety_output_.max_collision_cost_)
            eval_nurbs.safety_output_.max_collision_cost_ = node_collision_cost;

        eval_nurbs.safety_output_.nb_of_collision_checks_ += 1.0;
    } // 0 if out of octree -> probably empty
    else
        eval_nurbs.safety_output_.nb_of_collision_checks_ += 1.0;

    evalInsertionCost(last_point, color_octree, obb, eval_nurbs.safety_output_);


    return eval_nurbs;
}

// These next functions are just for backward compatibility
EvalNurbsOutput getEvalNurbs(Eigen::Vector3d* curve_points, octomap::ColorOcTree* color_octree, 
                             int sample_size, arena_core::OrientedBoundingBoxWrapper* obb)
{
    // Empty
    return EvalNurbsOutput();
}

EvalNurbsOutput getEvalNurbs(Eigen::Vector2d* curve_points, octomap::ColorOcTree* color_octree, 
                             int sample_size, const PipelineConfig& pipeline_config, arena_core::OrientedBoundingBoxWrapper* obb)
{
    // Empty
    return EvalNurbsOutput();
}

}; // namespace linedrone_navigation
