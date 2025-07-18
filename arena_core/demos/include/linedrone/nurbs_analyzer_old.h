#pragma once

// Local
#include "linedrone_navigation/pipeline_config.h"

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


namespace linedrone_navigation
{

struct OrientedBoundingBoxWrapper;

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
                             int sample_size, OrientedBoundingBoxWrapper* obb = nullptr);
EvalNurbsOutput getEvalNurbs(Eigen::VectorXd* curve_points, octomap::ColorOcTree* color_octree, 
                             int sample_size, const PipelineConfig& pipeline_config, 
                             std::unordered_map<std::string, OrientedBoundingBoxWrapper>& obb);

void evalTimeCost(double distance, double velocity, double& time_output);
void evalInsertionCost(Eigen::Vector3d point1, octomap::ColorOcTree* color_octree, 
                       std::unordered_map<std::string, OrientedBoundingBoxWrapper>& obb,
                       EvalSafetyOutput& output);
void evalCollisionCost(Eigen::Vector3d point1, Eigen::Vector3d point2, octomap::ColorOcTree* color_octree, 
                       std::unordered_map<std::string, OrientedBoundingBoxWrapper>& obb,
                       EvalSafetyOutput& output);
void evalSmoothnessCost(Eigen::Vector3d point_i_m_1, Eigen::Vector3d point_i, Eigen::Vector3d point_i_p_1, 
                         Eigen::Vector3d point_i_p_2, Eigen::Vector3d point_i_p_3,
                         double velocity_i_m_1, double velocity_i, double velocity_i_p_1, double velocity_i_p_2, 
                         EvalEnergyOutput& output);
void evalEnergyCost(Eigen::Vector3d point_i_m_1, Eigen::Vector3d point_i, Eigen::Vector3d point_i_p_1,
                    double velocity_i, double velocity_i_p_1, PipelineConfig pipeline_config, EvalEnergyOutput& output);

}; // namespace linedrone_navigation

