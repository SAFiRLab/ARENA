#pragma once

// Local
#include "arena_core/math/optimization/evaluation/INurbsAnalyzer.h"
#include "arena_core/geometry/oriented_bounding_box_wrapper.h"

// System
#include <memory>
#include <unordered_map>
#include <limits>

// Eternal Libraries
// Eigen
#include <Eigen/Dense>
// Octomap
#include <octomap/ColorOcTree.h>


namespace arena_demos
{

/**
 * @brief Configuration structure for NadileNurbsAnalyzer.
 * This structure holds the configuration parameters for the NadileNurbsAnalyzer.
 */
struct NadileNurbsAnalyzerConfig
{
    arena_core::NurbsAnalyzerConfig base_config; // Base configuration for NURBS analyzers

    double robot_mass_; // Mass of the robot in kg

    // Robot power consumption in steady state (in Watts)
    double robot_permanent_power_roll_;
    double robot_permanent_power_pitch_;
    std::vector<double> robot_permanent_power_quadric_surface_coefficients_;
    double A;
    double B;
    double C;
    double D;
    double E;
    double F;

    NadileNurbsAnalyzerConfig(unsigned int a_sample_size)
        : base_config(a_sample_size), robot_mass_(0.0)
    {}

    void setRobotPermanentPowerQuadricSurfaceCoefficients(
        double a_A, double a_B, double a_C, double a_D, double a_E, double a_F)
    {
        robot_permanent_power_quadric_surface_coefficients_ = {a_A, a_B, a_C, a_D, a_E, a_F};
        A = a_A;
        B = a_B;
        C = a_C;
        D = a_D;
        E = a_E;
        F = a_F;
    }
}; // NadileNurbsAnalyzerConfig


struct NadileEvalNurbsOutput
{
    // Time cost outputs
    double distance_output_; // Total distance covered by the NURBS curve
    double time_output_; // Total time taken for the NURBS curve

    // Safety cost outputs
    double total_collision_cost_; // Total collision cost
    double max_collision_cost_; // Maximum collision cost
    double max_occupancy_; // Maximum occupancy detected in the environment
    double nb_of_collision_checks_; // Number of collision checks
    
    double total_insertion_cost_; // Total insertion cost
    double max_insertion_cost_; // Maximum insertion cost
    double nb_of_insertion_checks_; // Number of insertion checks
    bool insertion_cost_computed_; // Flag to indicate if insertion cost has been computed

    // Energy cost outputs
    double total_energy_; // Total energy consumed by the NURBS curve
    double max_energy_; // Maximum energy consumed by the NURBS curve
    double max_acceleration_; // Maximum acceleration along the NURBS curve
    
    NadileEvalNurbsOutput() :
        distance_output_(0.0), time_output_(0.0),
        total_collision_cost_(0.0), max_collision_cost_(0.0), max_occupancy_(0.0), nb_of_collision_checks_(0.0),
        total_insertion_cost_(0.0), max_insertion_cost_(0.0), nb_of_insertion_checks_(0.0), insertion_cost_computed_(false),
        total_energy_(0.0), max_energy_(0.0), max_acceleration_(0.0)
        {}
}; // NadileEvalNurbsOutput


/**
 * @brief NadileNurbsAnalyzer class for evaluating NURBS curves.
 * This class implements the INurbsAnalyzer interface and provides methods to evaluate NURBS curves
 * with specific configurations and safety checks for the Nadile project.
 */
class NadileNurbsAnalyzer : public arena_core::INurbsAnalyzer
{
public:
    /**
     * @brief Default constructor for NadileNurbsAnalyzer.
     */
    NadileNurbsAnalyzer(const std::shared_ptr<octomap::ColorOcTree> a_color_octree,
                        const NadileNurbsAnalyzerConfig& a_config,
                        const std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>& a_obbs);

    /**
     * @brief Destructor for NadileNurbsAnalyzer.
     */
    ~NadileNurbsAnalyzer() override = default;

    /**
     * @brief Evaluate the NURBS curve.
     * This method implements the evaluation of the NURBS curve and populates the output structure with the results.
     *
     * @param a_output Reference to an EvalNurbsOutput structure where results will be stored.
     * @param a_curve_points Matrix of control points for the NURBS curve.
     */
    void eval(const Eigen::MatrixXd& a_curve_points, arena_core::EvalNurbsOutput& a_output) override;

private:

    /************* User-defined methods *************/
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
     * @brief Evaluate the collision cost based on the NURBS curve and environment.
     * This method computes the collision cost for the NURBS curve by checking against the environment.
     *
     * @param a_point1 The first point of the NURBS curve segment.
     * @param a_point2 The second point of the NURBS curve segment.
     */
    void evalCollisionCost(const Eigen::Vector3d& a_point1, const Eigen::Vector3d& a_point2);

    /**
     * @brief Evaluate the insertion cost based on the NURBS curve and environment.
     * This method computes the insertion cost for the NURBS curve by checking against the environment.
     *
     * @param a_point_i_m_1 The point before the current point in the NURBS curve.
     * @param a_point_i The current point in the NURBS curve.
     * @param a_point_i_p1 The point after the current point in the NURBS curve.
     * @param a_velocity_i The velocity at the current point in the NURBS curve.
     * @param a_velocity_i_p1 The velocity at the next point in the NURBS curve.
     */
    void evalEnergyCost(const Eigen::Vector3d& a_point_i_m_1, const Eigen::Vector3d& a_point_i, 
                        const Eigen::Vector3d& a_point_i_p1, const double a_velocity_i, const double a_velocity_i_p1);

    /**
     * @brief Evaluate the acceleration constraint based on the NURBS curve.
     * This method computes the acceleration constraint for the NURBS curve by checking the velocities at
     * the current and surrounding points.
     *
     * @param a_point_i_m_1 The point before the current point in the NURBS curve.
     * @param a_point_i The current point in the NURBS curve.
     * @param a_point_i_p_1 The point after the current point in the NURBS curve.
     * @param a_point_i_p_2 The second point after the current point in the NURBS curve.
     * @param a_point_i_p_3 The third point after the current point in the NURBS curve.
     * @param a_velocity_i_m_1 The velocity at the point before the current point in the NURBS curve.
     * @param a_velocity_i The velocity at the current point in the NURBS curve.
     * @param a_velocity_i_p_1 The velocity at the first point after the current point in the NURBS curve.
     * @param a_velocity_i_p_2 The velocity at the second point after the current point in the NURBS curve
     */
    void evalAccelerationConstraint(const Eigen::Vector3d& a_point_i_m_1, const Eigen::Vector3d& a_point_i, 
                                    const Eigen::Vector3d& a_point_i_p_1, const Eigen::Vector3d& a_point_i_p_2,
                                    const Eigen::Vector3d& a_point_i_p_3, const double a_velocity_i_m_1,
                                    const double a_velocity_i, const double a_velocity_i_p_1, const double a_velocity_i_p_2);

    /************* User-defined attributes *************/
    std::shared_ptr<octomap::ColorOcTree> color_octree_; // Pointer to the color octree
    NadileNurbsAnalyzerConfig nadile_config_; // Configuration for the analyzer
    std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper> obbs_; // OBBs for various objects
    NadileEvalNurbsOutput nadile_output_; // Output structure for NURBS evaluation results

}; // class NadileNurbsAnalyzer

}; // namespace arena_demos
