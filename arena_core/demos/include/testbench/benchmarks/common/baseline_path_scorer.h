/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, David-Alexandre Poissant, Université de Sherbrooke
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
#include "arena_core/math/nurbs.h"
#include "arena_core/math/control_point.h"
#include "linedrone/linedrone_nurbs_analyzer.h"
#include "linedrone/costmap_mapping.h"

// System
#include <limits>
#include <memory>
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

/**
 * @brief Trajectory of a benchmark planner evaluated like the trajectories of ARENA.
 */
struct ScoredPath
{
    Eigen::MatrixXd control_points; // 4 x K control points (x, y, z, speed) of the NURBS curve
    Eigen::MatrixXd samples;        // 4 x sample_size samples (x, y, z, speed) of the NURBS curve
    std::vector<double> fitness = std::vector<double>(3, std::numeric_limits<double>::max()); // Time, safety and energy costs
    double max_acceleration = std::numeric_limits<double>::max();
    double max_occupancy = std::numeric_limits<double>::max();
    bool feasible = false; // Respects the constraints of the ARENA optimizer (acceleration, occupancy at the samples)
    bool safe = false;     // No segment between two samples crosses an occupied voxel (same check as ARENA)
};


/**
 * @brief Evaluates the paths of the benchmark planners with the cost functions and checks of ARENA.
 *
 * A path (waypoints) becomes the control points of a NURBS curve with a speed profile, the curve is sampled like the
 * trajectories of ARENA and evaluated by the LinedroneNurbsAnalyzer (time, safety and energy costs, acceleration and
 * occupancy constraints). The safety check is the ray tracing between consecutive samples of LinedroneTestNode.
 * Every planner is then compared with the same objectives, constraints and safety check.
 */
class BaselinePathScorer
{
public:

    /**
     * @param a_costmap_mapping Mapping with the inflated octomap (collisions) and the SDF octomap (safety cost).
     * @param a_config Robot and sampling configuration of the analyzer (same as LinedroneTestNode).
     * @param a_degree Degree of the NURBS curve.
     */
    BaselinePathScorer(std::shared_ptr<arena_demos::CostmapMapping> a_costmap_mapping,
                       const arena_demos::LinedroneNurbsAnalyzerConfig& a_config, int a_degree = 5);

    /************* User-defined methods *************/
    /**
     * @brief Points of a polyline at regular distances along it, the first and last points are kept.
     * @param a_waypoints 3 x K (or more rows) points of the polyline.
     * @param a_spacing Distance between two consecutive points along the polyline, <= 0 returns the polyline.
     */
    static Eigen::MatrixXd resampleByDistance(const Eigen::MatrixXd& a_waypoints, double a_spacing);

    /**
     * @brief Inserts points evenly on the segments of a polyline until it has a_nb_of_points points.
     * Same method as LinedroneTestNode::addControlPointsToPath, used to have enough control points for the degree.
     */
    static Eigen::MatrixXd densify(const Eigen::MatrixXd& a_points, int a_nb_of_points);

    /**
     * @brief Control points (x, y, z, speed) of a path, with at least degree + 1 points.
     *
     * The speed is null at the first and last points and a_speed elsewhere, like the RRT paths that initialize ARENA.
     * With a_ramp_length > 0, the speed grows linearly over this distance from the start and decreases over this
     * distance before the goal.
     *
     * @param a_waypoints 3 x K points of the path, from the start to the goal.
     */
    Eigen::MatrixXd toControlPoints(const Eigen::MatrixXd& a_waypoints, double a_speed, double a_ramp_length = 0.0) const;

    /**
     * @brief Evaluates the NURBS curve of the control points.
     * @param a_control_points 4 x K control points (x, y, z, speed), K >= degree + 1.
     */
    ScoredPath score(const Eigen::MatrixXd& a_control_points);

    /**
     * @brief Evaluates the NURBS curve of the control points without the safety check (ray tracing).
     * Used in the fitness function of the optimizers, the safety check is done on their final population.
     */
    ScoredPath evaluate(const Eigen::MatrixXd& a_control_points);

    /**
     * @brief True if no segment between two consecutive samples crosses an occupied voxel of the inflated octomap.
     */
    bool isPathSafe(const Eigen::MatrixXd& a_samples) const;

    /**
     * @brief Number of segments between two consecutive samples crossing an occupied voxel of the inflated octomap.
     */
    int countUnsafeSegments(const Eigen::MatrixXd& a_samples) const;

    /**
     * @brief Steady-state power (W) of the robot moving in the direction a_direction, from the energy model of ARENA.
     */
    double steadyStatePower(const Eigen::Vector3d& a_direction) const;

    /************* Getters *************/
    int getDegree() const { return degree_; }
    unsigned int getSampleSize() const { return config_.base_config.sample_size; }
    const arena_demos::LinedroneNurbsAnalyzerConfig& getConfig() const { return config_; }

private:

    std::shared_ptr<arena_demos::CostmapMapping> costmap_mapping_;
    arena_demos::LinedroneNurbsAnalyzerConfig config_;
    std::shared_ptr<arena_demos::LinedroneNurbsAnalyzer> analyzer_;
    arena_core::EvalNurbsOutput output_;
    int degree_;

}; // class BaselinePathScorer

}; // namespace arena_benchmarks
