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

// System
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

/**
 * @brief State of a trajectory at a given time.
 */
struct TrajectoryPoint
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double time = 0.0; // s, from the start of the trajectory
};


/**
 * @brief Trajectory of a benchmark planner in its own representation (no conversion to the NURBS curves of ARENA).
 *
 * The robot moves on a straight line between two consecutive points, from the position and velocity of the first one to
 * the ones of the second one during the time between them. Two consecutive points at the same position and time with
 * different velocities are an instantaneous change of velocity (planners without dynamics). The dynamic planners give
 * points sampled on their trajectories (e.g. at every propagation step), with their exact positions and velocities.
 */
using NativeTrajectory = std::vector<TrajectoryPoint>;


/**
 * @brief Trajectory of a path flown at a speed per segment, the speed changing instantaneously at the waypoints, from
 * rest at the start and to rest at the goal (planners without dynamics, e.g. MOAR and its own speeds).
 *
 * @param a_waypoints Points of the path, from the start to the goal (consecutive identical points are skipped).
 * @param a_segment_speeds Speed on the segment from a_waypoints[i] to a_waypoints[i + 1] (> 0).
 */
NativeTrajectory segmentSpeedTrajectory(const std::vector<Eigen::Vector3d>& a_waypoints, const std::vector<double>& a_segment_speeds);

/**
 * @brief Trajectory of a path flown at a constant speed, reached instantaneously at the start and left instantaneously at
 * the goal (geometric planners).
 */
NativeTrajectory constantSpeedTrajectory(const std::vector<Eigen::Vector3d>& a_waypoints, double a_speed);

/** @brief Same as constantSpeedTrajectory() for a 3 x K matrix of waypoints. */
NativeTrajectory constantSpeedTrajectory(const Eigen::MatrixXd& a_waypoints, double a_speed);

/**
 * @brief Appends a braking at a constant deceleration a_deceleration, opposite to the final velocity, until the robot
 * stops (dynamic planners reaching their goal region with a residual velocity). Nothing is appended at rest.
 */
void appendBraking(NativeTrajectory& a_trajectory, double a_deceleration);

/**
 * @brief Appends the points of a motion at constant acceleration from a_start during a_duration, every a_time_step
 * (and at the end of the motion), the first point (a_start) isn't appended.
 */
void appendConstantAcceleration(NativeTrajectory& a_trajectory, const TrajectoryPoint& a_start, const Eigen::Vector3d& a_acceleration,
                                double a_duration, double a_time_step);

/** @brief Length of the path of a trajectory. */
double trajectoryLength(const NativeTrajectory& a_trajectory);

}; // namespace arena_benchmarks
