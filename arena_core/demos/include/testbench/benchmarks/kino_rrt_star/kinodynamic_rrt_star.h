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
#include "testbench/benchmarks/common/native_trajectory.h"

// System
#include <cstdint>
#include <functional>
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

struct KinodynamicRrtStarConfig
{
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero(); // Positions of X_free
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Ones();
    double max_speed = 0.5;         // X_free: ||v|| <= max_speed (m/s)
    double max_acceleration = 2.2;  // U_free: ||u|| <= max_acceleration (m/s^2)
    double control_penalty = 50.0;  // r of R = r I in the cost int (1 + u^T R u) dt
    double neighbor_radius = 20.0;  // Cost radius of the neighbors (c*[x, x_i] < radius), <= 0: infinite
    double check_step = 0.1;        // s, states and controls checked along the connections every check_step
    double time_budget = 5.0;       // s, anytime: the best trajectory to the goal is returned
    size_t max_nodes = 2000000;
    uint64_t seed = 0;              // 0: random seed
}; // struct KinodynamicRrtStarConfig


struct KinodynamicRrtStarStats
{
    size_t nb_of_iterations = 0;
    size_t nb_of_nodes = 0;
    size_t nb_of_goal_improvements = 0;
    double first_solution_time = -1.0;
    double cost = 0.0;
    double time = 0.0;
}; // struct KinodynamicRrtStarStats


/**
 * @brief Kinodynamic RRT* of Webb and van den Berg ("Kinodynamic RRT*: Asymptotically optimal motion planning for robots
 * with linear differential constraints", ICRA 2013, arXiv 1205.5088) for the 3D double integrator of their experiments
 * (x = (p, v), A = [0 I; 0 0], B = [0; I], c = 0, R = r I).
 *
 * Any pair of states is connected exactly by the optimal fixed-final-state free-final-time trajectory minimizing
 * c[pi] = int_0^tau (1 + u^T R u) dt (Section IV, closed form: A is nilpotent, the optimal arrival time is the global
 * minimum of c[tau] among the positive roots of a quartic, the trajectory is a cubic polynomial per axis). Algorithm of
 * Fig. 3: a state x_i is sampled in X_free (positions valid and in the bounds, ||v|| <= max_speed), its parent is the
 * neighbor (c*[x, x_i] < radius) minimizing cost[x] + c*[x, x_i] with a collision-free connection (states in X_free and
 * controls in U_free along it), then the neighbors and the goal state are rewired through x_i when it lowers their cost.
 * The sampled state itself is added to the tree (no steer). The costs of the descendants of a rewired node are updated.
 * Neighbors: the connections in X_free have ||v|| <= max_speed, so the neighbors with c* < radius (> tau*) are within
 * max_speed * radius of x_i, the candidates are found with this position radius and then filtered on c*.
 */
class KinodynamicRrtStar
{
public:

    using ValidityFn = std::function<bool(const Eigen::Vector3d&)>;

    KinodynamicRrtStar(const KinodynamicRrtStarConfig& a_config, ValidityFn a_validity)
    : config_(a_config), validity_(std::move(a_validity)) {}

    /** @brief Trajectory from a_start (at rest) to a_goal (at rest), false if the goal wasn't reached. */
    bool plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, NativeTrajectory& a_trajectory,
              double a_sampling_step, KinodynamicRrtStarStats* a_stats = nullptr) const;

    /** @brief Optimal connection between two states (Section IV): arrival time tau* and cost c*, false if none. */
    bool optimalConnection(const TrajectoryPoint& a_from, const TrajectoryPoint& a_to, double& a_tau, double& a_cost) const;

    /** @brief State at time a_t of the optimal connection of duration a_tau, and its control. */
    static TrajectoryPoint connectionState(const TrajectoryPoint& a_from, const TrajectoryPoint& a_to, double a_tau, double a_t,
                                           Eigen::Vector3d* a_control = nullptr);

    /** @brief COLLISIONFREE: states in X_free and controls in U_free along the connection. */
    bool isConnectionFree(const TrajectoryPoint& a_from, const TrajectoryPoint& a_to, double a_tau) const;

private:
    bool isStateFree(const TrajectoryPoint& a_state) const;

    KinodynamicRrtStarConfig config_;
    ValidityFn validity_;

}; // class KinodynamicRrtStar

}; // namespace arena_benchmarks
