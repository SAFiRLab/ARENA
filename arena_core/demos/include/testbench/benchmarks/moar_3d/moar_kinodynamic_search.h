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
#include "testbench/benchmarks/moar_3d/moar_graph_search.h"

// System
#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

struct MoarKinodynamicConfig
{
    double resolution = 1.0;                                 // Size of a grid cell in meters (MOAR grid)
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();    // Center of the first cell
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Zero();    // The primitives stay inside [min_bounds, max_bounds]
    double velocity_resolution = 0.25;                       // m/s, step of the velocity lattice (every component)
    int max_input_steps = 1;                                 // Inputs: velocity change of -max..max lattice steps per axis
    double tau = 2.0;                                        // s, duration of the motion primitives
    double max_acceleration = 2.2;                           // m/s^2, norm limit (inputs and connection to the goal)
    int check_num = 5;                                       // Checks (validity, speed, safety cost) per primitive
    double goal_radius = 1.5;                                // m, states connected to the goal at rest from this distance
    double check_resolution = 0.2;                           // m, cache of the cell function (validity, SDF cost)
    double time_tie_breaker = 0.01;                          // Added to k_T (the speeds are free, see the class)
    double heuristic_weight = 1.0;                           // Weight of the heuristic (1: A* of MOAR)
    bool direction_states = true;                            // States (cell, flight direction) of MOAR, else (cell, velocity)
    size_t max_expansions = 50000000;                        // The search fails after this number of expanded states
}; // struct MoarKinodynamicConfig


struct MoarKinodynamicStats
{
    size_t nb_of_expansions = 0;
    size_t nb_of_states = 0;
    double cost = 0.0;
}; // struct MoarKinodynamicStats


/**
 * @brief Kinodynamic version of the graph search of MOAR (Petit and Desbiens, ICRA 2024): the states of the A* search
 * are (cell, velocity) instead of (cell, flight direction), and the actions are double integrator motion primitives
 * (constant acceleration during tau) instead of straight steps flown at a constant speed, so that every trajectory found
 * respects the acceleration limit of the robot.
 *
 * - Velocity lattice: the inputs are u = k velocity_resolution / tau (k in {-max_input_steps, ..., max_input_steps} per
 *   axis), the velocities stay on the lattice of velocity_resolution, the positions are continuous.
 * - States: like MOAR, the cell of the position and the flight direction (one of the 26 directions of the grid, the
 *   components of the velocity rounded relatively to the largest one, or static at rest), each state holding the
 *   position and velocity of its cheapest arrival (hybrid A*, like the kinodynamic A* of Fast-Planner, whose states are
 *   the voxels only). With direction_states false, the states are (cell, velocity of the lattice). A primitive ending in
 *   the state of its parent is pruned.
 * - Constraints along a primitive (check_num points): valid cell function (clearance of the other benchmarks), speed
 *   ||v|| <= v(d_obs, theta) of MOAR (Eq. 5: minimum of v_sat(d_obs) and of the cruising speed of the flight direction,
 *   horizontal, diagonal or vertical by the elevation of the velocity), ||u|| <= max_acceleration.
 * - Costs of Eq. 7 of MOAR on a primitive: TC = tau, SC = sum C(d_obs) ds (safety cost integrated over the length),
 *   EC = ||v_end - v_start|| / (2 v_max) (velocity change). Heuristic of Eq. 8: k_T ||goal - p|| / v_max (euclidean
 *   instead of octile distance, the positions aren't on the grid) + k_E ||v|| / (2 v_max) (stop at the goal).
 * - Time tie-breaker: time_tie_breaker is added to k_T. The speeds of MOAR are given by Eq. 5, the duration of a path
 *   follows its geometry; here the search chooses them, and without a time cost (k_T = 0 in the weight sweep) slow or
 *   wandering trajectories in the free space (null safety cost) cost nothing: the tie-breaker keeps the fastest ones.
 * - Goal: like the static action of MOAR, a state within goal_radius of the goal is connected to the goal at rest by the
 *   cubic of the analytic expansion of Fast-Planner, with the shortest duration (candidates growing by 25 %) respecting
 *   the constraints; its costs (time, safety, ||a|| integrated for the velocity change) are added and the search ends
 *   when this goal state is expanded (A*).
 */
class MoarKinodynamicSearch
{
public:

    MoarKinodynamicSearch(const MoarKinodynamicConfig& a_config, MoarGraphSearch::CellFn a_cell_fn);
    ~MoarKinodynamicSearch();

    /**
     * @brief Searches the trajectory of minimal cost from a_start at rest to a_goal at rest.
     * @param a_trajectory Trajectory found, sampled every a_sampling_step (and at the end of every primitive).
     * @return true if a trajectory has been found.
     */
    bool search(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, const MoarWeights& a_weights,
                const MoarSpeeds& a_speeds, double a_sampling_step, NativeTrajectory& a_trajectory,
                MoarKinodynamicStats* a_stats = nullptr);

    /** @brief Forgets the cells (the map changed). */
    void clearCache();

    /** @brief Speed limit of MOAR (Eq. 5) for a velocity at a point of obstacle distance a_obstacle_distance. */
    static double speedLimit(const MoarSpeeds& a_speeds, double a_obstacle_distance, const Eigen::Vector3d& a_velocity);

private:

    struct Node;

    MoarGraphSearch::CellInfo cellInfo(const Eigen::Vector3d& a_point);
    bool inBounds(const Eigen::Vector3d& a_point) const;
    int64_t key(const Eigen::Vector3d& a_position, const Eigen::Vector3i& a_velocity) const;

    /**
     * @brief Costs (time, safety, energy) of a primitive from (p0, v0) with input u during tau, false if it violates a
     * constraint.
     */
    bool primitive(const Eigen::Vector3d& a_p0, const Eigen::Vector3d& a_v0, const Eigen::Vector3d& a_u, double a_tau,
                   const MoarSpeeds& a_speeds, Eigen::Vector3d& a_p1, MoarGraphSearch::CostVector& a_cost);

    /** @brief Connection to the goal at rest (cubic), false if no candidate duration respects the constraints. */
    bool connect(const Eigen::Vector3d& a_p0, const Eigen::Vector3d& a_v0, const Eigen::Vector3d& a_goal,
                 const MoarSpeeds& a_speeds, Eigen::Matrix<double, 3, 4>& a_coefficients, double& a_duration,
                 MoarGraphSearch::CostVector& a_cost);

    MoarKinodynamicConfig config_;
    MoarGraphSearch::CellFn cell_fn_;
    std::unordered_map<int64_t, MoarGraphSearch::CellInfo> cell_cache_; // Sparse cache (large maps)
    // Dense cache on the map bounds: safety cost and obstacle distance, safety cost -1 if invalid, NaN if unknown
    std::vector<std::array<float, 2>> dense_cache_;
    Eigen::Vector3i dense_size_ = Eigen::Vector3i::Zero();
    Eigen::Vector3i nb_of_cells_;

}; // class MoarKinodynamicSearch

}; // namespace arena_benchmarks
