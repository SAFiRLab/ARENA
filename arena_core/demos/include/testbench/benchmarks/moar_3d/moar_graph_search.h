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
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

/**
 * @brief Speeds of the MOAR planner (Petit and Desbiens, ICRA 2024, Eq. 5): v(d_obs, theta) = min(v_sat(d_obs), v_max,theta).
 */
struct MoarSpeeds
{
    double v_min = 0.4;               // Speed close to the obstacles (d_obs <= d_min)
    double v_max = 0.5;               // Maximum speed, the horizontal cruising speed (d_obs >= d_max)
    double d_min = 2.0;               // v_sat = v_min below this obstacle distance (m)
    double d_max = 4.0;               // v_sat = v_max above this obstacle distance (m)
    // Cruising speed of every flight direction (m/s)
    double up = 0.5;
    double diagonal_up = 0.5;
    double horizontal = 0.5;
    double diagonal_down = 0.5;
    double down = 0.5;
}; // struct MoarSpeeds


/**
 * @brief Coefficients of the cost function of MOAR (Eq. 1): safety, time and energy.
 */
struct MoarWeights
{
    double k_s = 1.0 / 3.0;
    double k_t = 1.0 / 3.0;
    double k_e = 1.0 / 3.0;
}; // struct MoarWeights


struct MoarGraphConfig
{
    double resolution = 1.0;                                 // Size of a grid cell in meters
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();    // Center of the first cell
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Zero();    // The grid covers the cells up to this point
    size_t max_expansions = 50000000;                        // The search fails after this number of expanded states
    size_t max_nb_of_states = 150000000;                     // Larger graphs are refused (dense arrays)
}; // struct MoarGraphConfig


/**
 * @brief Node of a path found by the MOAR graph search.
 */
struct MoarPathNode
{
    Eigen::Vector3d position; // Cell center (the start and the goal for the first and last nodes)
    double speed;             // Speed on the segment arriving at this node (0 at the start)
}; // struct MoarPathNode


struct MoarSearchStats
{
    size_t nb_of_expansions = 0;
    double cost = 0.0;
}; // struct MoarSearchStats


/**
 * @brief Graph search of the MOAR planner (Petit and Desbiens, "MOAR Planner: Multi-Objective and Adaptive Risk-Aware Path
 * Planning for Infrastructure Inspection with a UAV", ICRA 2024), extended from a 2D plane to 3D.
 *
 * As in the paper (Sec. IV), the graph states are (cell, flight direction theta): every state connects to its neighbors
 * with the motion primitives of a regular grid (26 directions in 3D instead of 8 in 2D, plus the static action used to stop
 * at the goal). The cost of a path to a state n (Eq. 7) is
 *     g_n = sum k_S SC_i d_i + sum k_T TC_i + sum k_E EC_i
 * with SC_i the safety cost of the cell (normalized in [0, 1], integrated over the segment length d_i), TC_i = d_i / v_i
 * the time cost with the speed v_i = v(d_obs, theta) of the cell in the direction of flight (Eq. 4-5), and
 * EC_i = ||v_i - v_{i-1}|| / (2 v_max) the energy cost of the velocity change (Eq. 6). The heuristic (Eq. 8) is
 *     h_n = k_T d(n, goal) / v_max + k_E EC(v_n -> 0)
 * with d the octile distance (diagonal and straight steps, extended to 3D). The search is A* and ends when a state of the
 * goal cell is expanded (it is then stopped with the static action, the heuristic includes this last velocity change).
 *
 * The cells are valid or not, and have a safety cost and an obstacle distance, given by a function evaluated lazily.
 */
class MoarGraphSearch
{
public:

    struct CellInfo
    {
        bool valid = false;
        double safety_cost = 1.0;        // SC in [0, 1]
        double obstacle_distance = 0.0;  // d_obs (m), used by the speed function
    };
    using CellFn = std::function<CellInfo(const Eigen::Vector3d&)>;

    MoarGraphSearch(const MoarGraphConfig& a_config, CellFn a_cell_fn);

    /**
     * @brief Searches the path of minimal cost between a_start and a_goal.
     * @param a_path Nodes of the path with their speeds (start, cell centers, goal) if found.
     * @return true if a path has been found.
     */
    bool search(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, const MoarWeights& a_weights,
                const MoarSpeeds& a_speeds, std::vector<MoarPathNode>& a_path, MoarSearchStats* a_stats = nullptr);

    /** @brief Forgets the cells (the map changed). */
    void clearCache();

    const MoarGraphConfig& getConfig() const { return config_; }

    /** @brief Speed v(d_obs, theta) of the MOAR paper (Eq. 5) for a direction of the grid (dx, dy, dz in {-1, 0, 1}). */
    static double speed(const MoarSpeeds& a_speeds, double a_obstacle_distance, const Eigen::Vector3i& a_direction);

    /********************* Read-only access to the graph, for other searches on the same graph *********************/

    static constexpr int kNbOfDirections = 27;
    static constexpr int kStatic = 13; // Direction (0, 0, 0)
    static constexpr double kStaticTimeCost = 1.0; // Time cost of the static action (computeTimeCost of MOAR at a null speed)

    /** @brief Costs (time, safety, energy) of an edge: the three terms of Eq. 7 without the coefficients. */
    using CostVector = std::array<double, 3>;

    struct Edge
    {
        int64_t state;   // State reached (cell * kNbOfDirections + arrival direction)
        CostVector cost; // TC = d / v, SC * d, EC = ||v - v_prev|| / (2 v_max)
    };

    /**
     * @brief Start state (closest valid cell to a_start, static direction) and goal cell (closest valid cell to a_goal),
     * like search().
     * @return false if there is no valid cell close to the start or the goal.
     */
    bool startAndGoal(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, int64_t& a_start_state,
                      Eigen::Vector3i& a_goal_cell);

    /** @brief Edges from a state to the states of the valid neighbor cells, with the costs of search(). */
    void successors(int64_t a_state, const MoarSpeeds& a_speeds, std::vector<Edge>& a_edges);

    /** @brief Speed of the segment arriving at a state (0 for the static direction). */
    double stateSpeed(int64_t a_state, const MoarSpeeds& a_speeds);

    /** @brief Cell of a state. */
    Eigen::Vector3i stateCell(int64_t a_state) const { return cell(a_state / kNbOfDirections); }

    /**
     * @brief Lower bounds of the time and safety costs from every cell to the goal cell: backward Dijkstra on the cells
     * with the time cost d / v and the safety cost SC * d of the cell entered (the costs of an edge don't depend on
     * the previous direction). Infinity for the cells that can't reach the goal.
     */
    void costToGo(const Eigen::Vector3i& a_goal_cell, const MoarSpeeds& a_speeds, std::vector<double>& a_time,
                  std::vector<double>& a_safety);

    /** @brief Index of a cell in the vectors of costToGo(). */
    int64_t cellIndexOf(const Eigen::Vector3i& a_cell) const { return cellIndex(a_cell); }

    /**
     * @brief Path of a sequence of states, like search(): cell centers and speeds, the start and the goal replace the
     * centers of the first and last cells.
     */
    void pathOfStates(const std::vector<int64_t>& a_states, const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal,
                      const MoarSpeeds& a_speeds, std::vector<MoarPathNode>& a_path);

private:

    int64_t cellIndex(const Eigen::Vector3i& a_cell) const { return (static_cast<int64_t>(a_cell.x()) * size_.y() + a_cell.y()) * size_.z() + a_cell.z(); }
    Eigen::Vector3i cell(int64_t a_index) const;
    Eigen::Vector3d center(const Eigen::Vector3i& a_cell) const { return config_.min_bounds + a_cell.cast<double>() * config_.resolution; }
    bool inGrid(const Eigen::Vector3i& a_cell) const;
    const CellInfo& info(const Eigen::Vector3i& a_cell);
    bool closestValidCell(const Eigen::Vector3d& a_point, int a_max_radius, Eigen::Vector3i& a_cell);
    /** @brief Branch length of the static action in the safety cost (getBranchDist of MOAR: a diagonal step). */
    double staticBranchLength() const { return config_.resolution * std::sqrt(2.0); }

    static Eigen::Vector3i direction(int a_index) { return Eigen::Vector3i(a_index / 9 - 1, (a_index / 3) % 3 - 1, a_index % 3 - 1); }

    MoarGraphConfig config_;
    CellFn cell_fn_;
    Eigen::Vector3i size_;
    std::vector<CellInfo> cells_;
    std::vector<uint8_t> cell_known_;

}; // class MoarGraphSearch

}; // namespace arena_benchmarks
