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
#include <cstdint>
#include <functional>
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

struct PoRrtStarConfig
{
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();  // Sampling bounds
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Ones();
    double step_size = 5.0;               // eta: steering step, maximum edge length and goal tolerance (m)
    double rewire_factor = 1.1;           // Factor of gamma* of Karaman and Frazzoli (like OMPL's RRT*)
    double time_budget = 5.0;             // s
    double occupancy_threshold = 0.9;     // Cells with an occupancy probability at or above it are impassable
    double edge_resolution = 0.5;         // Sampling step along the edges for the collision and log-survival (m)
    double cell_size = 1.0;               // Size of the grid cells of the occupancy map (m)
    size_t max_nodes = 5000000;           // The tree stops growing at this size
    unsigned int seed = 0;
}; // struct PoRrtStarConfig


struct PoRrtStarPath
{
    std::vector<Eigen::Vector3d> waypoints; // From the start to the goal
    double length = 0.0;                    // d (m)
    double log_survival = 0.0;              // L <= 0
    double failure_probability = 0.0;       // Pfail = 1 - exp(L)
}; // struct PoRrtStarPath


struct PoRrtStarStats
{
    size_t nb_of_iterations = 0;
    size_t nb_of_nodes = 0;
    size_t nb_of_parent_changes = 0;      // Strict Pareto-improving rewires
    size_t nb_of_alternative_nodes = 0;   // Non-dominated branches created by the rewiring
    size_t nb_of_goal_nodes = 0;
}; // struct PoRrtStarStats


/**
 * @brief PO-RRT* of Batonisashvili ("PO-RRT*: Pareto-Optimal, Safety-Aware, Multi-Objective Path Planning", MS thesis,
 * Boston University, 2026), in 3D.
 *
 * The cost of a path is the vector (d, L) of its Euclidean length and its log-survival L = sum log(1 - q) over the cells of
 * the occupancy map crossed by its edges (q: occupancy probability of the cell), both additive (Ch. 3). Dominance (Def. 4.1):
 * shorter or equal and safer or equal (Pfail = 1 - exp(L)), one of them strictly. Every node of the tree has one cost pair,
 * several nodes can share a position (implicit value sets, Sec. 6.2). One iteration (Alg. A.1):
 *     - sample a collision-free x_rand, steer from the nearest node by eta to x_new, collision-free
 *     - CHOOSE-PARENTS (Alg. 1, A.2): the candidate costs through every collision-free neighbor within
 *       r_n = min(gamma (log n / n)^(1/d), eta) are Pareto-filtered, one node at x_new per non-dominated candidate
 *     - REWIRE (Alg. 2, A.3) around every new node: the candidate costs of the neighbors through it are Pareto-filtered;
 *       a candidate strictly dominating the cost of the neighbor rewires it (new parent, costs propagated to its subtree),
 *       a candidate non-dominated with it creates an alternative node at its position with the new node as parent
 *     - a new node closer to the goal than eta connects to it with a goal node
 * At the end, the global Pareto filter of the goal nodes gives the Pareto front of complete paths (Sec. 6.6).
 *
 * Details not given in the thesis: as in RRT* (Karaman and Frazzoli), the nearest node is always a candidate parent; an
 * alternative node is only created if the candidate isn't dominated by (or equal to) the cost of the neighbor (Fig. 6.1);
 * the tree grows during time_budget seconds instead of a number of iterations.
 */
class PoRrtStar
{
public:

    /** @brief True if a point is collision-free. */
    using ValidityFn = std::function<bool(const Eigen::Vector3d&)>;
    /** @brief Occupancy probability q in [0, 1] at a point. */
    using OccupancyFn = std::function<double(const Eigen::Vector3d&)>;

    PoRrtStar(const PoRrtStarConfig& a_config, ValidityFn a_valid, OccupancyFn a_occupancy);

    /** @brief Pareto front of the complete paths from a_start to a_goal (empty if the goal isn't reached). */
    std::vector<PoRrtStarPath> plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PoRrtStarStats* a_stats = nullptr);

    /**
     * @brief Length and log-survival of a straight edge, false if it isn't collision-free (a sample invalid or in a cell
     * of occupancy at or above the threshold).
     */
    bool edgeCost(const Eigen::Vector3d& a_from, const Eigen::Vector3d& a_to, double& a_length, double& a_log_survival) const;

private:

    PoRrtStarConfig config_;
    ValidityFn valid_;
    OccupancyFn occupancy_;

}; // class PoRrtStar

}; // namespace arena_benchmarks
