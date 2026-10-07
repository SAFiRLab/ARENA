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

struct WeightedAStar3DConfig
{
    double resolution = 1.0;                                 // Size of a grid cell in meters
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();    // Center of the first cell
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Zero();    // The grid covers the cells up to this point
    size_t max_expansions = 20000000;                        // The search fails after this number of expanded cells
    size_t max_nb_of_cells = 60000000;                       // Larger grids are refused (dense arrays)
}; // struct WeightedAStar3DConfig


struct WeightedAStar3DStats
{
    size_t nb_of_expansions = 0;
    double cost = 0.0; // Cost of the path found
}; // struct WeightedAStar3DStats


/**
 * @brief A* on a regular 3D grid (26-connectivity) with a user-defined edge cost.
 *
 * The cells are valid or not according to a validity function evaluated lazily and cached until clearCache(). The edge
 * cost and the heuristic are given at every search, the heuristic must be admissible for the path to be optimal.
 * The path starts at the given start, goes through the centers of the cells and ends at the given goal.
 */
class WeightedAStar3D
{
public:

    using ValidityFn = std::function<bool(const Eigen::Vector3d&)>;
    using EdgeCostFn = std::function<double(const Eigen::Vector3d& from, const Eigen::Vector3d& to)>;
    using HeuristicFn = std::function<double(const Eigen::Vector3d& from, const Eigen::Vector3d& goal)>;

    WeightedAStar3D(const WeightedAStar3DConfig& a_config, ValidityFn a_validity);

    /**
     * @brief Searches the path of minimal cost between a_start and a_goal.
     * @param a_path Points of the path (start, cell centers, goal) if found.
     * @return true if a path has been found.
     */
    bool search(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, const EdgeCostFn& a_edge_cost,
                const HeuristicFn& a_heuristic, std::vector<Eigen::Vector3d>& a_path, WeightedAStar3DStats* a_stats = nullptr);

    /** @brief Forgets the validity of the cells (the map changed). */
    void clearCache();

    const WeightedAStar3DConfig& getConfig() const { return config_; }

private:

    int64_t index(const Eigen::Vector3i& a_cell) const { return (static_cast<int64_t>(a_cell.x()) * size_.y() + a_cell.y()) * size_.z() + a_cell.z(); }
    Eigen::Vector3i cell(int64_t a_index) const;
    Eigen::Vector3d center(const Eigen::Vector3i& a_cell) const { return config_.min_bounds + a_cell.cast<double>() * config_.resolution; }
    bool inGrid(const Eigen::Vector3i& a_cell) const;
    bool isValid(const Eigen::Vector3i& a_cell);

    /** @brief Closest valid cell to a point, searched in growing cubes up to a_max_radius cells. */
    bool closestValidCell(const Eigen::Vector3d& a_point, int a_max_radius, Eigen::Vector3i& a_cell);

    WeightedAStar3DConfig config_;
    ValidityFn validity_;
    Eigen::Vector3i size_;
    std::vector<int8_t> validity_cache_; // -1 unknown, 0 invalid, 1 valid

}; // class WeightedAStar3D

}; // namespace arena_benchmarks
