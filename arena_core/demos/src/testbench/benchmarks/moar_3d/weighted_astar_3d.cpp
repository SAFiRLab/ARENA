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

#include "testbench/benchmarks/moar_3d/weighted_astar_3d.h"

// System
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>


namespace arena_benchmarks
{

WeightedAStar3D::WeightedAStar3D(const WeightedAStar3DConfig& a_config, ValidityFn a_validity)
: config_(a_config), validity_(std::move(a_validity))
{
    if (config_.resolution <= 0.0)
        throw std::invalid_argument("WeightedAStar3D: the resolution must be positive.");
    if (!validity_)
        throw std::invalid_argument("WeightedAStar3D: the validity function is null.");

    Eigen::Vector3d extent = (config_.max_bounds - config_.min_bounds) / config_.resolution;
    size_ = Eigen::Vector3i(static_cast<int>(std::floor(extent.x())) + 1,
                            static_cast<int>(std::floor(extent.y())) + 1,
                            static_cast<int>(std::floor(extent.z())) + 1);
    if ((size_.array() <= 0).any())
        throw std::invalid_argument("WeightedAStar3D: the max bounds must be above the min bounds.");

    size_t nb_of_cells = static_cast<size_t>(size_.x()) * size_.y() * size_.z();
    if (nb_of_cells > config_.max_nb_of_cells)
        throw std::invalid_argument("WeightedAStar3D: the grid has too many cells (" + std::to_string(nb_of_cells) +
                                    "), increase the resolution.");

    validity_cache_.assign(nb_of_cells, -1);
}

Eigen::Vector3i WeightedAStar3D::cell(int64_t a_index) const
{
    int z = static_cast<int>(a_index % size_.z());
    int64_t xy = a_index / size_.z();
    int y = static_cast<int>(xy % size_.y());
    int x = static_cast<int>(xy / size_.y());
    return Eigen::Vector3i(x, y, z);
}

bool WeightedAStar3D::inGrid(const Eigen::Vector3i& a_cell) const
{
    return (a_cell.array() >= 0).all() && (a_cell.array() < size_.array()).all();
}

bool WeightedAStar3D::isValid(const Eigen::Vector3i& a_cell)
{
    int8_t& cached = validity_cache_[index(a_cell)];
    if (cached < 0)
        cached = validity_(center(a_cell)) ? 1 : 0;
    return cached == 1;
}

void WeightedAStar3D::clearCache()
{
    std::fill(validity_cache_.begin(), validity_cache_.end(), -1);
}

bool WeightedAStar3D::closestValidCell(const Eigen::Vector3d& a_point, int a_max_radius, Eigen::Vector3i& a_cell)
{
    Eigen::Vector3d grid_point = (a_point - config_.min_bounds) / config_.resolution;
    Eigen::Vector3i nearest(static_cast<int>(std::lround(grid_point.x())), static_cast<int>(std::lround(grid_point.y())),
                            static_cast<int>(std::lround(grid_point.z())));

    for (int radius = 0; radius <= a_max_radius; ++radius)
    {
        double best_distance = std::numeric_limits<double>::max();
        for (int dx = -radius; dx <= radius; ++dx)
            for (int dy = -radius; dy <= radius; ++dy)
                for (int dz = -radius; dz <= radius; ++dz)
                {
                    // Only the shell of the cube, the inside has been checked with the smaller radii
                    if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius)
                        continue;
                    Eigen::Vector3i candidate = nearest + Eigen::Vector3i(dx, dy, dz);
                    if (!inGrid(candidate) || !isValid(candidate))
                        continue;
                    double distance = (center(candidate) - a_point).norm();
                    if (distance < best_distance)
                    {
                        best_distance = distance;
                        a_cell = candidate;
                    }
                }
        if (best_distance < std::numeric_limits<double>::max())
            return true;
    }
    return false;
}

bool WeightedAStar3D::search(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, const EdgeCostFn& a_edge_cost,
                             const HeuristicFn& a_heuristic, std::vector<Eigen::Vector3d>& a_path, WeightedAStar3DStats* a_stats)
{
    a_path.clear();

    // The start and the goal are connected to their closest valid cells
    Eigen::Vector3i start_cell, goal_cell;
    if (!closestValidCell(a_start, 2, start_cell) || !closestValidCell(a_goal, 2, goal_cell))
        return false;

    const size_t nb_of_cells = validity_cache_.size();
    std::vector<double> g(nb_of_cells, std::numeric_limits<double>::infinity());
    std::vector<int64_t> parent(nb_of_cells, -1);
    std::vector<bool> closed(nb_of_cells, false);

    using Entry = std::pair<double, int64_t>; // f, cell index
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    const int64_t start_index = index(start_cell);
    const int64_t goal_index = index(goal_cell);
    const Eigen::Vector3d goal_center = center(goal_cell);
    g[start_index] = 0.0;
    open.emplace(a_heuristic(center(start_cell), goal_center), start_index);

    size_t nb_of_expansions = 0;
    bool found = false;
    while (!open.empty())
    {
        int64_t current_index = open.top().second;
        open.pop();
        if (closed[current_index])
            continue; // Outdated entry
        closed[current_index] = true;

        if (current_index == goal_index)
        {
            found = true;
            break;
        }

        if (++nb_of_expansions > config_.max_expansions)
            break;

        const Eigen::Vector3i current_cell = cell(current_index);
        const Eigen::Vector3d current_center = center(current_cell);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;
                    Eigen::Vector3i neighbor_cell = current_cell + Eigen::Vector3i(dx, dy, dz);
                    if (!inGrid(neighbor_cell))
                        continue;
                    int64_t neighbor_index = index(neighbor_cell);
                    if (closed[neighbor_index] || !isValid(neighbor_cell))
                        continue;

                    Eigen::Vector3d neighbor_center = center(neighbor_cell);
                    double tentative_g = g[current_index] + a_edge_cost(current_center, neighbor_center);
                    if (tentative_g < g[neighbor_index])
                    {
                        g[neighbor_index] = tentative_g;
                        parent[neighbor_index] = current_index;
                        open.emplace(tentative_g + a_heuristic(neighbor_center, goal_center), neighbor_index);
                    }
                }
    }

    if (a_stats)
    {
        a_stats->nb_of_expansions = nb_of_expansions;
        a_stats->cost = found ? g[goal_index] : std::numeric_limits<double>::infinity();
    }

    if (!found)
        return false;

    std::vector<Eigen::Vector3d> reversed;
    for (int64_t i = goal_index; i >= 0; i = parent[i])
        reversed.push_back(center(cell(i)));

    // The start and the goal replace the centers of their cells (first and last cells of the search)
    a_path.push_back(a_start);
    for (int i = static_cast<int>(reversed.size()) - 2; i >= 1; --i)
        a_path.push_back(reversed[i]);
    a_path.push_back(a_goal);

    return true;
}

}; // namespace arena_benchmarks
