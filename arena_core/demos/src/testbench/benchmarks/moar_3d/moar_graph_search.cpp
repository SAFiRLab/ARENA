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

#include "testbench/benchmarks/moar_3d/moar_graph_search.h"

// System
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>


namespace arena_benchmarks
{

namespace
{

// Octile distance in cells extended to 3D: diagonal steps in 3D, then in 2D, then straight steps (Eq. 8)
double octileDistance(const Eigen::Vector3i& a_from, const Eigen::Vector3i& a_to)
{
    std::array<int, 3> d = {std::abs(a_to.x() - a_from.x()), std::abs(a_to.y() - a_from.y()), std::abs(a_to.z() - a_from.z())};
    std::sort(d.begin(), d.end(), std::greater<int>());
    return std::sqrt(3.0) * d[2] + std::sqrt(2.0) * (d[1] - d[2]) + (d[0] - d[1]);
}

} // namespace

MoarGraphSearch::MoarGraphSearch(const MoarGraphConfig& a_config, CellFn a_cell_fn)
: config_(a_config), cell_fn_(std::move(a_cell_fn))
{
    if (config_.resolution <= 0.0)
        throw std::invalid_argument("MoarGraphSearch: the resolution must be positive.");
    if (!cell_fn_)
        throw std::invalid_argument("MoarGraphSearch: the cell function is null.");

    Eigen::Vector3d extent = (config_.max_bounds - config_.min_bounds) / config_.resolution;
    size_ = Eigen::Vector3i(static_cast<int>(std::floor(extent.x())) + 1,
                            static_cast<int>(std::floor(extent.y())) + 1,
                            static_cast<int>(std::floor(extent.z())) + 1);
    if ((size_.array() <= 0).any())
        throw std::invalid_argument("MoarGraphSearch: the max bounds must be above the min bounds.");

    size_t nb_of_cells = static_cast<size_t>(size_.x()) * size_.y() * size_.z();
    if (nb_of_cells * kNbOfDirections > config_.max_nb_of_states)
        throw std::invalid_argument("MoarGraphSearch: the graph has too many states (" + std::to_string(nb_of_cells * kNbOfDirections) +
                                    "), increase the resolution.");

    cells_.assign(nb_of_cells, CellInfo());
    cell_known_.assign(nb_of_cells, 0);
}

Eigen::Vector3i MoarGraphSearch::cell(int64_t a_index) const
{
    int z = static_cast<int>(a_index % size_.z());
    int64_t xy = a_index / size_.z();
    return Eigen::Vector3i(static_cast<int>(xy / size_.y()), static_cast<int>(xy % size_.y()), z);
}

bool MoarGraphSearch::inGrid(const Eigen::Vector3i& a_cell) const
{
    return (a_cell.array() >= 0).all() && (a_cell.array() < size_.array()).all();
}

const MoarGraphSearch::CellInfo& MoarGraphSearch::info(const Eigen::Vector3i& a_cell)
{
    int64_t index = cellIndex(a_cell);
    if (!cell_known_[index])
    {
        cells_[index] = cell_fn_(center(a_cell));
        cell_known_[index] = 1;
    }
    return cells_[index];
}

void MoarGraphSearch::clearCache()
{
    std::fill(cell_known_.begin(), cell_known_.end(), 0);
}

double MoarGraphSearch::speed(const MoarSpeeds& a_speeds, double a_obstacle_distance, const Eigen::Vector3i& a_direction)
{
    if (a_direction.isZero())
        return 0.0;

    // Maximum speed variation with the obstacle distance (Eq. 5)
    double v_sat;
    if (a_obstacle_distance < a_speeds.d_min)
        v_sat = a_speeds.v_min;
    else if (a_obstacle_distance <= a_speeds.d_max)
        v_sat = a_speeds.v_min + (a_speeds.v_max - a_speeds.v_min) * (a_obstacle_distance - a_speeds.d_min) / (a_speeds.d_max - a_speeds.d_min);
    else
        v_sat = a_speeds.v_max;

    // Cruising speed of the direction of flight, defined for orthogonal and diagonal directions
    double v_theta;
    if (a_direction.z() == 0)
        v_theta = a_speeds.horizontal;
    else if (a_direction.x() == 0 && a_direction.y() == 0)
        v_theta = a_direction.z() > 0 ? a_speeds.up : a_speeds.down;
    else
        v_theta = a_direction.z() > 0 ? a_speeds.diagonal_up : a_speeds.diagonal_down;

    return std::min(v_sat, v_theta);
}

bool MoarGraphSearch::closestValidCell(const Eigen::Vector3d& a_point, int a_max_radius, Eigen::Vector3i& a_cell)
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
                    if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius)
                        continue;
                    Eigen::Vector3i candidate = nearest + Eigen::Vector3i(dx, dy, dz);
                    if (!inGrid(candidate) || !info(candidate).valid)
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

bool MoarGraphSearch::search(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, const MoarWeights& a_weights,
                             const MoarSpeeds& a_speeds, std::vector<MoarPathNode>& a_path, MoarSearchStats* a_stats)
{
    a_path.clear();

    Eigen::Vector3i start_cell, goal_cell;
    if (!closestValidCell(a_start, 2, start_cell) || !closestValidCell(a_goal, 2, goal_cell))
        return false;

    const double res = config_.resolution;
    const double v_max = a_speeds.v_max;
    const size_t nb_of_states = cells_.size() * kNbOfDirections;
    std::vector<float> g(nb_of_states, std::numeric_limits<float>::infinity());
    std::vector<int32_t> parent(nb_of_states, -1);
    std::vector<uint8_t> closed(nb_of_states, 0);

    // Unit vectors and lengths of the motion primitives
    std::array<Eigen::Vector3d, kNbOfDirections> unit;
    std::array<double, kNbOfDirections> length;
    for (int d = 0; d < kNbOfDirections; ++d)
    {
        Eigen::Vector3d v = direction(d).cast<double>();
        length[d] = res * v.norm();
        unit[d] = d == kStatic ? Eigen::Vector3d::Zero() : Eigen::Vector3d(v.normalized());
    }

    auto heuristic = [&](const Eigen::Vector3i& c, double v) -> double
    {
        // Minimum time to the goal at the maximum speed and velocity change to stop (Eq. 8)
        return a_weights.k_t * octileDistance(c, goal_cell) * res / v_max + a_weights.k_e * v / (2.0 * v_max);
    };

    using Entry = std::pair<float, int64_t>; // f, state
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    const int64_t start_state = cellIndex(start_cell) * kNbOfDirections + kStatic;
    g[start_state] = 0.0f;
    open.emplace(static_cast<float>(heuristic(start_cell, 0.0)), start_state);

    size_t nb_of_expansions = 0;
    int64_t goal_state = -1;
    while (!open.empty())
    {
        int64_t state = open.top().second;
        open.pop();
        if (closed[state])
            continue;
        closed[state] = 1;

        const Eigen::Vector3i current_cell = cell(state / kNbOfDirections);
        const int current_direction = static_cast<int>(state % kNbOfDirections);

        // The goal is reached, the drone stops there with the static action (included in the heuristic)
        if (current_cell == goal_cell)
        {
            goal_state = state;
            break;
        }

        if (++nb_of_expansions > config_.max_expansions)
            break;

        const double current_speed = speed(a_speeds, info(current_cell).obstacle_distance, direction(current_direction));
        const Eigen::Vector3d current_velocity = unit[current_direction] * current_speed;

        for (int d = 0; d < kNbOfDirections; ++d)
        {
            if (d == kStatic)
            {
                // Static action "0" (Fig. 5, 9 actions in 2D): the drone stops in its cell. Costs of the MOAR
                // implementation (MOPP): branch length of a diagonal step (getBranchDist), time cost of 1 at a null speed
                // (computeTimeCost), velocity change to stop
                const int64_t static_state = cellIndex(current_cell) * kNbOfDirections + kStatic;
                if (closed[static_state])
                    continue;
                const double static_g = g[state]
                                      + a_weights.k_s * info(current_cell).safety_cost * staticBranchLength()
                                      + a_weights.k_t * kStaticTimeCost
                                      + a_weights.k_e * current_velocity.norm() / (2.0 * v_max);
                if (static_g < g[static_state])
                {
                    g[static_state] = static_cast<float>(static_g);
                    parent[static_state] = static_cast<int32_t>(state);
                    open.emplace(static_cast<float>(static_g + heuristic(current_cell, 0.0)), static_state);
                }
                continue;
            }
            const Eigen::Vector3i next_cell = current_cell + direction(d);
            if (!inGrid(next_cell))
                continue;
            const CellInfo& next_info = info(next_cell);
            if (!next_info.valid)
                continue;
            const int64_t next_state = cellIndex(next_cell) * kNbOfDirections + d;
            if (closed[next_state])
                continue;

            const double next_speed = speed(a_speeds, next_info.obstacle_distance, direction(d));
            const double velocity_change = (unit[d] * next_speed - current_velocity).norm() / (2.0 * v_max);

            // Cost of the path to the next state (Eq. 7)
            const double next_g = g[state]
                                + a_weights.k_s * next_info.safety_cost * length[d]
                                + a_weights.k_t * length[d] / next_speed
                                + a_weights.k_e * velocity_change;
            if (next_g < g[next_state])
            {
                g[next_state] = static_cast<float>(next_g);
                parent[next_state] = static_cast<int32_t>(state);
                open.emplace(static_cast<float>(next_g + heuristic(next_cell, next_speed)), next_state);
            }
        }
    }

    if (a_stats)
    {
        a_stats->nb_of_expansions = nb_of_expansions;
        a_stats->cost = goal_state >= 0 ? g[goal_state] : std::numeric_limits<double>::infinity();
    }
    if (goal_state < 0)
        return false;

    std::vector<int64_t> states;
    for (int64_t s = goal_state; s >= 0; s = parent[s])
        states.push_back(s);
    std::reverse(states.begin(), states.end());

    for (int64_t s : states)
    {
        const Eigen::Vector3i c = cell(s / kNbOfDirections);
        const int d = static_cast<int>(s % kNbOfDirections);
        a_path.push_back({center(c), speed(a_speeds, info(c).obstacle_distance, direction(d))});
    }
    // The start and the goal replace the centers of their cells
    a_path.front().position = a_start;
    a_path.front().speed = 0.0;
    if (a_path.size() == 1)
        a_path.push_back({a_goal, 0.0});
    else
        a_path.back().position = a_goal;

    return true;
}

bool MoarGraphSearch::startAndGoal(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, int64_t& a_start_state,
                                   Eigen::Vector3i& a_goal_cell)
{
    Eigen::Vector3i start_cell;
    if (!closestValidCell(a_start, 2, start_cell) || !closestValidCell(a_goal, 2, a_goal_cell))
        return false;
    a_start_state = cellIndex(start_cell) * kNbOfDirections + kStatic;
    return true;
}

double MoarGraphSearch::stateSpeed(int64_t a_state, const MoarSpeeds& a_speeds)
{
    const Eigen::Vector3i c = cell(a_state / kNbOfDirections);
    return speed(a_speeds, info(c).obstacle_distance, direction(static_cast<int>(a_state % kNbOfDirections)));
}

void MoarGraphSearch::successors(int64_t a_state, const MoarSpeeds& a_speeds, std::vector<Edge>& a_edges)
{
    a_edges.clear();
    const double res = config_.resolution;
    const double v_max = a_speeds.v_max;
    const Eigen::Vector3i current_cell = cell(a_state / kNbOfDirections);
    const int current_direction = static_cast<int>(a_state % kNbOfDirections);

    // Same costs as search() (Eq. 7)
    const double current_speed = speed(a_speeds, info(current_cell).obstacle_distance, direction(current_direction));
    const Eigen::Vector3d current_velocity = current_direction == kStatic ?
        Eigen::Vector3d::Zero() : Eigen::Vector3d(direction(current_direction).cast<double>().normalized() * current_speed);

    for (int d = 0; d < kNbOfDirections; ++d)
    {
        if (d == kStatic)
        {
            // Static action, same costs as search()
            if (current_direction != kStatic)
                a_edges.push_back({cellIndex(current_cell) * kNbOfDirections + kStatic,
                                   {kStaticTimeCost, info(current_cell).safety_cost * staticBranchLength(),
                                    current_velocity.norm() / (2.0 * v_max)}});
            continue;
        }
        const Eigen::Vector3i next_cell = current_cell + direction(d);
        if (!inGrid(next_cell))
            continue;
        const CellInfo& next_info = info(next_cell);
        if (!next_info.valid)
            continue;

        const Eigen::Vector3d step = direction(d).cast<double>();
        const double length = res * step.norm();
        const double next_speed = speed(a_speeds, next_info.obstacle_distance, direction(d));
        const double velocity_change = (step.normalized() * next_speed - current_velocity).norm() / (2.0 * v_max);
        a_edges.push_back({cellIndex(next_cell) * kNbOfDirections + d,
                           {length / next_speed, next_info.safety_cost * length, velocity_change}});
    }
}

void MoarGraphSearch::costToGo(const Eigen::Vector3i& a_goal_cell, const MoarSpeeds& a_speeds, std::vector<double>& a_time,
                               std::vector<double>& a_safety)
{
    const double res = config_.resolution;
    const double infinity = std::numeric_limits<double>::infinity();

    // One backward Dijkstra per objective: the cost of the edge p -> c depends on the cell c entered and the direction
    auto dijkstra = [&](std::vector<double>& a_cost, bool a_time_cost)
    {
        a_cost.assign(cells_.size(), infinity);
        using Entry = std::pair<double, int64_t>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        a_cost[cellIndex(a_goal_cell)] = 0.0;
        open.emplace(0.0, cellIndex(a_goal_cell));
        while (!open.empty())
        {
            auto [cost, index] = open.top();
            open.pop();
            if (cost > a_cost[index])
                continue;
            const Eigen::Vector3i c = cell(index);
            const CellInfo& c_info = info(c);
            for (int d = 0; d < kNbOfDirections; ++d)
            {
                if (d == kStatic)
                    continue;
                // Predecessor p entering c with the direction d
                const Eigen::Vector3i p = c - direction(d);
                if (!inGrid(p) || !info(p).valid)
                    continue;
                const double length = res * direction(d).cast<double>().norm();
                const double edge = a_time_cost ? length / speed(a_speeds, c_info.obstacle_distance, direction(d))
                                                : c_info.safety_cost * length;
                const int64_t p_index = cellIndex(p);
                if (cost + edge < a_cost[p_index])
                {
                    a_cost[p_index] = cost + edge;
                    open.emplace(cost + edge, p_index);
                }
            }
        }
    };
    dijkstra(a_time, true);
    dijkstra(a_safety, false);
}

void MoarGraphSearch::pathOfStates(const std::vector<int64_t>& a_states, const Eigen::Vector3d& a_start,
                                   const Eigen::Vector3d& a_goal, const MoarSpeeds& a_speeds, std::vector<MoarPathNode>& a_path)
{
    a_path.clear();
    for (int64_t s : a_states)
    {
        const Eigen::Vector3i c = cell(s / kNbOfDirections);
        const int d = static_cast<int>(s % kNbOfDirections);
        a_path.push_back({center(c), speed(a_speeds, info(c).obstacle_distance, direction(d))});
    }
    if (a_path.empty())
        return;
    a_path.front().position = a_start;
    a_path.front().speed = 0.0;
    if (a_path.size() == 1)
        a_path.push_back({a_goal, 0.0});
    else
        a_path.back().position = a_goal;
}

}; // namespace arena_benchmarks
