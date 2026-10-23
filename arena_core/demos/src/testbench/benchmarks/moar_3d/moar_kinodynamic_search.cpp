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

#include "testbench/benchmarks/moar_3d/moar_kinodynamic_search.h"

// System
#include <algorithm>
#include <cmath>
#include <queue>
#include <tuple>


namespace arena_benchmarks
{

struct MoarKinodynamicSearch::Node
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3i velocity = Eigen::Vector3i::Zero(); // Lattice index (velocity = index * velocity_resolution)
    Eigen::Vector3d input = Eigen::Vector3d::Zero();    // Input of the primitive arriving at this node
    double g = 0.0;
    Node* parent = nullptr;
    bool closed = false;
    bool goal = false; // Goal at rest, reached by the connection from its parent
};

MoarKinodynamicSearch::MoarKinodynamicSearch(const MoarKinodynamicConfig& a_config, MoarGraphSearch::CellFn a_cell_fn)
: config_(a_config), cell_fn_(std::move(a_cell_fn))
{
    const Eigen::Vector3d extent = (config_.max_bounds - config_.min_bounds) / config_.resolution;
    nb_of_cells_ = (extent.array().floor() + 2.0).cast<int>();

    // Dense cache of the cell function on the map bounds when it fits (at most 64 M points)
    const Eigen::Vector3d points = (config_.max_bounds - config_.min_bounds) / config_.check_resolution;
    const Eigen::Vector3i size = (points.array().ceil() + 2.0).cast<int>();
    if (static_cast<double>(size.x()) * size.y() * size.z() <= 64.0e6)
    {
        dense_size_ = size;
        dense_cache_.assign(static_cast<size_t>(size.x()) * size.y() * size.z(), {NAN, 0.0f});
    }
}

void MoarKinodynamicSearch::clearCache()
{
    cell_cache_.clear();
    std::fill(dense_cache_.begin(), dense_cache_.end(), std::array<float, 2>{NAN, 0.0f});
}

MoarKinodynamicSearch::~MoarKinodynamicSearch() = default;

double MoarKinodynamicSearch::speedLimit(const MoarSpeeds& a_speeds, double a_obstacle_distance, const Eigen::Vector3d& a_velocity)
{
    // Below every limit (v_min is the lowest v_sat, the cruising speeds are checked below)
    const double lowest = std::min({a_speeds.v_min, a_speeds.horizontal, a_speeds.up, a_speeds.down, a_speeds.diagonal_up,
                                    a_speeds.diagonal_down});
    if (a_velocity.squaredNorm() <= lowest * lowest)
        return lowest;

    // Maximum speed variation with the obstacle distance (Eq. 5)
    double v_sat;
    if (a_obstacle_distance < a_speeds.d_min)
        v_sat = a_speeds.v_min;
    else if (a_obstacle_distance <= a_speeds.d_max)
        v_sat = a_speeds.v_min + (a_speeds.v_max - a_speeds.v_min) * (a_obstacle_distance - a_speeds.d_min) / (a_speeds.d_max - a_speeds.d_min);
    else
        v_sat = a_speeds.v_max;

    // Cruising speed of the flight direction: horizontal, diagonal or vertical (closest direction of the grid of MOAR by
    // the elevation of the velocity)
    const double horizontal = a_velocity.head<2>().norm();
    const double elevation = std::atan2(std::abs(a_velocity.z()), horizontal);
    double v_theta;
    if (elevation < M_PI / 8.0)
        v_theta = a_speeds.horizontal;
    else if (elevation > 3.0 * M_PI / 8.0)
        v_theta = a_velocity.z() > 0.0 ? a_speeds.up : a_speeds.down;
    else
        v_theta = a_velocity.z() > 0.0 ? a_speeds.diagonal_up : a_speeds.diagonal_down;

    return std::min(v_sat, v_theta);
}

MoarGraphSearch::CellInfo MoarKinodynamicSearch::cellInfo(const Eigen::Vector3d& a_point)
{
    // Cached on a grid of check_resolution (the cell function reads the octomap)
    if (!dense_cache_.empty())
    {
        const Eigen::Vector3i i = ((a_point - config_.min_bounds) / config_.check_resolution).array().round().cast<int>();
        if ((i.array() >= 0).all() && (i.array() < dense_size_.array()).all())
        {
            const size_t index = (static_cast<size_t>(i.x()) * dense_size_.y() + i.y()) * dense_size_.z() + i.z();
            std::array<float, 2>& cached = dense_cache_[index];
            if (std::isnan(cached[0]))
            {
                const MoarGraphSearch::CellInfo info = cell_fn_(config_.min_bounds + i.cast<double>() * config_.check_resolution);
                cached = {info.valid ? static_cast<float>(info.safety_cost) : -1.0f, static_cast<float>(info.obstacle_distance)};
            }
            MoarGraphSearch::CellInfo info;
            info.valid = cached[0] >= 0.0f;
            info.safety_cost = cached[0];
            info.obstacle_distance = cached[1];
            return info;
        }
    }
    const Eigen::Vector3i index = (a_point / config_.check_resolution).array().round().cast<int>();
    const int64_t offset = int64_t(1) << 20;
    const int64_t cache_key = ((index.x() + offset) << 42) | ((index.y() + offset) << 21) | (index.z() + offset);
    auto found = cell_cache_.find(cache_key);
    if (found != cell_cache_.end())
        return found->second;
    MoarGraphSearch::CellInfo info = cell_fn_(index.cast<double>() * config_.check_resolution);
    cell_cache_.emplace(cache_key, info);
    return info;
}

bool MoarKinodynamicSearch::inBounds(const Eigen::Vector3d& a_point) const
{
    return (a_point.array() >= config_.min_bounds.array()).all() && (a_point.array() <= config_.max_bounds.array()).all();
}

int64_t MoarKinodynamicSearch::key(const Eigen::Vector3d& a_position, const Eigen::Vector3i& a_velocity) const
{
    const Eigen::Vector3i cell = ((a_position - config_.min_bounds) / config_.resolution).array().round().cast<int>();
    const int64_t cell_index = (int64_t(cell.x()) * nb_of_cells_.y() + cell.y()) * nb_of_cells_.z() + cell.z();
    if (config_.direction_states)
    {
        // Flight direction: components rounded relatively to the largest one (static at rest)
        const int largest = a_velocity.cwiseAbs().maxCoeff();
        Eigen::Vector3i direction = Eigen::Vector3i::Zero();
        if (largest > 0)
            for (int i = 0; i < 3; ++i)
                direction(i) = static_cast<int>(std::lround(static_cast<double>(a_velocity(i)) / largest));
        return cell_index * 27 + ((direction.x() + 1) * 3 + direction.y() + 1) * 3 + direction.z() + 1;
    }
    // Velocity indices in [-64, 63]
    const int64_t velocity_index = ((int64_t(a_velocity.x()) + 64) << 14) | ((int64_t(a_velocity.y()) + 64) << 7) | (a_velocity.z() + 64);
    return (cell_index << 21) | velocity_index;
}

bool MoarKinodynamicSearch::primitive(const Eigen::Vector3d& a_p0, const Eigen::Vector3d& a_v0, const Eigen::Vector3d& a_u,
                                      double a_tau, const MoarSpeeds& a_speeds, Eigen::Vector3d& a_p1,
                                      MoarGraphSearch::CostVector& a_cost)
{
    double safety = 0.0;
    Eigen::Vector3d previous = a_p0;
    for (int k = 1; k <= config_.check_num; ++k)
    {
        const double t = a_tau * k / config_.check_num;
        const Eigen::Vector3d p = a_p0 + a_v0 * t + 0.5 * a_u * t * t;
        const Eigen::Vector3d v = a_v0 + a_u * t;
        if (!inBounds(p))
            return false;
        const MoarGraphSearch::CellInfo info = cellInfo(p);
        if (!info.valid || v.norm() > speedLimit(a_speeds, info.obstacle_distance, v) + 1.0e-9)
            return false;
        safety += info.safety_cost * (p - previous).norm();
        previous = p;
    }
    a_p1 = previous;
    a_cost = {a_tau, safety, (a_u * a_tau).norm() / (2.0 * a_speeds.v_max)};
    return true;
}

bool MoarKinodynamicSearch::connect(const Eigen::Vector3d& a_p0, const Eigen::Vector3d& a_v0, const Eigen::Vector3d& a_goal,
                                    const MoarSpeeds& a_speeds, Eigen::Matrix<double, 3, 4>& a_coefficients,
                                    double& a_duration, MoarGraphSearch::CostVector& a_cost)
{
    const Eigen::Vector3d dp = a_goal - a_p0;
    const Eigen::Vector3d dv = -a_v0; // To the goal at rest
    const int nb_of_samples = std::max(config_.check_num, static_cast<int>(std::ceil(dp.norm() / config_.check_resolution)) + 1);
    const double t_min = std::max(dp.norm() / a_speeds.v_max, 0.2);
    for (int candidate = 0; candidate < 25; ++candidate)
    {
        // Cubic of the analytic expansion of Fast-Planner (computeShotTraj): p(t) = p0 + v0 t + b t^2 + a t^3
        const double T = t_min * std::pow(1.25, candidate);
        const Eigen::Vector3d a = 1.0 / 6.0 * (-12.0 / (T * T * T) * (dp - a_v0 * T) + 6.0 / (T * T) * dv);
        const Eigen::Vector3d b = 0.5 * (6.0 / (T * T) * (dp - a_v0 * T) - 2.0 / T * dv);
        // The acceleration 2 b + 6 a t is linear: its norm is the largest at an end
        if ((2.0 * b).norm() > config_.max_acceleration || (2.0 * b + 6.0 * a * T).norm() > config_.max_acceleration)
            continue;

        bool feasible = true;
        double safety = 0.0, velocity_change = 0.0;
        Eigen::Vector3d previous = a_p0;
        for (int k = 1; k <= nb_of_samples && feasible; ++k)
        {
            const double t = T * k / nb_of_samples;
            const Eigen::Vector3d p = a_p0 + a_v0 * t + b * t * t + a * t * t * t;
            const Eigen::Vector3d v = a_v0 + 2.0 * b * t + 3.0 * a * t * t;
            const MoarGraphSearch::CellInfo info = cellInfo(p);
            if (!inBounds(p) || !info.valid || v.norm() > speedLimit(a_speeds, info.obstacle_distance, v) + 1.0e-9)
            {
                feasible = false;
                break;
            }
            safety += info.safety_cost * (p - previous).norm();
            // Velocity change: integral of ||a|| (midpoint of the step)
            const double t_mid = T * (k - 0.5) / nb_of_samples;
            velocity_change += (2.0 * b + 6.0 * a * t_mid).norm() * T / nb_of_samples;
            previous = p;
        }
        if (!feasible)
            continue;

        a_coefficients.col(0) = a_p0;
        a_coefficients.col(1) = a_v0;
        a_coefficients.col(2) = b;
        a_coefficients.col(3) = a;
        a_duration = T;
        a_cost = {T, safety, velocity_change / (2.0 * a_speeds.v_max)};
        return true;
    }
    return false;
}

bool MoarKinodynamicSearch::search(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, const MoarWeights& a_weights,
                                   const MoarSpeeds& a_speeds, double a_sampling_step, NativeTrajectory& a_trajectory,
                                   MoarKinodynamicStats* a_stats)
{
    a_trajectory.clear();
    const double v_res = config_.velocity_resolution;
    const double v_max = a_speeds.v_max;
    const double k_t = a_weights.k_t + config_.time_tie_breaker;
    auto cost = [&a_weights, k_t](const MoarGraphSearch::CostVector& c) { return k_t * c[0] + a_weights.k_s * c[1] + a_weights.k_e * c[2]; };
    auto heuristic = [&](const Eigen::Vector3d& p, const Eigen::Vector3d& v)
    {
        return config_.heuristic_weight * (k_t * (a_goal - p).norm() / v_max + a_weights.k_e * v.norm() / (2.0 * v_max));
    };

    // Inputs: velocity changes of the lattice within the acceleration limit
    std::vector<Eigen::Vector3i> steps;
    const int K = config_.max_input_steps;
    for (int x = -K; x <= K; ++x)
        for (int y = -K; y <= K; ++y)
            for (int z = -K; z <= K; ++z)
                if (Eigen::Vector3d(x, y, z).norm() * v_res / config_.tau <= config_.max_acceleration + 1.0e-9)
                    steps.emplace_back(x, y, z);

    std::vector<std::unique_ptr<Node>> pool;
    std::unordered_map<int64_t, Node*> states;
    states.reserve(1 << 20);
    using Entry = std::tuple<double, double, Node*>; // f, g at the push (lazy deletion), node
    auto greater = [](const Entry& a, const Entry& b) { return std::get<0>(a) > std::get<0>(b); };
    std::priority_queue<Entry, std::vector<Entry>, decltype(greater)> open(greater);

    pool.push_back(std::make_unique<Node>());
    Node* start = pool.back().get();
    start->position = a_start;
    states[key(a_start, start->velocity)] = start;
    open.emplace(heuristic(a_start, Eigen::Vector3d::Zero()), 0.0, start);

    Node* goal_node = nullptr;
    Eigen::Matrix<double, 3, 4> goal_coefficients = Eigen::Matrix<double, 3, 4>::Zero(); // p(t) = sum c_i t^i
    double goal_duration = 0.0;
    size_t nb_of_expansions = 0;
    bool found = false;
    while (!open.empty())
    {
        auto [f, g_at_push, node] = open.top();
        open.pop();
        if (node->closed || g_at_push != node->g)
            continue;
        if (node->goal)
        {
            found = true;
            break;
        }
        node->closed = true;
        if (++nb_of_expansions > config_.max_expansions)
            break;

        const Eigen::Vector3d v0 = node->velocity.cast<double>() * v_res;

        // Connection to the goal at rest (static action of MOAR at the goal)
        if ((a_goal - node->position).norm() <= config_.goal_radius)
        {
            Eigen::Matrix<double, 3, 4> coefficients;
            double duration;
            MoarGraphSearch::CostVector c;
            if (connect(node->position, v0, a_goal, a_speeds, coefficients, duration, c))
            {
                const double g = node->g + cost(c);
                if (!goal_node || g < goal_node->g)
                {
                    if (!goal_node)
                    {
                        pool.push_back(std::make_unique<Node>());
                        goal_node = pool.back().get();
                        goal_node->goal = true;
                        goal_node->position = a_goal;
                    }
                    goal_node->g = g;
                    goal_node->parent = node;
                    goal_coefficients = coefficients;
                    goal_duration = duration;
                    open.emplace(g, g, goal_node);
                }
            }
        }

        const int64_t node_key = key(node->position, node->velocity);
        for (const Eigen::Vector3i& step : steps)
        {
            const Eigen::Vector3i velocity = node->velocity + step;
            if (velocity.cast<double>().norm() * v_res > v_max + 1.0e-9)
                continue;
            const Eigen::Vector3d u = step.cast<double>() * v_res / config_.tau;
            // End state first (no check for a pruned or closed state)
            Eigen::Vector3d p1 = node->position + v0 * config_.tau + 0.5 * u * config_.tau * config_.tau;
            const int64_t next_key = key(p1, velocity);
            if (next_key == node_key)
                continue;
            auto known = states.find(next_key);
            if (known != states.end() && known->second->closed)
                continue;
            MoarGraphSearch::CostVector c;
            if (!primitive(node->position, v0, u, config_.tau, a_speeds, p1, c))
                continue;
            const double g = node->g + cost(c);
            auto existing = states.find(next_key);
            Node* next;
            if (existing == states.end())
            {
                pool.push_back(std::make_unique<Node>());
                next = pool.back().get();
                next->velocity = velocity;
                states.emplace(next_key, next);
            }
            else
            {
                next = existing->second;
                if (next->closed || g >= next->g)
                    continue;
            }
            next->position = p1;
            next->velocity = velocity; // The cheaper arrival may have another velocity of the same direction
            next->input = u;
            next->g = g;
            next->parent = node;
            open.emplace(g + heuristic(p1, velocity.cast<double>() * v_res), g, next);
        }
    }

    if (a_stats)
    {
        a_stats->nb_of_expansions = nb_of_expansions;
        a_stats->nb_of_states = states.size();
        a_stats->cost = found ? goal_node->g : 0.0;
    }
    if (!found)
        return false;

    // Primitives from the start, then the connection to the goal
    std::vector<Node*> chain;
    for (Node* n = goal_node->parent; n; n = n->parent)
        chain.push_back(n);
    std::reverse(chain.begin(), chain.end());
    a_trajectory.push_back({a_start, Eigen::Vector3d::Zero(), 0.0});
    for (size_t i = 1; i < chain.size(); ++i)
    {
        const TrajectoryPoint previous = a_trajectory.back();
        appendConstantAcceleration(a_trajectory, previous, chain[i]->input, config_.tau, a_sampling_step);
    }
    const double t0 = a_trajectory.back().time;
    const Eigen::Matrix<double, 3, 4>& c = goal_coefficients;
    const int nb_of_steps = std::max(1, static_cast<int>(std::ceil(goal_duration / a_sampling_step - 1.0e-9)));
    for (int k = 1; k <= nb_of_steps; ++k)
    {
        const double t = k == nb_of_steps ? goal_duration : k * a_sampling_step;
        TrajectoryPoint point;
        point.position = c.col(0) + c.col(1) * t + c.col(2) * t * t + c.col(3) * t * t * t;
        point.velocity = c.col(1) + 2.0 * c.col(2) * t + 3.0 * c.col(3) * t * t;
        point.time = t0 + t;
        a_trajectory.push_back(point);
    }
    return true;
}

}; // namespace arena_benchmarks
