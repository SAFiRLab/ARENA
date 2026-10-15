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

#include "testbench/benchmarks/po_rrt_star/po_rrt_star.h"

// External Libraries
// OMPL
#include <ompl/datastructures/NearestNeighborsGNAT.h>

// System
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>


namespace arena_benchmarks
{

namespace
{

struct TreeNode
{
    Eigen::Vector3d position;
    double length;         // d: from the start
    double log_survival;   // L: from the start
    double p_fail;         // 1 - exp(L)
    int64_t parent;        // -1 for the root
    double edge_length;    // Costs of the edge from the parent
    double edge_log_survival;
    std::vector<int64_t> children;
    bool goal = false;
};

// Pareto dominance (Def. 4.1): no longer and no less safe, one of them strictly
bool dominates(double a_length, double a_p_fail, double b_length, double b_p_fail)
{
    return a_length <= b_length && a_p_fail <= b_p_fail && (a_length < b_length || a_p_fail < b_p_fail);
}

struct Candidate
{
    int64_t node;          // Parent (CHOOSE-PARENTS) or neighbor to rewire (REWIRE)
    double length;
    double log_survival;
    double p_fail;
    double edge_length;
    double edge_log_survival;
};

// ParetoFilter: the candidates not dominated by another candidate (one of the candidates with equal costs)
std::vector<Candidate> paretoFilter(const std::vector<Candidate>& a_candidates)
{
    std::vector<Candidate> result;
    for (size_t a = 0; a < a_candidates.size(); ++a)
    {
        bool dominated = false;
        for (size_t b = 0; b < a_candidates.size() && !dominated; ++b)
        {
            if (a == b)
                continue;
            const Candidate& ca = a_candidates[a];
            const Candidate& cb = a_candidates[b];
            dominated = dominates(cb.length, cb.p_fail, ca.length, ca.p_fail) ||
                        (b < a && cb.length == ca.length && cb.p_fail == ca.p_fail);
        }
        if (!dominated)
            result.push_back(a_candidates[a]);
    }
    return result;
}

}; // namespace


PoRrtStar::PoRrtStar(const PoRrtStarConfig& a_config, ValidityFn a_valid, OccupancyFn a_occupancy)
: config_(a_config), valid_(std::move(a_valid)), occupancy_(std::move(a_occupancy))
{
    if (!valid_ || !occupancy_)
        throw std::invalid_argument("PoRrtStar: null validity or occupancy function.");
    if (config_.step_size <= 0.0 || config_.edge_resolution <= 0.0 || config_.cell_size <= 0.0)
        throw std::invalid_argument("PoRrtStar: the step size, edge resolution and cell size must be positive.");
}

bool PoRrtStar::edgeCost(const Eigen::Vector3d& a_from, const Eigen::Vector3d& a_to, double& a_length, double& a_log_survival) const
{
    a_length = (a_to - a_from).norm();
    a_log_survival = 0.0;

    // Cells of the occupancy map crossed by the edge, the cell of its origin belongs to the previous edge
    auto cellOf = [this](const Eigen::Vector3d& p) -> Eigen::Vector3i
    {
        Eigen::Vector3d c = ((p - config_.min_bounds) / config_.cell_size).array().floor();
        return c.cast<int>();
    };
    std::vector<Eigen::Vector3i> crossed = {cellOf(a_from)};

    const int nb_of_steps = std::max(1, static_cast<int>(std::ceil(a_length / config_.edge_resolution)));
    for (int i = 1; i <= nb_of_steps; ++i)
    {
        const Eigen::Vector3d p = a_from + (a_to - a_from) * (static_cast<double>(i) / nb_of_steps);
        if (!valid_(p))
            return false;
        const Eigen::Vector3i c = cellOf(p);
        if (std::find(crossed.begin(), crossed.end(), c) != crossed.end())
            continue;
        crossed.push_back(c);

        // Log-survival of the cell (Eq. 3.6), impassable cells at or above the occupancy threshold
        const double q = std::clamp(occupancy_(p), 0.0, 1.0);
        if (q >= config_.occupancy_threshold)
            return false;
        a_log_survival += std::log1p(-q);
    }
    return true;
}

std::vector<PoRrtStarPath> PoRrtStar::plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PoRrtStarStats* a_stats)
{
    const auto start_time = std::chrono::steady_clock::now();
    auto elapsed = [&start_time]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };

    std::mt19937_64 rng(config_.seed ? config_.seed : std::random_device()());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    const double eta = config_.step_size;

    std::vector<TreeNode> nodes;
    Eigen::Vector3d query = Eigen::Vector3d::Zero();
    ompl::NearestNeighborsGNAT<int64_t> nn;
    nn.setDistanceFunction([&nodes, &query](const int64_t& a, const int64_t& b)
    {
        const Eigen::Vector3d& pa = a < 0 ? query : nodes[a].position;
        const Eigen::Vector3d& pb = b < 0 ? query : nodes[b].position;
        return (pa - pb).norm();
    });
    auto near = [&](const Eigen::Vector3d& a_point, double a_radius)
    {
        query = a_point;
        std::vector<int64_t> result;
        nn.nearestR(-1, a_radius, result);
        return result;
    };

    PoRrtStarStats stats;
    auto addNode = [&](const Eigen::Vector3d& a_position, int64_t a_parent, double a_edge_length, double a_edge_log_survival,
                       bool a_goal) -> int64_t
    {
        TreeNode node;
        node.position = a_position;
        node.parent = a_parent;
        node.edge_length = a_edge_length;
        node.edge_log_survival = a_edge_log_survival;
        node.length = a_parent < 0 ? 0.0 : nodes[a_parent].length + a_edge_length;
        node.log_survival = a_parent < 0 ? 0.0 : nodes[a_parent].log_survival + a_edge_log_survival;
        node.p_fail = 1.0 - std::exp(node.log_survival);
        node.goal = a_goal;
        nodes.push_back(node);
        const int64_t index = static_cast<int64_t>(nodes.size()) - 1;
        if (a_parent >= 0)
            nodes[a_parent].children.push_back(index);
        nn.add(index);
        return index;
    };

    // PROPAGATE-COST: costs of the subtree of a node from the costs of its edges
    auto propagate = [&](int64_t a_node)
    {
        std::vector<int64_t> stack = {a_node};
        while (!stack.empty())
        {
            const int64_t n = stack.back();
            stack.pop_back();
            for (int64_t child : nodes[n].children)
            {
                TreeNode& c = nodes[child];
                c.length = nodes[n].length + c.edge_length;
                c.log_survival = nodes[n].log_survival + c.edge_log_survival;
                c.p_fail = 1.0 - std::exp(c.log_survival);
                stack.push_back(child);
            }
        }
    };

    // Connection radius r_n = min(gamma (log n / n)^(1/d), eta), d = 3, gamma of Karaman and Frazzoli on the sampling box
    const double dimension = 3.0;
    const Eigen::Vector3d extent = config_.max_bounds - config_.min_bounds;
    const double volume = extent.x() * extent.y() * extent.z();
    const double unit_ball = 4.0 / 3.0 * M_PI;
    const double gamma = config_.rewire_factor * std::pow(2.0 * (1.0 + 1.0 / dimension), 1.0 / dimension) *
                         std::pow(volume / unit_ball, 1.0 / dimension);
    auto radius = [&]()
    {
        const double n = static_cast<double>(nodes.size());
        return n < 2.0 ? eta : std::min(gamma * std::pow(std::log(n) / n, 1.0 / dimension), eta);
    };

    // REWIRE (Alg. 2, A.3)
    auto rewire = [&](int64_t a_new)
    {
        std::vector<Candidate> candidates;
        for (int64_t z : near(nodes[a_new].position, radius()))
        {
            if (z == a_new)
                continue;
            double length, log_survival;
            if ((nodes[z].position - nodes[a_new].position).norm() > eta ||
                !edgeCost(nodes[a_new].position, nodes[z].position, length, log_survival))
                continue;
            const double l = nodes[a_new].length + length;
            const double s = nodes[a_new].log_survival + log_survival;
            candidates.push_back({z, l, s, 1.0 - std::exp(s), length, log_survival});
        }
        for (const Candidate& c : paretoFilter(candidates))
        {
            TreeNode& z = nodes[c.node];
            if (dominates(c.length, c.p_fail, z.length, z.p_fail))
            {
                // Case A: new parent, the old path is discarded and the costs are propagated to the subtree
                auto& siblings = nodes[z.parent].children;
                siblings.erase(std::remove(siblings.begin(), siblings.end(), c.node), siblings.end());
                z.parent = a_new;
                z.edge_length = c.edge_length;
                z.edge_log_survival = c.edge_log_survival;
                z.length = c.length;
                z.log_survival = c.log_survival;
                z.p_fail = c.p_fail;
                nodes[a_new].children.push_back(c.node);
                propagate(c.node);
                ++stats.nb_of_parent_changes;
            }
            else if (!(z.length <= c.length && z.p_fail <= c.p_fail))
            {
                // Case B: alternative non-dominated branch, a new leaf at the position of z
                const Eigen::Vector3d position = z.position;
                const bool goal = z.goal;
                addNode(position, a_new, c.edge_length, c.edge_log_survival, goal);
                ++stats.nb_of_alternative_nodes;
            }
        }
    };

    // Alg. A.1
    addNode(a_start, -1, 0.0, 0.0, false);
    while (elapsed() < config_.time_budget && nodes.size() < config_.max_nodes)
    {
        ++stats.nb_of_iterations;
        Eigen::Vector3d x_rand;
        for (int i = 0; i < 3; ++i)
            x_rand[i] = config_.min_bounds[i] + uniform(rng) * extent[i];
        if (!valid_(x_rand))
            continue;

        query = x_rand;
        const int64_t nearest = nn.nearest(-1);
        Eigen::Vector3d x_new = x_rand;
        const Eigen::Vector3d direction = x_rand - nodes[nearest].position;
        if (direction.norm() > eta)
            x_new = nodes[nearest].position + direction.normalized() * eta;
        if (!valid_(x_new))
            continue;

        // CHOOSE-PARENTS (Alg. 1, A.2), the nearest node is always a candidate like in RRT*
        std::vector<int64_t> neighbors = near(x_new, radius());
        if (std::find(neighbors.begin(), neighbors.end(), nearest) == neighbors.end())
            neighbors.push_back(nearest);
        std::vector<Candidate> candidates;
        for (int64_t z : neighbors)
        {
            double length, log_survival;
            if ((nodes[z].position - x_new).norm() > eta || !edgeCost(nodes[z].position, x_new, length, log_survival))
                continue;
            const double l = nodes[z].length + length;
            const double s = nodes[z].log_survival + log_survival;
            candidates.push_back({z, l, s, 1.0 - std::exp(s), length, log_survival});
        }

        // One node at x_new per non-dominated parent
        std::vector<int64_t> new_nodes;
        for (const Candidate& c : paretoFilter(candidates))
            new_nodes.push_back(addNode(x_new, c.node, c.edge_length, c.edge_log_survival, false));

        for (int64_t n : new_nodes)
        {
            rewire(n);
            // A node closer to the goal than eta connects to it
            double length, log_survival;
            if ((a_goal - nodes[n].position).norm() <= eta && edgeCost(nodes[n].position, a_goal, length, log_survival))
            {
                addNode(a_goal, n, length, log_survival, true);
                ++stats.nb_of_goal_nodes;
            }
        }
    }

    // Global Pareto filter of the complete paths (Sec. 6.6), one path per cost pair
    std::vector<Candidate> goals;
    for (int64_t i = 0; i < static_cast<int64_t>(nodes.size()); ++i)
        if (nodes[i].goal)
            goals.push_back({i, nodes[i].length, nodes[i].log_survival, nodes[i].p_fail, 0.0, 0.0});
    std::vector<PoRrtStarPath> paths;
    for (const Candidate& c : paretoFilter(goals))
    {
        PoRrtStarPath path;
        path.length = c.length;
        path.log_survival = c.log_survival;
        path.failure_probability = c.p_fail;
        for (int64_t n = c.node; n >= 0; n = nodes[n].parent)
            path.waypoints.push_back(nodes[n].position);
        std::reverse(path.waypoints.begin(), path.waypoints.end());
        paths.push_back(std::move(path));
    }
    std::sort(paths.begin(), paths.end(), [](const PoRrtStarPath& a, const PoRrtStarPath& b) { return a.length < b.length; });

    stats.nb_of_nodes = nodes.size();
    if (a_stats)
        *a_stats = stats;
    return paths;
}

}; // namespace arena_benchmarks
