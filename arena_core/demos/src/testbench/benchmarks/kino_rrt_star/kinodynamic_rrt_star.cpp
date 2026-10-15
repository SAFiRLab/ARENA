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

#include "testbench/benchmarks/kino_rrt_star/kinodynamic_rrt_star.h"

// External Libraries
// Eigen
#include <Eigen/Eigenvalues>
// OMPL
#include <ompl/datastructures/NearestNeighborsGNAT.h>

// System
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>


namespace arena_benchmarks
{

namespace
{

struct TreeNode
{
    TrajectoryPoint state;
    int parent = -1;
    double cost = 0.0;       // cost[x], from the start
    double edge_cost = 0.0;  // c*[parent, x]
    double edge_tau = 0.0;   // tau*[parent, x]
    std::vector<int> children;
};

/** Updates the costs of the descendants of a node after a change of its cost. */
void propagateCost(std::vector<TreeNode>& a_nodes, int a_index)
{
    std::vector<int> stack = {a_index};
    while (!stack.empty())
    {
        const int index = stack.back();
        stack.pop_back();
        for (int child : a_nodes[index].children)
        {
            a_nodes[child].cost = a_nodes[index].cost + a_nodes[child].edge_cost;
            stack.push_back(child);
        }
    }
}

void setParent(std::vector<TreeNode>& a_nodes, int a_child, int a_parent, double a_cost, double a_tau)
{
    TreeNode& child = a_nodes[a_child];
    if (child.parent >= 0)
    {
        auto& siblings = a_nodes[child.parent].children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), a_child), siblings.end());
    }
    child.parent = a_parent;
    child.edge_cost = a_cost;
    child.edge_tau = a_tau;
    child.cost = a_nodes[a_parent].cost + a_cost;
    a_nodes[a_parent].children.push_back(a_child);
    propagateCost(a_nodes, a_child);
}

}; // namespace

bool KinodynamicRrtStar::optimalConnection(const TrajectoryPoint& a_from, const TrajectoryPoint& a_to, double& a_tau, double& a_cost) const
{
    // c[tau] = tau + r (12 |dp|^2 / tau^3 - 12 dp.(v0 + v1) / tau^2 + 4 (|v0|^2 + v0.v1 + |v1|^2) / tau)   (Eq. 11)
    const double r = config_.control_penalty;
    const Eigen::Vector3d dp = a_to.position - a_from.position;
    const Eigen::Vector3d& v0 = a_from.velocity;
    const Eigen::Vector3d& v1 = a_to.velocity;
    const double p2 = dp.squaredNorm();
    const double ps = dp.dot(v0 + v1);
    const double q = v0.squaredNorm() + v0.dot(v1) + v1.squaredNorm();
    if (p2 < 1.0e-18 && q < 1.0e-18)
        return false; // Same state

    auto cost = [&](double tau) { return tau + r * (12.0 * p2 / (tau * tau * tau) - 12.0 * ps / (tau * tau) + 4.0 * q / tau); };

    // dc/dtau = 0 (Eq. 13) <=> tau^4 - 4 r q tau^2 + 24 r ps tau - 36 r p2 = 0: roots of the companion matrix
    Eigen::Matrix4d companion = Eigen::Matrix4d::Zero();
    companion(1, 0) = companion(2, 1) = companion(3, 2) = 1.0;
    companion(0, 3) = 36.0 * r * p2;
    companion(1, 3) = -24.0 * r * ps;
    companion(2, 3) = 4.0 * r * q;
    companion(3, 3) = 0.0;
    Eigen::EigenSolver<Eigen::Matrix4d> solver(companion, false);

    a_cost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 4; ++i)
    {
        const std::complex<double> root = solver.eigenvalues()[i];
        if (root.real() <= 1.0e-9 || std::abs(root.imag()) > 1.0e-6 * std::max(1.0, std::abs(root.real())))
            continue;
        const double c = cost(root.real());
        if (c < a_cost)
        {
            a_cost = c;
            a_tau = root.real();
        }
    }
    return std::isfinite(a_cost);
}

TrajectoryPoint KinodynamicRrtStar::connectionState(const TrajectoryPoint& a_from, const TrajectoryPoint& a_to, double a_tau,
                                                    double a_t, Eigen::Vector3d* a_control)
{
    // Optimal trajectory (Eq. 20) of the double integrator: cubic polynomial per axis
    const Eigen::Vector3d dp = a_to.position - a_from.position - a_from.velocity * a_tau;
    const Eigen::Vector3d dv = a_to.velocity - a_from.velocity;
    const Eigen::Vector3d a = -2.0 * dp / (a_tau * a_tau * a_tau) + dv / (a_tau * a_tau);
    const Eigen::Vector3d b = 3.0 * dp / (a_tau * a_tau) - dv / a_tau;
    TrajectoryPoint state;
    state.position = a_from.position + a_from.velocity * a_t + b * a_t * a_t + a * a_t * a_t * a_t;
    state.velocity = a_from.velocity + 2.0 * b * a_t + 3.0 * a * a_t * a_t;
    state.time = a_from.time + a_t;
    if (a_control)
        *a_control = 2.0 * b + 6.0 * a * a_t;
    return state;
}

bool KinodynamicRrtStar::isStateFree(const TrajectoryPoint& a_state) const
{
    for (int i = 0; i < 3; ++i)
        if (a_state.position[i] < config_.min_bounds[i] || a_state.position[i] > config_.max_bounds[i])
            return false;
    return a_state.velocity.norm() <= config_.max_speed * (1.0 + 1.0e-9) && validity_(a_state.position);
}

bool KinodynamicRrtStar::isConnectionFree(const TrajectoryPoint& a_from, const TrajectoryPoint& a_to, double a_tau) const
{
    // The control is linear in time, its norm is maximal at an end of the connection
    Eigen::Vector3d u0, u1;
    connectionState(a_from, a_to, a_tau, 0.0, &u0);
    connectionState(a_from, a_to, a_tau, a_tau, &u1);
    const double max_control = config_.max_acceleration * (1.0 + 1.0e-9);
    if (u0.norm() > max_control || u1.norm() > max_control)
        return false;

    const int nb_of_steps = std::max(1, static_cast<int>(std::ceil(a_tau / config_.check_step)));
    for (int k = 1; k <= nb_of_steps; ++k)
        if (!isStateFree(connectionState(a_from, a_to, a_tau, a_tau * k / nb_of_steps)))
            return false;
    return true;
}

bool KinodynamicRrtStar::plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, NativeTrajectory& a_trajectory,
                              double a_sampling_step, KinodynamicRrtStarStats* a_stats) const
{
    const auto start_time = std::chrono::steady_clock::now();
    auto elapsed = [&start_time]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };
    std::mt19937_64 rng(config_.seed ? config_.seed : std::random_device()());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> normal(0.0, 1.0);
    const bool infinite_radius = config_.neighbor_radius <= 0.0;
    const double radius = infinite_radius ? std::numeric_limits<double>::infinity() : config_.neighbor_radius;

    KinodynamicRrtStarStats stats;
    std::vector<TreeNode> nodes(2);
    nodes[0].state.position = a_start;                     // Root: start state, at rest
    nodes[1].state.position = a_goal;                      // Goal state, at rest, not part of the tree T
    nodes[1].cost = std::numeric_limits<double>::infinity();
    const int kGoal = 1;

    ompl::NearestNeighborsGNAT<int> nn;
    nn.setDistanceFunction([&nodes](const int a, const int b) { return (nodes[a].state.position - nodes[b].state.position).norm(); });
    nn.add(0);
    std::vector<int> all_nodes = {0};

    TreeNode query;
    nodes.push_back(query); // Query slot for the nearest neighbors, index 2
    const int kQuery = 2;

    struct Candidate { int index; double cost; double tau; double total; };
    std::vector<int> near;
    std::vector<Candidate> candidates;
    double best_goal_cost = std::numeric_limits<double>::infinity();

    while (elapsed() < config_.time_budget && nodes.size() < config_.max_nodes)
    {
        stats.nb_of_iterations++;

        // Line 3: random state of X_free (valid position in the bounds, velocity in the ball ||v|| <= max_speed)
        TrajectoryPoint sample;
        for (int i = 0; i < 3; ++i)
            sample.position[i] = config_.min_bounds[i] + uniform(rng) * (config_.max_bounds[i] - config_.min_bounds[i]);
        Eigen::Vector3d direction(normal(rng), normal(rng), normal(rng));
        sample.velocity = direction.normalized() * config_.max_speed * std::cbrt(uniform(rng));
        if (!isStateFree(sample))
            continue;

        // Neighbors: c*[x, x_i] < r implies tau* < r and a distance below max_speed * tau* for a connection in X_free
        nodes[kQuery].state = sample;
        if (infinite_radius)
            near = all_nodes;
        else
            nn.nearestR(kQuery, config_.max_speed * radius, near);

        // Lines 4-6: parent minimizing cost[x] + c*[x, x_i] among the collision-free connections
        candidates.clear();
        for (int index : near)
        {
            double tau, cost;
            if (optimalConnection(nodes[index].state, sample, tau, cost) && cost < radius)
                candidates.push_back({index, cost, tau, nodes[index].cost + cost});
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.total < b.total; });
        int parent = -1;
        double parent_cost = 0.0, parent_tau = 0.0;
        for (const Candidate& candidate : candidates)
        {
            if (isConnectionFree(nodes[candidate.index].state, sample, candidate.tau))
            {
                parent = candidate.index;
                parent_cost = candidate.cost;
                parent_tau = candidate.tau;
                break;
            }
        }
        if (parent < 0)
            continue;

        const int new_index = static_cast<int>(nodes.size());
        TreeNode node;
        node.state = sample;
        nodes.push_back(node);
        setParent(nodes, new_index, parent, parent_cost, parent_tau);

        // Lines 7-9: rewiring of the neighbors and of the goal state through x_i
        near.push_back(kGoal);
        for (int index : near)
        {
            if (index == parent)
                continue;
            double tau, cost;
            if (!optimalConnection(sample, nodes[index].state, tau, cost) || !(cost < radius) ||
                !(nodes[new_index].cost + cost < nodes[index].cost))
                continue;
            if (isConnectionFree(sample, nodes[index].state, tau))
                setParent(nodes, index, new_index, cost, tau);
        }

        // Line 10
        nn.add(new_index);
        all_nodes.push_back(new_index);

        if (nodes[kGoal].cost < best_goal_cost)
        {
            best_goal_cost = nodes[kGoal].cost;
            stats.nb_of_goal_improvements++;
            if (stats.first_solution_time < 0.0)
                stats.first_solution_time = elapsed();
        }
    }

    stats.nb_of_nodes = nodes.size() - 2;
    stats.time = elapsed();
    stats.cost = nodes[kGoal].cost;
    if (a_stats)
        *a_stats = stats;
    if (nodes[kGoal].parent < 0)
        return false;

    // Trajectory: optimal connections from the start to the goal, sampled every a_sampling_step
    std::vector<int> chain;
    for (int index = kGoal; index >= 0; index = nodes[index].parent)
        chain.push_back(index);
    std::reverse(chain.begin(), chain.end());

    a_trajectory.clear();
    TrajectoryPoint point = nodes[chain.front()].state;
    point.time = 0.0;
    a_trajectory.push_back(point);
    for (size_t i = 1; i < chain.size(); ++i)
    {
        TrajectoryPoint from = nodes[chain[i - 1]].state;
        from.time = a_trajectory.back().time;
        const TrajectoryPoint& to = nodes[chain[i]].state;
        const double tau = nodes[chain[i]].edge_tau;
        const int nb_of_steps = std::max(1, static_cast<int>(std::ceil(tau / a_sampling_step - 1.0e-9)));
        for (int k = 1; k <= nb_of_steps; ++k)
            a_trajectory.push_back(connectionState(from, to, tau, tau * k / nb_of_steps));
    }
    return true;
}

}; // namespace arena_benchmarks
