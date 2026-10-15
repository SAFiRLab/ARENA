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

#include "testbench/benchmarks/toa_star/toa_star_search.h"

// System
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>


namespace arena_benchmarks
{

/****************************************************** ToaFrontier ****************************************************/

bool ToaFrontier::covers(const Vector2& a_b, double a_epsilon) const
{
    const Vector2 b(a_b.first * (1.0 + a_epsilon), a_b.second * (1.0 + a_epsilon));

    // Vector of largest first component not above b's: it has the smallest second component of these vectors
    auto it = set_.upper_bound(Vector2(b.first, std::numeric_limits<double>::infinity()));
    if (it == set_.begin())
        return false;
    --it;
    return it->second <= b.second;
}

void ToaFrontier::insert(const Vector2& a_b)
{
    // The vectors dominated by b have a first component >= b's and a second one >= b's: the run that follows b
    auto it = set_.lower_bound(Vector2(a_b.first, -std::numeric_limits<double>::infinity()));
    while (it != set_.end() && it->second >= a_b.second)
        it = set_.erase(it);
    set_.insert(a_b);
}


/***************************************************** ToaStarSearch ***************************************************/

namespace
{

struct Label
{
    int64_t vertex;
    ToaCost g;
    int64_t parent; // Index of the parent label, -1 for the start
};

ToaFrontier::Vector2 projection(const ToaCost& a_cost)
{
    return {a_cost[1], a_cost[2]};
}

}; // namespace

std::vector<ToaStarSearch::Solution> ToaStarSearch::search(int64_t a_start, int64_t a_destination, const SuccessorFn& a_successors,
                                                            const HeuristicFn& a_heuristic, ToaStarStats* a_stats) const
{
    const auto start_time = std::chrono::steady_clock::now();
    auto elapsed = [&start_time]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };
    const double epsilon = config_.epsilon;

    std::vector<Label> labels;
    std::unordered_map<int64_t, ToaFrontier> frontiers; // alpha(v)
    ToaFrontier solution_frontier;                     // S = alpha(destination)
    std::vector<int64_t> solution_labels;

    // OPEN: lexicographic order of f
    using Entry = std::pair<ToaCost, int64_t>; // f, label
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    // The frontier checks are exact, only the solution check uses the epsilon-dominance: a discarded label has a lower
    // bound f covered by (1 + epsilon) times a solution, every Pareto-optimal cost is then covered by (1 + epsilon) times
    // a solution
    auto frontierCovers = [&](int64_t a_vertex, const ToaFrontier::Vector2& a_b)
    {
        if (a_vertex == a_destination)
            return solution_frontier.covers(a_b);
        auto it = frontiers.find(a_vertex);
        return it != frontiers.end() && it->second.covers(a_b);
    };

    // Alg. 1, lines 1-3
    const ToaCost h_start = a_heuristic(a_start);
    labels.push_back({a_start, {0.0, 0.0, 0.0}, -1});
    open.emplace(h_start, 0);

    ToaStarStats stats;
    std::vector<std::pair<int64_t, ToaCost>> successors;
    size_t iteration = 0;
    while (!open.empty())
    {
        // Limits of the search (not in the paper): the solutions found so far are kept
        if ((++iteration & 1023) == 0 && elapsed() > config_.max_time)
        {
            stats.complete = false;
            break;
        }
        if (labels.size() > config_.max_labels)
        {
            stats.complete = false;
            break;
        }

        // Lines 5-7: pop the lexicographically smallest label, FrontierCheck and SolutionCheck
        const auto [f, label_index] = open.top();
        open.pop();
        const Label label = labels[label_index];
        if (frontierCovers(label.vertex, projection(label.g)) || solution_frontier.covers(projection(f), epsilon))
            continue;

        // Line 8: UpdateFrontier
        if (label.vertex == a_destination)
        {
            // Lines 9-10: a solution
            solution_frontier.insert(projection(label.g));
            solution_labels.push_back(label_index);
            continue;
        }
        frontiers[label.vertex].insert(projection(label.g));

        // Lines 11-16: expansion
        ++stats.nb_of_expansions;
        a_successors(label.vertex, successors);
        for (const auto& [vertex, cost] : successors)
        {
            const ToaCost g = {label.g[0] + cost[0], label.g[1] + cost[1], label.g[2] + cost[2]};
            const ToaCost h = a_heuristic(vertex);
            if (!std::isfinite(h[0]) || !std::isfinite(h[1]) || !std::isfinite(h[2]))
                continue; // The destination can't be reached from this vertex
            const ToaCost f_next = {g[0] + h[0], g[1] + h[1], g[2] + h[2]};
            if (frontierCovers(vertex, projection(g)) || solution_frontier.covers(projection(f_next), epsilon))
                continue;
            labels.push_back({vertex, g, label_index});
            open.emplace(f_next, static_cast<int64_t>(labels.size()) - 1);
        }
    }

    // Paths of the solutions
    std::vector<Solution> solutions;
    for (int64_t index : solution_labels)
    {
        Solution solution;
        solution.cost = labels[index].g;
        for (int64_t l = index; l >= 0; l = labels[l].parent)
            solution.vertices.push_back(labels[l].vertex);
        std::reverse(solution.vertices.begin(), solution.vertices.end());
        solutions.push_back(std::move(solution));
    }

    stats.nb_of_labels = labels.size();
    stats.time = elapsed();
    if (a_stats)
        *a_stats = stats;
    return solutions;
}

}; // namespace arena_benchmarks
