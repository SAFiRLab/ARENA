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

#include "testbench/benchmarks/apex/apex_search.h"

// System
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>


namespace arena_benchmarks
{

namespace
{

// Node of the tree of the representative paths: a path is its last node, extended paths share their prefix
struct PathNode
{
    int64_t vertex;
    int64_t parent; // -1 for the start
    ToaCost g;
};

// Apex-path pair
struct ApexPathPair
{
    int64_t vertex;
    ToaCost apex;   // A
    ToaCost h;      // h(vertex)
    int64_t path;   // Representative path (PathNode)
    bool alive = true;
};

ToaCost add(const ToaCost& a, const ToaCost& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
ToaCost min(const ToaCost& a, const ToaCost& b) { return {std::min(a[0], b[0]), std::min(a[1], b[1]), std::min(a[2], b[2])}; }

}; // namespace

std::vector<ApexSearch::Solution> ApexSearch::search(int64_t a_start, int64_t a_destination, const SuccessorFn& a_successors,
                                                      const HeuristicFn& a_heuristic, ApexSearchStats* a_stats) const
{
    const auto start_time = std::chrono::steady_clock::now();
    auto elapsed = [&start_time]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };
    const ToaCost& eps = config_.epsilon;

    std::vector<PathNode> paths;
    std::vector<ApexPathPair> pairs;
    std::unordered_map<int64_t, std::vector<int64_t>> open_at;  // Apex-path pairs of Open of every state
    std::unordered_map<int64_t, ToaFrontier> closed;             // G_cl^T(s): truncated f-values of the expanded pairs
    std::vector<int64_t> solutions;                              // Apex-path pairs of the solution set
    ApexSearchStats stats;

    using Entry = std::pair<ToaCost, int64_t>; // f(AP), pair
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    auto fApex = [&](const ApexPathPair& ap) { return add(ap.apex, ap.h); };
    auto fPath = [&](const ApexPathPair& ap) { return add(paths[ap.path].g, ap.h); };

    // f(pi) <=_eps f(AP) for the components [a_first, 3)
    auto epsDominates = [&eps](const ToaCost& f_path, const ToaCost& f_apex, int a_first)
    {
        for (int i = a_first; i < 3; ++i)
            if (f_path[i] > (1.0 + eps[i]) * f_apex[i] + 1e-12 * std::abs(f_apex[i]))
                return false;
        return true;
    };

    // Slack of a representative path in a merged pair (Greedy method)
    auto slack = [&eps](const ToaCost& f_path, const ToaCost& f_apex)
    {
        double s = std::numeric_limits<double>::infinity();
        for (int i = 0; i < 3; ++i)
        {
            const double ratio = f_apex[i] > 0.0 ? f_path[i] / f_apex[i] : 1.0;
            s = std::min(s, eps[i] > 0.0 ? (1.0 + eps[i] - ratio) / eps[i] : (ratio <= 1.0 ? 1.0 : -1.0));
        }
        return s;
    };

    // merge: epsilon-bounded merged pair of two pairs of the same state, if any
    auto merge = [&](const ApexPathPair& a, const ApexPathPair& b) -> std::optional<ApexPathPair>
    {
        ApexPathPair merged = a;
        merged.apex = min(a.apex, b.apex);
        const ToaCost f_merged = fApex(merged);
        const bool a_candidate = epsDominates(fPath(a), f_merged, 0);
        const bool b_candidate = epsDominates(fPath(b), f_merged, 0);
        if (!a_candidate && !b_candidate)
            return std::nullopt;
        if (a_candidate && b_candidate)
            merged.path = slack(fPath(a), f_merged) >= slack(fPath(b), f_merged) ? a.path : b.path;
        else
            merged.path = a_candidate ? a.path : b.path;
        return merged;
    };

    // is_dominated (lines 19-26)
    auto isDominated = [&](const ApexPathPair& ap)
    {
        const ToaCost f = fApex(ap);
        for (int64_t s : solutions)
        {
            ApexPathPair& solution = pairs[s];
            if (epsDominates(fPath(solution), f, 1))
            {
                solution.apex = min(f, solution.apex);
                return true;
            }
        }
        auto it = closed.find(ap.vertex);
        return it != closed.end() && it->second.covers({f[1], f[2]});
    };

    // insert (lines 27-35) into Open or into the solution set
    auto insertOpen = [&](ApexPathPair ap)
    {
        auto& at = open_at[ap.vertex];
        for (size_t i = 0; i < at.size(); ++i)
        {
            if (auto merged = merge(ap, pairs[at[i]]))
            {
                pairs[at[i]].alive = false;
                pairs.push_back(*merged);
                at[i] = static_cast<int64_t>(pairs.size()) - 1;
                open.emplace(fApex(pairs.back()), at[i]);
                ++stats.nb_of_merges;
                return;
            }
        }
        pairs.push_back(ap);
        at.push_back(static_cast<int64_t>(pairs.size()) - 1);
        open.emplace(fApex(ap), at.back());
    };
    auto insertSolution = [&](const ApexPathPair& ap)
    {
        for (int64_t& s : solutions)
        {
            if (auto merged = merge(ap, pairs[s]))
            {
                pairs.push_back(*merged);
                s = static_cast<int64_t>(pairs.size()) - 1;
                ++stats.nb_of_merges;
                return;
            }
        }
        pairs.push_back(ap);
        solutions.push_back(static_cast<int64_t>(pairs.size()) - 1);
    };

    // Line 1
    paths.push_back({a_start, -1, {0.0, 0.0, 0.0}});
    insertOpen({a_start, {0.0, 0.0, 0.0}, a_heuristic(a_start), 0, true});

    std::vector<std::pair<int64_t, ToaCost>> successors;
    size_t iteration = 0;
    while (!open.empty())
    {
        if ((++iteration & 1023) == 0 && elapsed() > config_.max_time)
        {
            stats.complete = false;
            break;
        }
        if (pairs.size() > config_.max_nodes)
        {
            stats.complete = false;
            break;
        }

        // Line 6: extract the pair with the lexicographically smallest f-value (merged pairs are no longer alive)
        const int64_t index = open.top().second;
        open.pop();
        if (!pairs[index].alive)
            continue;
        pairs[index].alive = false;
        const ApexPathPair ap = pairs[index];
        auto& at = open_at[ap.vertex];
        at.erase(std::find(at.begin(), at.end(), index));

        // Lines 7-9
        if (isDominated(ap))
            continue;
        const ToaCost f = fApex(ap);
        closed[ap.vertex].insert({f[1], f[2]});

        // Lines 10-12
        if (ap.vertex == a_destination)
        {
            insertSolution(ap);
            continue;
        }

        // Lines 13-17
        ++stats.nb_of_expansions;
        a_successors(ap.vertex, successors);
        for (const auto& [vertex, cost] : successors)
        {
            const ToaCost h = a_heuristic(vertex);
            if (!std::isfinite(h[0]) || !std::isfinite(h[1]) || !std::isfinite(h[2]))
                continue; // The destination can't be reached from this state
            ApexPathPair child{vertex, add(ap.apex, cost), h, -1, true};
            if (isDominated(child))
                continue;
            paths.push_back({vertex, ap.path, add(paths[ap.path].g, cost)});
            child.path = static_cast<int64_t>(paths.size()) - 1;
            insertOpen(child);
        }
    }

    // Line 18: representative paths of the solution set
    std::vector<Solution> result;
    for (int64_t s : solutions)
    {
        Solution solution;
        solution.cost = paths[pairs[s].path].g;
        for (int64_t p = pairs[s].path; p >= 0; p = paths[p].parent)
            solution.vertices.push_back(paths[p].vertex);
        std::reverse(solution.vertices.begin(), solution.vertices.end());
        result.push_back(std::move(solution));
    }
    std::sort(result.begin(), result.end(), [](const Solution& a, const Solution& b) { return a.cost < b.cost; });

    stats.nb_of_apex_path_pairs = pairs.size();
    stats.time = elapsed();
    if (a_stats)
        *a_stats = stats;
    return result;
}

}; // namespace arena_benchmarks
