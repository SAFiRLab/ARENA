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
#include <cstdint>
#include <functional>
#include <set>
#include <utility>
#include <vector>


namespace arena_benchmarks
{

/** @brief Cost vector of the three objectives of TOA*. */
using ToaCost = std::array<double, 3>;


/**
 * @brief Non-dominated set of 2D vectors (the projected cost vectors p(g) of TOA*), ordered lexicographically.
 *
 * In a 2D non-dominated set sorted by the first component, the second component decreases: the dominance check of a vector
 * b only compares b with the vector of largest first component not above b's (O(log n), Theorem 1 of the TOA* paper), and
 * the vectors dominated by b are the contiguous run that follows b (filter of Alg. 3).
 */
class ToaFrontier
{
public:

    using Vector2 = std::pair<double, double>;

    /** @brief True if a vector of the set is component-wise no larger than (1 + a_epsilon) * a_b (dominates or equals). */
    bool covers(const Vector2& a_b, double a_epsilon = 0.0) const;

    /** @brief Removes the vectors dominated by a_b and inserts a_b (a_b must not be covered by the set). */
    void insert(const Vector2& a_b);

    size_t size() const { return set_.size(); }

private:
    std::set<Vector2> set_;

}; // class ToaFrontier


struct ToaStarConfig
{
    double max_time = 600.0;           // s, the search stops after this time (the solutions found are kept)
    size_t max_labels = 200000000;     // The search stops after generating this number of labels
    double epsilon = 0.0;              // epsilon-dominance of the SolutionCheck (Perny and Spanjaard 2008), 0: exact front
}; // struct ToaStarConfig


struct ToaStarStats
{
    size_t nb_of_expansions = 0;
    size_t nb_of_labels = 0;
    bool complete = true;  // false if the search stopped on max_time or max_labels before OPEN was empty
    double time = 0.0;     // s
}; // struct ToaStarStats


/**
 * @brief TOA* (Tri-Objective A*) of Ren, Zhan, Rathinam, Likhachev and Choset, "Enhanced Multi-Objective A* Using Balanced
 * Binary Search Trees" (arXiv:2202.08992, 2022): the EMOA* search framework (Alg. 1) for three objectives.
 *
 * Labels l = (v, g) are popped from OPEN in the lexicographic order of f = g + h (consistent heuristic). With lazy checks, a
 * label is discarded when popped or generated if its projected cost vector p(g) (the cost vector without its first
 * component, dimensionality reduction) is dominated by or equal to a vector of the frontier set of its vertex
 * (FrontierCheck), or if p(f) is dominated by or equal to a vector of the solution set (SolutionCheck). A popped label that
 * isn't discarded updates the frontier set of its vertex (UpdateFrontier), and is a solution if its vertex is the
 * destination, otherwise it is expanded. At the end (OPEN empty), the solutions are all the cost-unique Pareto-optimal
 * paths. The frontier sets are balanced binary search trees of the projected vectors (ToaFrontier).
 *
 * With epsilon > 0, the SolutionCheck uses the epsilon-dominance of Perny and Spanjaard (2008): a label is discarded if a
 * solution is no larger than (1 + epsilon) times its f vector. Every Pareto-optimal cost is then covered by (1 + epsilon)
 * times a solution (an epsilon-approximation of the Pareto front). The frontier checks stay exact.
 */
class ToaStarSearch
{
public:

    /** @brief Successors of a vertex with the cost vectors of the edges (non-negative). */
    using SuccessorFn = std::function<void(int64_t, std::vector<std::pair<int64_t, ToaCost>>&)>;
    /** @brief Consistent heuristic of a vertex, infinite components if the destination can't be reached. */
    using HeuristicFn = std::function<ToaCost(int64_t)>;

    struct Solution
    {
        ToaCost cost;
        std::vector<int64_t> vertices; // From the start to the destination
    };

    explicit ToaStarSearch(const ToaStarConfig& a_config = ToaStarConfig()) : config_(a_config) {}

    /** @brief Pareto-optimal paths from a_start to a_destination, in the lexicographic order of their costs. */
    std::vector<Solution> search(int64_t a_start, int64_t a_destination, const SuccessorFn& a_successors,
                                 const HeuristicFn& a_heuristic, ToaStarStats* a_stats = nullptr) const;

private:
    ToaStarConfig config_;

}; // class ToaStarSearch

}; // namespace arena_benchmarks
