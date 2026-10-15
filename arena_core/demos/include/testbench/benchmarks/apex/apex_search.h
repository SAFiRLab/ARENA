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

// Local
#include "testbench/benchmarks/toa_star/toa_star_search.h"

// System
#include <cstdint>
#include <functional>
#include <vector>


namespace arena_benchmarks
{

struct ApexSearchConfig
{
    ToaCost epsilon = {0.01, 0.01, 0.01}; // Approximation factor of every objective
    double max_time = 600.0;              // s, the search stops after this time (the solutions found are kept)
    size_t max_nodes = 100000000;         // The search stops after generating this number of apex-path pairs
}; // struct ApexSearchConfig


struct ApexSearchStats
{
    size_t nb_of_expansions = 0;
    size_t nb_of_apex_path_pairs = 0;
    size_t nb_of_merges = 0;
    bool complete = true;  // false if the search stopped on max_time or max_nodes before Open was empty
    double time = 0.0;     // s
}; // struct ApexSearchStats


/**
 * @brief A*pex of Zhang, Salzman, Kumar, Felner, Hernandez Ulloa and Koenig, "A*pex: Efficient Approximate Multi-Objective
 * Search on Graphs" (ICAPS 2022), Algorithm 2 with the Greedy merge method (A*pex-G).
 *
 * Every node of Open is an epsilon-bounded apex-path pair AP = <A, pi>: the apex A is the component-wise minimum of the
 * g-values of the paths it represents, pi a representative path, epsilon-bounded iff f(pi) <=_eps f(AP) (f(AP) = A + h).
 * Apex-path pairs are extracted in the lexicographic order of f(AP) and discarded (is_dominated) if the truncated f-value
 * (without the first component) of the representative path of a solution epsilon-dominates the truncated f(AP) (the apex of
 * this solution is then updated to the component-wise minimum of f(AP) and its apex), or if a truncated f-value of an
 * expanded apex-path pair of the same state weakly dominates the truncated f(AP). A generated apex-path pair (resp. a
 * solution) is merged with one of Open (resp. of the solution set) of the same state when the merge is epsilon-bounded:
 * apex = component-wise minimum, representative path = the candidate with the larger slack
 * min_i (1 + eps_i - f_i(pi) / f_i(AP)) / eps_i. At the end, the representative paths of the solution set form an
 * epsilon-approximate Pareto-optimal solution set (Theorem 1): every Pareto-optimal path is epsilon-dominated by one of them.
 */
class ApexSearch
{
public:

    using SuccessorFn = ToaStarSearch::SuccessorFn;
    using HeuristicFn = ToaStarSearch::HeuristicFn;
    using Solution = ToaStarSearch::Solution;

    explicit ApexSearch(const ApexSearchConfig& a_config = ApexSearchConfig()) : config_(a_config) {}

    /** @brief epsilon-approximate Pareto-optimal paths from a_start to a_destination (representative paths). */
    std::vector<Solution> search(int64_t a_start, int64_t a_destination, const SuccessorFn& a_successors,
                                 const HeuristicFn& a_heuristic, ApexSearchStats* a_stats = nullptr) const;

private:
    ApexSearchConfig config_;

}; // class ApexSearch

}; // namespace arena_benchmarks
