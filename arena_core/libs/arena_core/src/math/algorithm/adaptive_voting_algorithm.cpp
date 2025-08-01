/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2025, David-Alexandre Poissant, Université de Sherbrooke
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

#include "arena_core/math/algorithm/adaptive_voting_algorithm.h"

// System
#include <algorithm>
#include <numeric>
#include <cassert>
#include <limits>
#include <vector>
#include <stdexcept>


namespace arena_core
{

namespace adaptive_voting_algorithm
{

double getLowestCost(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index)
{
    if (a_population_fitness_values.empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getLowestCost() -> Population cannot be empty.");
    
    if (a_population_fitness_values[0].empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getLowestCost() -> Fitness values cannot be empty.");

    if (a_fitness_index < 0 || a_fitness_index >= a_population_fitness_values[0].size())
        throw std::out_of_range("arena_core::adaptive_voting_algorithm::getLowestCost() -> Fitness index is out of bounds for the fitness values provided.");

    const auto expected_size = a_population_fitness_values[0].size();
    
    auto worst_cost = std::max_element(a_population_fitness_values.begin(), a_population_fitness_values.end(),
        [a_fitness_index, expected_size](const std::vector<double>& a, const std::vector<double>& b)
        {
            if (a.size() != expected_size || b.size() != expected_size)
                throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getLowestCost() -> All candidates must have the same number of fitness values.");
            return a[a_fitness_index] < b[a_fitness_index];
        });

    return (*worst_cost)[a_fitness_index];
}

double getHighestCost(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index)
{
    if (a_population_fitness_values.empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getHighestCost() -> Population cannot be empty.");
    
    if (a_population_fitness_values[0].empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getHighestCost() -> Fitness values cannot be empty.");
    
    if (a_fitness_index < 0 || a_fitness_index >= a_population_fitness_values[0].size())
        throw std::out_of_range("arena_core::adaptive_voting_algorithm::getHighestCost() -> Fitness index is out of bounds for the fitness values provided.");

    const auto expected_size = a_population_fitness_values[0].size();
    
    auto best_cost = std::min_element(a_population_fitness_values.begin(), a_population_fitness_values.end(),
        [a_fitness_index, expected_size](const std::vector<double>& a, const std::vector<double>& b)
        {
            if (a.size() != expected_size || b.size() != expected_size)
                throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getHighestCost() -> All candidates must have the same number of fitness values.");
            return a[a_fitness_index] < b[a_fitness_index];
        });

    return (*best_cost)[a_fitness_index];
}

std::vector<double> getCostsFromLowest(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index)
{
    if (a_population_fitness_values.empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getCostsFromLowest() -> Population cannot be empty.");

    if (a_population_fitness_values[0].empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getCostsFromLowest() -> Fitness values cannot be empty.");

    if (a_fitness_index < 0 || a_fitness_index >= a_population_fitness_values[0].size())
        throw std::out_of_range("arena_core::adaptive_voting_algorithm::getCostsFromLowest() -> Fitness index is out of bounds for the fitness values provided.");

    const auto expected_size = a_population_fitness_values[0].size();
    
    auto worst_cost = std::max_element(a_population_fitness_values.begin(), a_population_fitness_values.end(),
        [a_fitness_index, expected_size](const std::vector<double>& a, const std::vector<double>& b)
        {
            if (a.size() != expected_size || b.size() != expected_size)
                throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getCostsFromLowest() -> All candidates must have the same number of fitness values.");
            return a[a_fitness_index] < b[a_fitness_index];
        });
    
    std::vector<double> worst_costs;
    for (const auto& fitness_value : *worst_cost)
        worst_costs.push_back(fitness_value);

    return worst_costs;
}

std::vector<double> getCostsFromHighest(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index)
{
    if (a_population_fitness_values.empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getCostsFromHighest() -> Population cannot be empty.");

    if (a_population_fitness_values[0].empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getCostsFromHighest() -> Fitness values cannot be empty.");

    if (a_fitness_index < 0 || a_fitness_index >= a_population_fitness_values[0].size())
        throw std::out_of_range("arena_core::adaptive_voting_algorithm::getCostsFromHighest() -> Fitness index is out of bounds for the fitness values provided.");

    const auto expected_size = a_population_fitness_values[0].size();
    
    auto best_cost = std::min_element(a_population_fitness_values.begin(), a_population_fitness_values.end(),
        [a_fitness_index, expected_size](const std::vector<double>& a, const std::vector<double>& b)
        {
            if (a.size() != expected_size || b.size() != expected_size)
                throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getCostsFromHighest() -> All candidates must have the same number of fitness values.");
            return a[a_fitness_index] < b[a_fitness_index];
        });
    
    std::vector<double> best_costs;
    for (const auto& fitness_value : *best_cost)
        best_costs.push_back(fitness_value);

    return best_costs;
}

std::vector<double> getFitnessRanking(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index)
{
    if (a_population_fitness_values.empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getFitnessRanking() -> Population cannot be empty.");

    if (a_population_fitness_values[0].empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getFitnessRanking() -> Fitness values cannot be empty.");

    if (a_fitness_index < 0 || a_fitness_index >= a_population_fitness_values[0].size())
        throw std::out_of_range("arena_core::adaptive_voting_algorithm::getFitnessRanking() -> Fitness index is out of bounds for the fitness values provided.");

    const auto expected_size = a_population_fitness_values[0].size();
    
    std::vector<double> candidate_score_ranks(a_population_fitness_values.size(), std::numeric_limits<double>::max());

    for (size_t i = 0; i < a_population_fitness_values.size(); ++i)
    {
        int rank = 0;
        for (size_t j = 0; j < a_population_fitness_values.size(); ++j)
        {
            if (a_population_fitness_values[i].size() != expected_size || a_population_fitness_values[j].size() != expected_size)
                throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getFitnessRanking() -> All candidates must have the same number of fitness values.");
            
            if (i != j && a_population_fitness_values[i][a_fitness_index] > a_population_fitness_values[j][a_fitness_index])
                ++rank;
        }
        candidate_score_ranks[i] = rank;
    }

    return candidate_score_ranks;
}

int getBetterCandidateIndex(const std::vector<std::vector<double>>& a_population_fitness_values, const std::vector<double>& a_fitness_coefficients)
{
    if (a_population_fitness_values.empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getBetterCandidateIndex() -> Population cannot be empty.");

    if (a_population_fitness_values[0].empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getBetterCandidateIndex() -> Fitness values cannot be empty.");

    if (a_fitness_coefficients.empty())
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getBetterCandidateIndex() -> Fitness coefficients cannot be empty.");

    const auto expected_size = a_population_fitness_values[0].size();

    if (a_fitness_coefficients.size() != expected_size)
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getBetterCandidateIndex() -> Fitness coefficients size must match the number of fitness values.");

    // Check if all coefficients are positive and at least one is non-zero
    if (std::any_of(a_fitness_coefficients.begin(), a_fitness_coefficients.end(), [](double coeff) { return coeff < 0; }))
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getBetterCandidateIndex() -> All fitness coefficients must be non-negative.");

    if (std::all_of(a_fitness_coefficients.begin(), a_fitness_coefficients.end(), [](double coeff) { return coeff == 0; }))
        throw std::invalid_argument("arena_core::adaptive_voting_algorithm::getBetterCandidateIndex() -> At least one fitness coefficient must be non-zero.");

    // Calculate ranks for each fitness
    std::vector<std::vector<double>> fitnesses_score_ranks;
    for (size_t i = 0; i < expected_size; ++i)
    {
        auto fitness_ranks = getFitnessRanking(a_population_fitness_values, i);

        // Normalize ranks with max ranks
        double max_rank = *std::max_element(fitness_ranks.begin(), fitness_ranks.end());
        if (max_rank > 0)
        {
            for (auto& rank : fitness_ranks)
                rank /= max_rank;
        }

        fitnesses_score_ranks.push_back(fitness_ranks);
    }

    // Calculate weighted ranks
    std::vector<double> weighted_ranks(a_population_fitness_values.size(), 0.0);
    for (size_t i = 0; i < a_population_fitness_values.size(); ++i)
    {
        for (size_t j = 0; j < expected_size; ++j)
            weighted_ranks[i] += fitnesses_score_ranks[j][i] * a_fitness_coefficients[j];
    }

    auto best_candidate = std::min_element(weighted_ranks.begin(), weighted_ranks.end());

    return std::distance(weighted_ranks.begin(), best_candidate);
}

}; // adaptive_voting_algorithm

}; // arena_core
