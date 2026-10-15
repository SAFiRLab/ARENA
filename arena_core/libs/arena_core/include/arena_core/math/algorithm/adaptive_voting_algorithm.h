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

#pragma once

// System
#include <vector>


namespace arena_core
{

namespace adaptive_voting_algorithm
{

/**
* @brief Get lowest cost from a population of candidates and a fitness index
*
* This function retrieves the lowest cost from a population of candidates based on a specific fitness index.
*
* @param a_population_fitness_values A vector containing the fitness values of the candidates.
* @param a_fitness_index The index of the fitness value to be used for determining the lowest cost.
* @return The lowest cost value from the population based on the specified fitness index.
*/
double getLowestCost(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index);

/**
* @brief Get the costs from a population of candidates based on the lowest cost of a fitness index
*
* This function retrieves all the fitness values of a candidate that has the lowest cost based on a specific fitness index.
*
* @param a_population_fitness_values A vector containing the fitness values of the candidates.
* @param a_fitness_index The index of the fitness value to be used for determining the lowest cost.
* @return The costs values from the candidate with the lowest cost based on the specified fitness index.
*/
std::vector<double> getCostsFromLowest(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index);

/**
* @brief Get the highest cost from a population of candidates and a fitness index
*
* This function retrieves the highest cost from a population of candidates based on a specific fitness index.
*
* @param a_population_fitness_values A vector containing the fitness values of the candidates.
* @param a_fitness_index The index of the fitness value to be used for determining the highest cost.
* @return The highest cost value from the population based on the specified fitness index.
*/
double getHighestCost(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index);

/**
* @brief Get the costs from a population of candidates based on the highest cost of a fitness index 
*
* This function retrieves all the fitness values of a candidate that has the highest cost based on a specific fitness index.
*
* @param a_population_fitness_values A vector containing the fitness values of the candidates.
* @param a_fitness_index The index of the fitness value to be used for determining the highest cost.
* @return The costs values from the candidate with the highest cost based on the specified fitness index.
*/
std::vector<double> getCostsFromHighest(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index);

/**
* @brief Fitness ranking of the population
*
* This algorithm takes a set of candidates and their fitness values, and a fitness index
* to rank the candidates based on that specific fitness.
* 
* @param a_population_fitness_values A vector containing the fitness values of the candidates.
* @param a_fitness_index The index of the fitness value to be used for ranking.
* @return A vector of indices representing the ranked order of candidates based on the specified fitness.
*/
std::vector<double> getFitnessRanking(const std::vector<std::vector<double>>& a_population_fitness_values, int a_fitness_index);

/**
* @brief Adaptive Voting Algorithm for determining the best candidate based on adaptive coefficients.
*
* This algorithm takes a set of candidates and their fitness values, and determines
* the best candidate based on an daptive weighted voting mechanism. The weights are considered adjusted
* adaptively in real-time based on mission related factors.
* 
* @param a_population_fitness_values A vector containing the fitness values of the candidates.
* @param a_fitness_coefficients A vector containing the coefficients for each candidate's fitness value
*                               that adaptively adjust the influence of each candidate's fitness.
* @return The index of the candidate with the highest weighted rank.
*/
int getBetterCandidateIndex(const std::vector<std::vector<double>>& a_population_fitness_values, const std::vector<double>& a_fitness_coefficients);

/**
* @brief Weighted Tchebycheff selection of the best candidate (alternative to the rank voting of getBetterCandidateIndex).
*
* The fitness values are normalized with the minimum and maximum of every objective over the candidates, the best
* candidate minimizes the largest weighted normalized fitness, max_j(w_j * f_ij), plus a small weighted sum (augmented
* Tchebycheff, rho) so that a weakly dominated candidate is never chosen. Unlike a weighted sum of ranks or of costs,
* every Pareto optimal candidate can be chosen with some weights, including the ones in non-convex parts of the front.
*
* @param a_population_fitness_values A vector containing the fitness values of the candidates.
* @param a_fitness_coefficients The weight of every fitness value (non-negative, at least one non-zero).
* @param a_rho Weight of the augmentation term.
* @return The index of the chosen candidate.
*/
int getTchebycheffCandidateIndex(const std::vector<std::vector<double>>& a_population_fitness_values,
                                 const std::vector<double>& a_fitness_coefficients, double a_rho = 1.0e-3);

}; // adaptive_voting_algorithm

}; // arena_core
