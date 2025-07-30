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

}; // adaptive_voting_algorithm

}; // arena_core
