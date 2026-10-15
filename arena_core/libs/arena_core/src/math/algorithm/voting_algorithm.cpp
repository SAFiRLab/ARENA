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

#include "arena_core/math/algorithm/voting_algorithm.h"
#include "arena_core/math/algorithm/adaptive_voting_algorithm.h"

// System
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>


namespace arena_core
{

namespace
{

// Risks in the order of the columns of the influence matrix, checked to be in [0, 1]
std::array<double, 4> checkedRisks(const OperationalRisks& a_risks, const std::string& a_caller)
{
    const std::array<double, 4> risks = {a_risks.wind, a_risks.communication, a_risks.localization, a_risks.battery};
    for (double risk : risks)
    {
        if (!(risk >= 0.0 && risk <= 1.0))
            throw std::invalid_argument("arena_core::" + a_caller + "::computeCoefficients() -> The risks must be in [0, 1].");
    }
    return risks;
}

void checkNominalCoefficients(const ObjectiveCoefficients& a_coefficients, const std::string& a_caller)
{
    if (std::any_of(a_coefficients.begin(), a_coefficients.end(), [](double k) { return !(k > 0.0) || !std::isfinite(k); }))
        throw std::invalid_argument("arena_core::" + a_caller + " -> The nominal coefficients must be positive.");
}

// Coefficients divided by their sum
ObjectiveCoefficients normalized(ObjectiveCoefficients a_coefficients)
{
    double sum = a_coefficients[0] + a_coefficients[1] + a_coefficients[2];
    if (sum <= 0.0)
        return {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
    for (double& coefficient : a_coefficients)
        coefficient /= sum;
    return a_coefficients;
}

}; // namespace


/************************************************* RankVotingAlgorithm ************************************************/

RankVotingAlgorithm::RankVotingAlgorithm(const ObjectiveCoefficients& a_nominal_coefficients)
: nominal_coefficients_(a_nominal_coefficients)
{
    checkNominalCoefficients(nominal_coefficients_, "RankVotingAlgorithm");
}

ObjectiveCoefficients RankVotingAlgorithm::computeCoefficients(const OperationalRisks& a_risks) const
{
    checkedRisks(a_risks, "RankVotingAlgorithm");

    // Eq. 11 of the first submission, the coefficients are clamped at 0 (s > 1) and normalized (gamma)
    const double s = 0.5 * a_risks.wind + 0.25 * a_risks.communication + 0.25 * a_risks.localization - a_risks.battery;
    return normalized({std::max(0.0, nominal_coefficients_[0] * (1.0 - s)),
                       std::max(0.0, nominal_coefficients_[1] * (1.0 + s)),
                       std::max(0.0, nominal_coefficients_[2] * (1.0 + 0.5 * a_risks.wind + 0.5 * a_risks.battery))});
}

int RankVotingAlgorithm::selectCandidate(const std::vector<std::vector<double>>& a_population_fitness_values,
                                         const ObjectiveCoefficients& a_coefficients) const
{
    return adaptive_voting_algorithm::getBetterCandidateIndex(a_population_fitness_values,
                                                              std::vector<double>(a_coefficients.begin(), a_coefficients.end()));
}


/******************************************** RiskWeightedTchebycheffVoting *******************************************/

RiskWeightedTchebycheffVoting::RiskWeightedTchebycheffVoting(const Parameters& a_parameters)
: parameters_(a_parameters)
{
    checkNominalCoefficients(parameters_.nominal_coefficients, "RiskWeightedTchebycheffVoting");
    if (!(parameters_.gain > 0.0) || !std::isfinite(parameters_.gain))
        throw std::invalid_argument("arena_core::RiskWeightedTchebycheffVoting -> The gain must be positive.");
    if (!(parameters_.augmentation >= 0.0) || !std::isfinite(parameters_.augmentation))
        throw std::invalid_argument("arena_core::RiskWeightedTchebycheffVoting -> The augmentation must be non-negative.");
}

ObjectiveCoefficients RiskWeightedTchebycheffVoting::computeCoefficients(const OperationalRisks& a_risks) const
{
    const std::array<double, 4> risks = checkedRisks(a_risks, "RiskWeightedTchebycheffVoting");

    // h(r) = -ln(1 - r), r capped below 1 so that a critical risk gives a finite but dominant exponent
    constexpr double MAX_RISK = 1.0 - 1.0e-12;
    std::array<double, 4> hazards;
    for (size_t r = 0; r < risks.size(); ++r)
        hazards[r] = -std::log1p(-std::min(risks[r], MAX_RISK));

    // Logarithm of k_j,0 prod_r (1 - r)^(-beta a_jr), normalized in the log domain (log-sum-exp) to avoid overflows
    std::array<double, 3> log_weights;
    for (size_t j = 0; j < log_weights.size(); ++j)
    {
        double exponent = 0.0;
        for (size_t r = 0; r < hazards.size(); ++r)
            exponent += parameters_.influence[j][r] * hazards[r];
        log_weights[j] = std::log(parameters_.nominal_coefficients[j]) + parameters_.gain * exponent;
    }
    const double max_log_weight = *std::max_element(log_weights.begin(), log_weights.end());

    ObjectiveCoefficients coefficients;
    for (size_t j = 0; j < coefficients.size(); ++j)
        coefficients[j] = std::exp(log_weights[j] - max_log_weight);
    return normalized(coefficients);
}

int RiskWeightedTchebycheffVoting::selectCandidate(const std::vector<std::vector<double>>& a_population_fitness_values,
                                                   const ObjectiveCoefficients& a_coefficients) const
{
    return adaptive_voting_algorithm::getTchebycheffCandidateIndex(a_population_fitness_values,
                                                                   std::vector<double>(a_coefficients.begin(), a_coefficients.end()),
                                                                   parameters_.augmentation);
}


/******************************************************* Factory ******************************************************/

VotingAlgorithmType votingAlgorithmTypeFromString(const std::string& a_name)
{
    if (a_name == "rank")
        return VotingAlgorithmType::Rank;
    if (a_name == "risk_weighted_tchebycheff")
        return VotingAlgorithmType::RiskWeightedTchebycheff;
    throw std::invalid_argument("arena_core::votingAlgorithmTypeFromString() -> Unknown voting algorithm \"" + a_name +
                                "\" (rank or risk_weighted_tchebycheff).");
}

std::unique_ptr<IVotingAlgorithm> createVotingAlgorithm(const VotingAlgorithmSettings& a_settings)
{
    switch (a_settings.type)
    {
        case VotingAlgorithmType::Rank:
            return std::make_unique<RankVotingAlgorithm>(a_settings.nominal_coefficients);

        case VotingAlgorithmType::RiskWeightedTchebycheff:
        {
            RiskWeightedTchebycheffVoting::Parameters parameters;
            parameters.nominal_coefficients = a_settings.nominal_coefficients;
            parameters.gain = a_settings.gain;
            parameters.augmentation = a_settings.augmentation;
            return std::make_unique<RiskWeightedTchebycheffVoting>(parameters);
        }
    }
    throw std::invalid_argument("arena_core::createVotingAlgorithm() -> Unknown voting algorithm type.");
}

}; // namespace arena_core
