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
#include <memory>
#include <string>
#include <vector>


namespace arena_core
{

/**
 * @brief Normalized estimates of the operational risks, in [0, 1] (0: nominal, 1: critical).
 */
struct OperationalRisks
{
    double wind = 0.0;
    double communication = 0.0;
    double localization = 0.0;
    double battery = 0.0;
};

/** @brief Coefficients of the objectives (time, safety, energy). */
using ObjectiveCoefficients = std::array<double, 3>;


/**
 * @brief Interface of the voting algorithms selecting the trajectory to execute from the approximated Pareto set.
 *
 * A voting algorithm maps the operational risks to the coefficients of the objectives (computeCoefficients()) and
 * selects one candidate of the set with these coefficients (selectCandidate()). The two steps are separate so that the
 * coefficients can also be given directly (tests with fixed coefficients). The fitness values of every candidate are
 * ordered (time, safety, energy).
 */
class IVotingAlgorithm
{
public:
    virtual ~IVotingAlgorithm() = default;

    /**
     * @brief Coefficients of the objectives (time, safety, energy) for the given risks, summed to 1.
     * @throw std::invalid_argument if a risk is not in [0, 1].
     */
    virtual ObjectiveCoefficients computeCoefficients(const OperationalRisks& a_risks) const = 0;

    /**
     * @brief Index of the candidate selected with the given coefficients.
     * @param a_population_fitness_values Fitness values (time, safety, energy) of every candidate.
     * @param a_coefficients Coefficients of the objectives (non-negative, at least one non-zero).
     * @throw std::invalid_argument if the population is empty or the coefficients are invalid.
     */
    virtual int selectCandidate(const std::vector<std::vector<double>>& a_population_fitness_values,
                                const ObjectiveCoefficients& a_coefficients) const = 0;

    /** @brief Name of the algorithm (the string accepted by votingAlgorithmTypeFromString()). */
    virtual std::string name() const = 0;

    /** @brief Index of the candidate selected for the given risks: selectCandidate(computeCoefficients(a_risks)). */
    int vote(const std::vector<std::vector<double>>& a_population_fitness_values, const OperationalRisks& a_risks) const
    {
        return selectCandidate(a_population_fitness_values, computeCoefficients(a_risks));
    }

}; // class IVotingAlgorithm


/**
 * @brief Rank voting of the first ARENA submission.
 *
 * Coefficients (Eq. 11): with s = WR/2 + CR/4 + LR/4 - BR,
 *     k_T = k_T,0 (1 - s), k_S = k_S,0 (1 + s), k_E = k_E,0 (1 + WR/2 + BR/2),
 * clamped at 0 and normalized to a sum of 1. Selection: weighted sum of the normalized ranks of the candidates
 * (adaptive_voting_algorithm::getBetterCandidateIndex).
 */
class RankVotingAlgorithm final : public IVotingAlgorithm
{
public:
    explicit RankVotingAlgorithm(const ObjectiveCoefficients& a_nominal_coefficients = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0});

    ObjectiveCoefficients computeCoefficients(const OperationalRisks& a_risks) const override;
    int selectCandidate(const std::vector<std::vector<double>>& a_population_fitness_values,
                        const ObjectiveCoefficients& a_coefficients) const override;
    std::string name() const override { return "rank"; }

private:
    ObjectiveCoefficients nominal_coefficients_;

}; // class RankVotingAlgorithm


/**
 * @brief Risk-weighted augmented Tchebycheff voting.
 *
 * Coefficients: with the influence a_jr of every risk r (wind, communication, localization, battery) on every
 * objective j (time, safety, energy),
 *     k_j = k_j,0 prod_r (1 - r)^(-beta a_jr) / sum_m k_m,0 prod_r (1 - r)^(-beta a_mr).
 * They sum to 1, equal the nominal coefficients without risk, reduce to a linear adjustment of the nominal coefficients
 * for low risks and give all the weight to the objective a risk influences most when this risk tends to 1.
 *
 * Selection: the fitness values are normalized with the minimum and maximum of every objective over the candidates and
 * the selected candidate minimizes max_j k_j f_ij + epsilon sum_j k_j f_ij (augmented weighted Tchebycheff,
 * adaptive_voting_algorithm::getTchebycheffCandidateIndex): every Pareto optimal candidate can be selected.
 */
class RiskWeightedTchebycheffVoting final : public IVotingAlgorithm
{
public:
    /** @brief Influence of the risks (columns: wind, communication, localization, battery) on the objectives (rows). */
    using InfluenceMatrix = std::array<std::array<double, 4>, 3>;

    /**
     * @brief Default influences: the signs of Eq. 11, the influence of the wind on energy at half of its influence on
     * safety, every column scaled to a largest magnitude of 1.
     */
    static constexpr InfluenceMatrix DEFAULT_INFLUENCE = {{{-1.0, -1.0, -1.0, 1.0},
                                                           {1.0, 1.0, 1.0, -1.0},
                                                           {0.5, 0.0, 0.0, 0.5}}};

    struct Parameters
    {
        ObjectiveCoefficients nominal_coefficients = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}; // k_j,0
        InfluenceMatrix influence = DEFAULT_INFLUENCE;                                  // a_jr
        double gain = 2.0;                                                              // beta
        double augmentation = 1.0e-3;                                                   // epsilon
    };

    RiskWeightedTchebycheffVoting() : RiskWeightedTchebycheffVoting(Parameters()) {}
    explicit RiskWeightedTchebycheffVoting(const Parameters& a_parameters);

    ObjectiveCoefficients computeCoefficients(const OperationalRisks& a_risks) const override;
    int selectCandidate(const std::vector<std::vector<double>>& a_population_fitness_values,
                        const ObjectiveCoefficients& a_coefficients) const override;
    std::string name() const override { return "risk_weighted_tchebycheff"; }

    const Parameters& getParameters() const { return parameters_; }

private:
    Parameters parameters_;

}; // class RiskWeightedTchebycheffVoting


/** @brief Voting algorithms available through createVotingAlgorithm(). */
enum class VotingAlgorithmType
{
    Rank,
    RiskWeightedTchebycheff
};

/**
 * @brief Type of the voting algorithm from its name ("rank" or "risk_weighted_tchebycheff").
 * @throw std::invalid_argument for an unknown name.
 */
VotingAlgorithmType votingAlgorithmTypeFromString(const std::string& a_name);

/**
 * @brief Settings of the voting algorithms created by createVotingAlgorithm().
 */
struct VotingAlgorithmSettings
{
    VotingAlgorithmType type = VotingAlgorithmType::RiskWeightedTchebycheff;
    ObjectiveCoefficients nominal_coefficients = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
    double gain = 2.0;            // RiskWeightedTchebycheff only
    double augmentation = 1.0e-3; // RiskWeightedTchebycheff only
};

/** @brief Voting algorithm of the given settings. */
std::unique_ptr<IVotingAlgorithm> createVotingAlgorithm(const VotingAlgorithmSettings& a_settings);

}; // namespace arena_core
