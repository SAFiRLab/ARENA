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

#include <gtest/gtest.h>
#include "arena_core/math/algorithm/voting_algorithm.h"

#include <cmath>
#include <stdexcept>
#include <vector>

using namespace arena_core;

namespace
{

double sum(const ObjectiveCoefficients& a_coefficients)
{
    return a_coefficients[0] + a_coefficients[1] + a_coefficients[2];
}

// Pareto set (time, safety, energy): fastest and lowest energy (0), balanced (1), safest (2), intermediate (3)
const std::vector<std::vector<double>> FRONT = {{138.0, 0.60, 452.0e3},
                                                {148.0, 0.15, 480.0e3},
                                                {171.0, 0.00, 528.0e3},
                                                {155.0, 0.08, 495.0e3}};

}; // namespace


TEST(RiskWeightedTchebycheffVotingTest, NominalCoefficientsWithoutRisk)
{
    RiskWeightedTchebycheffVoting voting;
    ObjectiveCoefficients k = voting.computeCoefficients(OperationalRisks());
    for (double coefficient : k)
        EXPECT_NEAR(coefficient, 1.0 / 3.0, 1e-12);
}

TEST(RiskWeightedTchebycheffVotingTest, CoefficientsSumToOne)
{
    RiskWeightedTchebycheffVoting voting;
    for (double wind : {0.0, 0.3, 0.9})
        for (double localization : {0.0, 0.5, 1.0})
            for (double battery : {0.0, 0.4, 1.0})
            {
                ObjectiveCoefficients k = voting.computeCoefficients({wind, 0.2, localization, battery});
                EXPECT_NEAR(sum(k), 1.0, 1e-12);
                for (double coefficient : k)
                {
                    EXPECT_TRUE(std::isfinite(coefficient));
                    EXPECT_GE(coefficient, 0.0);
                }
            }
}

TEST(RiskWeightedTchebycheffVotingTest, MatchesTheClosedForm)
{
    // k_j = k_j,0 prod_r (1 - r)^(-beta a_jr), normalized, with the default influences and beta = 2
    RiskWeightedTchebycheffVoting voting;
    const OperationalRisks risks{0.6, 0.0, 0.2, 0.3}; // wind, communication, localization, battery
    const double r[4] = {risks.wind, risks.communication, risks.localization, risks.battery};
    const auto& a = RiskWeightedTchebycheffVoting::DEFAULT_INFLUENCE;
    double expected[3];
    double total = 0.0;
    for (int j = 0; j < 3; ++j)
    {
        expected[j] = 1.0 / 3.0;
        for (int i = 0; i < 4; ++i)
            expected[j] *= std::pow(1.0 - r[i], -2.0 * a[j][i]);
        total += expected[j];
    }
    ObjectiveCoefficients k = voting.computeCoefficients(risks);
    for (int j = 0; j < 3; ++j)
        EXPECT_NEAR(k[j], expected[j] / total, 1e-12);
}

TEST(RiskWeightedTchebycheffVotingTest, CriticalRisksReachTheExtremes)
{
    RiskWeightedTchebycheffVoting voting;

    // A critical localization (or communication, or wind) risk gives all the weight to safety
    for (const OperationalRisks& risks : {OperationalRisks{0.0, 0.0, 1.0, 0.0}, OperationalRisks{0.0, 1.0, 0.0, 0.0},
                                          OperationalRisks{1.0, 0.0, 0.0, 0.0}})
    {
        ObjectiveCoefficients k = voting.computeCoefficients(risks);
        EXPECT_NEAR(k[1], 1.0, 1e-5);
        EXPECT_EQ(voting.vote(FRONT, risks), 2); // Safest
    }

    // A critical battery risk gives all the weight to time
    ObjectiveCoefficients k = voting.computeCoefficients({0.0, 0.0, 0.0, 1.0});
    EXPECT_NEAR(k[0], 1.0, 1e-5);
    EXPECT_EQ(voting.vote(FRONT, {0.0, 0.0, 0.0, 1.0}), 0); // Fastest
}

TEST(RiskWeightedTchebycheffVotingTest, SelectionFollowsTheRisks)
{
    RiskWeightedTchebycheffVoting voting;
    EXPECT_EQ(voting.vote(FRONT, OperationalRisks()), 1); // Balanced without risk

    // The safety cost of the selected candidate does not increase with the localization risk
    double previous_safety = 1.0;
    for (double localization = 0.0; localization <= 1.0; localization += 0.05)
    {
        double safety = FRONT[voting.vote(FRONT, {0.0, 0.0, localization, 0.0})][1];
        EXPECT_LE(safety, previous_safety + 1e-12);
        previous_safety = safety;
    }
}

TEST(RiskWeightedTchebycheffVotingTest, FixedCoefficients)
{
    RiskWeightedTchebycheffVoting voting;
    EXPECT_EQ(voting.selectCandidate(FRONT, {1.0, 0.0, 0.0}), 0);
    EXPECT_EQ(voting.selectCandidate(FRONT, {0.0, 1.0, 0.0}), 2);
}

TEST(RiskWeightedTchebycheffVotingTest, InvalidInputs)
{
    RiskWeightedTchebycheffVoting voting;
    EXPECT_THROW(voting.computeCoefficients({1.5, 0.0, 0.0, 0.0}), std::invalid_argument);
    EXPECT_THROW(voting.computeCoefficients({0.0, -0.1, 0.0, 0.0}), std::invalid_argument);
    EXPECT_THROW(voting.computeCoefficients({0.0, 0.0, std::nan(""), 0.0}), std::invalid_argument);
    EXPECT_THROW(voting.selectCandidate({}, {1.0, 0.0, 0.0}), std::invalid_argument);
    EXPECT_THROW(voting.selectCandidate(FRONT, {0.0, 0.0, 0.0}), std::invalid_argument);

    RiskWeightedTchebycheffVoting::Parameters parameters;
    parameters.gain = 0.0;
    EXPECT_THROW(RiskWeightedTchebycheffVoting{parameters}, std::invalid_argument);
    parameters = RiskWeightedTchebycheffVoting::Parameters();
    parameters.nominal_coefficients = {0.0, 0.5, 0.5};
    EXPECT_THROW(RiskWeightedTchebycheffVoting{parameters}, std::invalid_argument);
}

TEST(RankVotingAlgorithmTest, Equation11)
{
    RankVotingAlgorithm voting({0.25, 0.5, 0.25});
    const OperationalRisks risks{0.4, 0.2, 0.6, 0.1};
    const double s = 0.5 * 0.4 + 0.25 * 0.2 + 0.25 * 0.6 - 0.1;
    const double k_t = 0.25 * (1.0 - s), k_s = 0.5 * (1.0 + s), k_e = 0.25 * (1.0 + 0.5 * 0.4 + 0.5 * 0.1);
    ObjectiveCoefficients k = voting.computeCoefficients(risks);
    EXPECT_NEAR(k[0], k_t / (k_t + k_s + k_e), 1e-12);
    EXPECT_NEAR(k[1], k_s / (k_t + k_s + k_e), 1e-12);
    EXPECT_NEAR(k[2], k_e / (k_t + k_s + k_e), 1e-12);
}

TEST(RankVotingAlgorithmTest, SelectionIsTheRankVoting)
{
    RankVotingAlgorithm voting;
    EXPECT_EQ(voting.selectCandidate(FRONT, {1.0, 0.0, 0.0}), 0);
    EXPECT_EQ(voting.selectCandidate(FRONT, {0.0, 1.0, 0.0}), 2);
}

TEST(VotingAlgorithmFactoryTest, CreatesTheRequestedAlgorithm)
{
    EXPECT_EQ(votingAlgorithmTypeFromString("rank"), VotingAlgorithmType::Rank);
    EXPECT_EQ(votingAlgorithmTypeFromString("risk_weighted_tchebycheff"), VotingAlgorithmType::RiskWeightedTchebycheff);
    EXPECT_THROW(votingAlgorithmTypeFromString("unknown"), std::invalid_argument);

    VotingAlgorithmSettings settings;
    std::unique_ptr<IVotingAlgorithm> voting = createVotingAlgorithm(settings);
    EXPECT_EQ(voting->name(), "risk_weighted_tchebycheff");

    settings.type = VotingAlgorithmType::Rank;
    voting = createVotingAlgorithm(settings);
    EXPECT_EQ(voting->name(), "rank");
}
