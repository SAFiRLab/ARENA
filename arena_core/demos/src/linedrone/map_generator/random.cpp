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

// Local
#include "linedrone/map_generator/random.hpp"

// System
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

// External Libraries
#include <Eigen/Geometry>


namespace arena_demos
{

namespace map_generator
{

uint64_t Random::deriveSeed(uint64_t seed, uint64_t stream)
{
    auto splitmix64 = [](uint64_t x)
    {
        x += 0x9E3779B97F4A7C15ULL;
        x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
        x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
        return x ^ (x >> 31);
    };

    return splitmix64(seed ^ splitmix64(stream));
}

double Random::uniform()
{
    // 53 random bits -> double in [0, 1)
    return static_cast<double>(engine_() >> 11) * 0x1.0p-53;
}

double Random::uniform(double min, double max)
{
    return min + (max - min) * uniform();
}

double Random::logUniform(double min, double max)
{
    if (min <= 0.0 || max <= 0.0)
        throw std::invalid_argument("Random::logUniform => min and max must be positive.");

    return std::exp(uniform(std::log(min), std::log(max)));
}

int Random::uniformInt(int min, int max)
{
    if (max < min)
        throw std::invalid_argument("Random::uniformInt => max must be greater or equal to min.");

    int value = min + static_cast<int>(std::floor(uniform() * (static_cast<double>(max) - min + 1.0)));
    return std::min(value, max);
}

double Random::normal(double mean, double stddev)
{
    double u1 = 1.0 - uniform(); // (0, 1], avoids log(0)
    double u2 = uniform();
    return mean + stddev * std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
}

double Random::logNormal(double mean, double std_ratio)
{
    if (mean <= 0.0)
        throw std::invalid_argument("Random::logNormal => mean must be positive.");

    if (std_ratio <= 0.0)
        return mean;

    double sigma_squared = std::log(1.0 + std_ratio * std_ratio);
    double mu = std::log(mean) - sigma_squared / 2.0;
    return std::exp(normal(mu, std::sqrt(sigma_squared)));
}

size_t Random::weightedIndex(const std::vector<double>& weights)
{
    double total = std::accumulate(weights.begin(), weights.end(), 0.0);
    if (weights.empty() || total <= 0.0)
        throw std::invalid_argument("Random::weightedIndex => the sum of the weights must be positive.");

    double target = uniform() * total;
    double cumulative = 0.0;
    for (size_t i = 0; i < weights.size(); ++i)
    {
        cumulative += weights[i];
        if (target < cumulative && weights[i] > 0.0)
            return i;
    }

    // Floating point rounding: return the last index with a positive weight
    for (size_t i = weights.size(); i-- > 0;)
    {
        if (weights[i] > 0.0)
            return i;
    }
    return weights.size() - 1;
}

Eigen::Matrix3d Random::rotation(OrientationMode mode)
{
    switch (mode)
    {
        case OrientationMode::None:
            return Eigen::Matrix3d::Identity();

        case OrientationMode::Yaw:
            return Eigen::AngleAxisd(uniform(0.0, 2.0 * M_PI), Eigen::Vector3d::UnitZ()).toRotationMatrix();

        case OrientationMode::Full:
        {
            // Uniformly distributed unit quaternion (Shoemake, Graphics Gems III)
            double u1 = uniform();
            double u2 = uniform(0.0, 2.0 * M_PI);
            double u3 = uniform(0.0, 2.0 * M_PI);
            double a = std::sqrt(1.0 - u1);
            double b = std::sqrt(u1);
            Eigen::Quaterniond q(a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3));
            return q.normalized().toRotationMatrix();
        }
    }

    return Eigen::Matrix3d::Identity();
}

}; // namespace map_generator

}; // namespace arena_demos
