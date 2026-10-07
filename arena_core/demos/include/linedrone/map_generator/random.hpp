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
#include <cstdint>
#include <random>
#include <vector>

// External Libraries
#include <Eigen/Dense>


namespace arena_demos
{

namespace map_generator
{

/**
 * @brief Orientation applied to a randomly generated obstacle.
 */
enum class OrientationMode
{
    None,   // Axis aligned
    Yaw,    // Random rotation around the z axis only (obstacle stays upright)
    Full    // Uniformly distributed random 3D rotation
}; // enum class OrientationMode


/**
 * @brief Seeded random number generator that gives the same sequence on every platform.
 *
 * std::mt19937_64 is fully specified by the standard, but the std::*_distribution classes are not
 * (libstdc++ and libc++ give different values for the same engine state). Every distribution is
 * therefore implemented here directly from the raw engine output, so a seed always produces the same map.
 */
class Random
{
public:

    explicit Random(uint64_t seed) : engine_(seed) {}

    /**
     * @brief Derive an independent seed from a base seed and a stream index (splitmix64 mixing).
     *
     * Used to give every obstacle its own random stream, so obstacle i is the same whatever the number
     * of obstacles requested (e.g. increasing the density only adds obstacles to the map).
     */
    static uint64_t deriveSeed(uint64_t seed, uint64_t stream);

    /** @brief Uniform double in [0, 1). */
    double uniform();

    /** @brief Uniform double in [min, max). */
    double uniform(double min, double max);

    /** @brief Uniform double in [min, max) on a logarithmic scale (min and max must be > 0). */
    double logUniform(double min, double max);

    /** @brief Uniform integer in [min, max] (both inclusive). */
    int uniformInt(int min, int max);

    /** @brief Normally distributed double (Box-Muller). */
    double normal(double mean, double stddev);

    /**
     * @brief Log-normally distributed double with the given mean and coefficient of variation (stddev / mean).
     *
     * Always positive, which makes it a good fit for volumes.
     */
    double logNormal(double mean, double std_ratio);

    /** @brief Index drawn with a probability proportional to its weight. */
    size_t weightedIndex(const std::vector<double>& weights);

    /** @brief Random rotation matrix following the orientation mode. */
    Eigen::Matrix3d rotation(OrientationMode mode);

private:

    std::mt19937_64 engine_;

}; // class Random

}; // namespace map_generator

}; // namespace arena_demos
