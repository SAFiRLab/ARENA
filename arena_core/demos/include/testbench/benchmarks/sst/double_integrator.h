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
#include "testbench/benchmarks/common/native_trajectory.h"

// System
#include <array>
#include <cstdint>
#include <functional>
#include <random>
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

/**
 * @brief 3D double integrator of the dynamic sampling-based planners (SST, poSST): state (position, velocity), control
 * (acceleration), x'' = u with ||u|| <= max_acceleration. The states must be valid (position in the map bounds and valid
 * for the validity function, ||v|| <= max_speed).
 */
struct DoubleIntegratorConfig
{
    Eigen::Vector3d min_bounds = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bounds = Eigen::Vector3d::Ones();
    double max_speed = 0.5;               // m/s, ||v||
    double max_acceleration = 2.2;        // m/s^2, ||u||
    double propagation_step = 0.1;        // s, the states are checked at every step (OMPL propagation step size)
    unsigned min_control_steps = 1;       // A control is applied during a random number of steps in
    unsigned max_control_steps = 20;      // [min_control_steps, max_control_steps] (MonteCarlo-Prop, T_prop = max * step)
}; // struct DoubleIntegratorConfig


class DoubleIntegrator
{
public:

    using ValidityFn = std::function<bool(const Eigen::Vector3d&)>;

    DoubleIntegrator(const DoubleIntegratorConfig& a_config, ValidityFn a_validity)
    : config_(a_config), validity_(std::move(a_validity)) {}

    /** @brief State after applying the control a_control during a_time from a_state. */
    static TrajectoryPoint propagate(const TrajectoryPoint& a_state, const Eigen::Vector3d& a_control, double a_time)
    {
        return {a_state.position + a_state.velocity * a_time + 0.5 * a_control * a_time * a_time,
                a_state.velocity + a_control * a_time, a_state.time + a_time};
    }

    /** @brief Position in the bounds and valid, speed at most max_speed. */
    bool isValid(const TrajectoryPoint& a_state) const
    {
        for (int i = 0; i < 3; ++i)
            if (a_state.position[i] < config_.min_bounds[i] || a_state.position[i] > config_.max_bounds[i])
                return false;
        return a_state.velocity.norm() <= config_.max_speed && validity_(a_state.position);
    }

    /**
     * @brief Applies a_control during a_steps propagation steps from a_state, checking every step (OMPL
     * propagateWhileValid). Returns false at the first invalid step, otherwise a_states holds the states of the steps
     * (a_states.back() is the final state). The speed is a convex function of the time under a constant control, it is
     * at most max_speed between two valid steps.
     */
    bool propagateWhileValid(const TrajectoryPoint& a_state, const Eigen::Vector3d& a_control, unsigned a_steps,
                             std::vector<TrajectoryPoint>& a_states) const
    {
        a_states.clear();
        for (unsigned k = 1; k <= a_steps; ++k)
        {
            a_states.push_back(propagate(a_state, a_control, k * config_.propagation_step));
            if (!isValid(a_states.back()))
                return false;
        }
        return true;
    }

    /** @brief Control sampled uniformly in the ball ||u|| <= max_acceleration (the control space U). */
    template <typename Rng>
    Eigen::Vector3d sampleControl(Rng& a_rng) const
    {
        std::normal_distribution<double> normal(0.0, 1.0);
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        Eigen::Vector3d direction(normal(a_rng), normal(a_rng), normal(a_rng));
        const double norm = direction.norm();
        if (norm < 1.0e-12)
            return Eigen::Vector3d::Zero();
        return direction / norm * config_.max_acceleration * std::cbrt(uniform(a_rng));
    }

    /** @brief Number of propagation steps sampled uniformly in [min_control_steps, max_control_steps]. */
    template <typename Rng>
    unsigned sampleSteps(Rng& a_rng) const
    {
        return std::uniform_int_distribution<unsigned>(config_.min_control_steps, config_.max_control_steps)(a_rng);
    }

    /** @brief Position sampled uniformly in the map bounds. */
    template <typename Rng>
    Eigen::Vector3d samplePosition(Rng& a_rng) const
    {
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        Eigen::Vector3d p;
        for (int i = 0; i < 3; ++i)
            p[i] = config_.min_bounds[i] + uniform(a_rng) * (config_.max_bounds[i] - config_.min_bounds[i]);
        return p;
    }

    const DoubleIntegratorConfig& config() const { return config_; }

private:
    DoubleIntegratorConfig config_;
    ValidityFn validity_;

}; // class DoubleIntegrator


/**
 * @brief Motion plan of SST and poSST: the states at every propagation step from the start (with their velocities and
 * times) and the cost(s) of the plan.
 */
struct KinodynamicPlan
{
    NativeTrajectory states;
    std::array<double, 3> costs = {0.0, 0.0, 0.0}; // SST: costs[0] is the duration
}; // struct KinodynamicPlan

}; // namespace arena_benchmarks
