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

#include "testbench/benchmarks/common/native_trajectory.h"

// System
#include <cmath>
#include <stdexcept>


namespace arena_benchmarks
{

NativeTrajectory segmentSpeedTrajectory(const std::vector<Eigen::Vector3d>& a_waypoints, const std::vector<double>& a_segment_speeds)
{
    if (a_waypoints.size() < 2 || a_segment_speeds.size() + 1 != a_waypoints.size())
        throw std::invalid_argument("segmentSpeedTrajectory: K >= 2 waypoints and K - 1 segment speeds are needed.");

    NativeTrajectory trajectory;
    double time = 0.0;
    trajectory.push_back({a_waypoints.front(), Eigen::Vector3d::Zero(), time});
    for (size_t i = 0; i + 1 < a_waypoints.size(); ++i)
    {
        const Eigen::Vector3d displacement = a_waypoints[i + 1] - a_waypoints[i];
        const double length = displacement.norm();
        if (length < 1.0e-9)
            continue;
        if (!(a_segment_speeds[i] > 0.0))
            throw std::invalid_argument("segmentSpeedTrajectory: the speeds must be positive.");

        const Eigen::Vector3d velocity = displacement / length * a_segment_speeds[i];
        trajectory.push_back({a_waypoints[i], velocity, time}); // Instantaneous change of velocity
        time += length / a_segment_speeds[i];
        trajectory.push_back({a_waypoints[i + 1], velocity, time});
    }
    trajectory.push_back({a_waypoints.back(), Eigen::Vector3d::Zero(), time});
    return trajectory;
}

NativeTrajectory constantSpeedTrajectory(const std::vector<Eigen::Vector3d>& a_waypoints, double a_speed)
{
    return segmentSpeedTrajectory(a_waypoints, std::vector<double>(a_waypoints.size() > 0 ? a_waypoints.size() - 1 : 0, a_speed));
}

NativeTrajectory constantSpeedTrajectory(const Eigen::MatrixXd& a_waypoints, double a_speed)
{
    std::vector<Eigen::Vector3d> waypoints;
    for (int i = 0; i < a_waypoints.cols(); ++i)
        waypoints.push_back(a_waypoints.block<3, 1>(0, i));
    return constantSpeedTrajectory(waypoints, a_speed);
}

void appendConstantAcceleration(NativeTrajectory& a_trajectory, const TrajectoryPoint& a_start, const Eigen::Vector3d& a_acceleration,
                                double a_duration, double a_time_step)
{
    if (a_duration <= 0.0)
        return;
    const int nb_of_steps = std::max(1, static_cast<int>(std::ceil(a_duration / a_time_step - 1.0e-9)));
    for (int k = 1; k <= nb_of_steps; ++k)
    {
        const double t = k == nb_of_steps ? a_duration : k * a_time_step;
        a_trajectory.push_back({a_start.position + a_start.velocity * t + 0.5 * a_acceleration * t * t,
                                a_start.velocity + a_acceleration * t, a_start.time + t});
    }
}

void appendBraking(NativeTrajectory& a_trajectory, double a_deceleration)
{
    if (a_trajectory.empty())
        return;
    const TrajectoryPoint last = a_trajectory.back();
    const double speed = last.velocity.norm();
    if (speed < 1.0e-9)
        return;
    const double duration = speed / a_deceleration;
    appendConstantAcceleration(a_trajectory, last, -last.velocity / speed * a_deceleration, duration, duration / 10.0);
    a_trajectory.back().velocity.setZero(); // Exactly at rest
}

double trajectoryLength(const NativeTrajectory& a_trajectory)
{
    double length = 0.0;
    for (size_t i = 1; i < a_trajectory.size(); ++i)
        length += (a_trajectory[i].position - a_trajectory[i - 1].position).norm();
    return length;
}

}; // namespace arena_benchmarks
