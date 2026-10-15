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
#include "linedrone/costmap_mapping.h"
#include "testbench/benchmarks/moar_3d/moar_graph_search.h"
#include "testbench/benchmarks/common/native_trajectory.h"

// System
#include <algorithm>
#include <functional>
#include <memory>
#include <string>


namespace arena_benchmarks
{

/**
 * @brief Speeds of the MOAR implementation (moar_3d.speeds.*), relative to its horizontal cruising speed (0.6 m/s):
 * minimum 0.5, up 0.58, diagonal up 0.57, diagonal down 0.58, down 0.57 m/s, maximum speed reached 4 m away from the
 * obstacles. The horizontal cruising speed is the maximum speed of the robot.
 *
 * @param a_v_max Maximum speed of the robot.
 * @param a_param Value of a parameter, with its default value.
 */
inline MoarSpeeds moarSpeeds(double a_v_max, const std::function<double(const std::string&, double)>& a_param)
{
    MoarSpeeds speeds;
    speeds.v_max = a_v_max;
    speeds.horizontal = a_v_max;
    speeds.v_min = a_v_max * a_param("moar_3d.speeds.min_ratio", 0.5 / 0.6);
    speeds.up = a_v_max * a_param("moar_3d.speeds.up_ratio", 0.58 / 0.6);
    speeds.diagonal_up = a_v_max * a_param("moar_3d.speeds.diagonal_up_ratio", 0.57 / 0.6);
    speeds.diagonal_down = a_v_max * a_param("moar_3d.speeds.diagonal_down_ratio", 0.58 / 0.6);
    speeds.down = a_v_max * a_param("moar_3d.speeds.down_ratio", 0.57 / 0.6);
    speeds.d_min = a_param("moar_3d.speeds.d_min", a_param("moar_3d.sdf_min_distance", 2.0));
    speeds.d_max = a_param("moar_3d.speeds.d_max", 4.0);
    return speeds;
}

/**
 * @brief Cells of the MOAR graph: valid if clear of the inflated octomap within a_clearance, safety cost = collision cost
 * C(d_obs) of the SDF octomap (Eq. 3), obstacle distance computed back from it.
 */
inline MoarGraphSearch::CellFn moarCellFn(std::shared_ptr<arena_demos::CostmapMapping> a_mapping,
                                          std::shared_ptr<arena_demos::CostSDFCapability> a_sdf, double a_clearance,
                                          double a_sdf_d_min, double a_sdf_d_max)
{
    const double alpha = a_sdf_d_min / (a_sdf_d_max - a_sdf_d_min);
    return [a_mapping, a_sdf, a_clearance, alpha, a_sdf_d_min, a_sdf_d_max](const Eigen::Vector3d& p)
    {
        MoarGraphSearch::CellInfo info;
        info.valid = a_mapping->isClearance(p, a_clearance);
        if (!info.valid)
            return info;
        info.safety_cost = std::clamp(a_sdf->getCollisionCost(p), 0.0, 1.0);
        if (info.safety_cost >= 1.0)
            info.obstacle_distance = a_sdf_d_min;
        else if (info.safety_cost <= 0.0)
            info.obstacle_distance = a_sdf_d_max;
        else
            info.obstacle_distance = a_sdf_d_max / (1.0 + info.safety_cost / alpha);
        return info;
    };
}

/**
 * @brief Trajectory of a MOAR path as planned: the cell centers (start and goal at the ends), every segment flown at the
 * speed planned for it (MoarPathNode::speed of its end), instantaneous changes of speed, from and to rest. A static action
 * inside the path (speed 0) is a stop in its cell: the speed drops to 0 and the next segment starts from rest.
 */
inline NativeTrajectory moarTrajectory(const std::vector<MoarPathNode>& a_path)
{
    NativeTrajectory trajectory;
    if (a_path.empty())
        return trajectory;
    trajectory.push_back({a_path.front().position, Eigen::Vector3d::Zero(), 0.0});
    for (size_t i = 1; i < a_path.size(); ++i)
    {
        const TrajectoryPoint previous = trajectory.back();
        const Eigen::Vector3d displacement = a_path[i].position - previous.position;
        const double length = displacement.norm();
        if (a_path[i].speed <= 0.0 || length < 1.0e-9)
        {
            // Static action: stop in the cell (instantaneous)
            if (previous.velocity.norm() > 0.0)
                trajectory.push_back({previous.position, Eigen::Vector3d::Zero(), previous.time});
            continue;
        }
        const Eigen::Vector3d velocity = displacement / length * a_path[i].speed;
        trajectory.push_back({previous.position, velocity, previous.time}); // Instantaneous change of velocity
        trajectory.push_back({a_path[i].position, velocity, previous.time + length / a_path[i].speed});
    }
    trajectory.push_back({trajectory.back().position, Eigen::Vector3d::Zero(), trajectory.back().time});
    return trajectory;
}

}; // namespace arena_benchmarks
