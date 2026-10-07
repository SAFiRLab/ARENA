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

#include "testbench/benchmarks/common/baseline_path_scorer.h"

// System
#include <algorithm>
#include <cmath>
#include <stdexcept>


namespace arena_benchmarks
{

BaselinePathScorer::BaselinePathScorer(std::shared_ptr<arena_demos::CostmapMapping> a_costmap_mapping,
                                       const arena_demos::LinedroneNurbsAnalyzerConfig& a_config, int a_degree)
: costmap_mapping_(a_costmap_mapping), config_(a_config), degree_(a_degree)
{
    if (!costmap_mapping_)
        throw std::invalid_argument("BaselinePathScorer: the costmap mapping is null.");

    analyzer_ = std::make_shared<arena_demos::LinedroneNurbsAnalyzer>(costmap_mapping_, config_,
                                                                      std::unordered_map<std::string, arena_core::OrientedBoundingBoxWrapper>{});

    // Same output structure as LinedroneTestNode
    output_.fitness_size_ = 3;
    output_.fitness_array_ = std::vector<double>(output_.fitness_size_, std::numeric_limits<double>::max());
    output_.constraint_size_ = 2;
    output_.constraint_array_ = std::vector<double>(output_.constraint_size_, std::numeric_limits<double>::max());
}

Eigen::MatrixXd BaselinePathScorer::resampleByDistance(const Eigen::MatrixXd& a_waypoints, double a_spacing)
{
    if (a_spacing <= 0.0 || a_waypoints.cols() < 2)
        return a_waypoints;

    std::vector<Eigen::VectorXd> points = {a_waypoints.col(0)};
    double distance_since_last = 0.0;
    for (int i = 1; i < a_waypoints.cols(); ++i)
    {
        Eigen::VectorXd start = a_waypoints.col(i - 1);
        Eigen::VectorXd end = a_waypoints.col(i);
        double segment_length = (end - start).head<3>().norm();
        double position = 0.0; // Distance travelled on the segment

        while (segment_length - position + distance_since_last >= a_spacing)
        {
            position += a_spacing - distance_since_last;
            points.push_back(start + (end - start) * (position / segment_length));
            distance_since_last = 0.0;
        }
        distance_since_last += segment_length - position;
    }

    // The last point is the goal, the previous point is dropped when it is too close to it
    Eigen::VectorXd goal = a_waypoints.col(a_waypoints.cols() - 1);
    if (points.size() > 1 && (points.back() - goal).head<3>().norm() < 0.5 * a_spacing)
        points.pop_back();
    points.push_back(goal);

    Eigen::MatrixXd resampled(a_waypoints.rows(), points.size());
    for (size_t i = 0; i < points.size(); ++i)
        resampled.col(i) = points[i];
    return resampled;
}

Eigen::MatrixXd BaselinePathScorer::densify(const Eigen::MatrixXd& a_points, int a_nb_of_points)
{
    int initial_size = a_points.cols();
    if (initial_size < 2)
        throw std::invalid_argument("BaselinePathScorer::densify => the path must have at least 2 points.");
    if (a_nb_of_points <= initial_size)
        return a_points;

    // Distribute the new points as evenly as possible between the segments
    int segments = initial_size - 1;
    std::vector<int> insert_counts(segments, 0);
    for (int i = 0; i < a_nb_of_points - initial_size; ++i)
        insert_counts[i % segments] += 1;

    Eigen::MatrixXd new_path(a_points.rows(), a_nb_of_points);
    int new_index = 0;
    for (int i = 0; i < segments; ++i)
    {
        Eigen::VectorXd start = a_points.col(i);
        Eigen::VectorXd end = a_points.col(i + 1);
        new_path.col(new_index++) = start;
        for (int j = 1; j <= insert_counts[i]; ++j)
        {
            double alpha = static_cast<double>(j) / (insert_counts[i] + 1);
            new_path.col(new_index++) = (1.0 - alpha) * start + alpha * end;
        }
    }
    new_path.col(new_index++) = a_points.col(initial_size - 1);

    return new_path;
}

Eigen::MatrixXd BaselinePathScorer::toControlPoints(const Eigen::MatrixXd& a_waypoints, double a_speed, double a_ramp_length) const
{
    if (a_waypoints.cols() < 2)
        throw std::invalid_argument("BaselinePathScorer::toControlPoints => the path must have at least 2 points.");

    Eigen::MatrixXd points = densify(a_waypoints.topRows<3>(), degree_ + 1);
    const int nb_of_points = points.cols();

    // Distance along the control polygon, for the speed ramps
    std::vector<double> distance(nb_of_points, 0.0);
    for (int i = 1; i < nb_of_points; ++i)
        distance[i] = distance[i - 1] + (points.col(i) - points.col(i - 1)).norm();
    const double length = distance.back();

    Eigen::MatrixXd control_points(4, nb_of_points);
    control_points.topRows<3>() = points;
    for (int i = 0; i < nb_of_points; ++i)
    {
        double speed = a_speed;
        if (a_ramp_length > 0.0)
        {
            double ramp = std::min({1.0, distance[i] / a_ramp_length, (length - distance[i]) / a_ramp_length});
            // An interior point never stops the robot, the time cost would be infinite
            speed = a_speed * std::max(ramp, 0.1);
        }
        control_points(3, i) = speed;
    }

    // No speed at the start and the goal, like the trajectories of ARENA
    control_points(3, 0) = 0.0;
    control_points(3, nb_of_points - 1) = 0.0;

    return control_points;
}

ScoredPath BaselinePathScorer::evaluate(const Eigen::MatrixXd& a_control_points)
{
    ScoredPath scored;
    scored.control_points = a_control_points;

    if (a_control_points.rows() != 4 || a_control_points.cols() < degree_ + 1)
        return scored;

    // Weights of 1, the weights of the ARENA decision vector are given as ids to the control points and stay at 1
    std::vector<arena_core::ControlPoint<double, 4>> control_points;
    for (int i = 0; i < a_control_points.cols(); ++i)
        control_points.push_back(arena_core::ControlPoint<double, 4>(Eigen::Vector4d(a_control_points.col(i))));

    arena_core::Nurbs<4> nurbs(control_points, static_cast<int>(config_.base_config.sample_size), degree_);
    scored.samples = nurbs.evaluate();
    if (scored.samples.cols() != static_cast<int>(config_.base_config.sample_size) || scored.samples.hasNaN())
        return scored;

    analyzer_->eval(scored.samples, output_);
    scored.fitness = output_.fitness_array_;
    scored.max_acceleration = output_.constraint_array_[0];
    scored.max_occupancy = output_.constraint_array_[1];

    // Same rejection as LinedroneTestNode::linedroneFitness
    const double max_cost = std::numeric_limits<double>::max();
    scored.feasible = scored.max_acceleration <= config_.robot_max_acceleration_ && scored.max_occupancy <= 0.5 &&
                      std::none_of(scored.fitness.begin(), scored.fitness.end(), [max_cost](double f) { return f == max_cost || std::isnan(f); });

    return scored;
}

ScoredPath BaselinePathScorer::score(const Eigen::MatrixXd& a_control_points)
{
    ScoredPath scored = evaluate(a_control_points);
    if (scored.samples.cols() > 0)
        scored.safe = isPathSafe(scored.samples);
    return scored;
}

bool BaselinePathScorer::isPathSafe(const Eigen::MatrixXd& a_samples) const
{
    // Same check as LinedroneTestNode::isPathSafe
    for (int i = 0; i < a_samples.cols() - 1; ++i)
    {
        Eigen::Vector3d start = a_samples.block<3, 1>(0, i);
        Eigen::Vector3d end = a_samples.block<3, 1>(0, i + 1);
        if (costmap_mapping_->isOccupiedRayTracing(start, end))
            return false;
    }
    return true;
}

int BaselinePathScorer::countUnsafeSegments(const Eigen::MatrixXd& a_samples) const
{
    int count = 0;
    for (int i = 0; i < a_samples.cols() - 1; ++i)
    {
        Eigen::Vector3d start = a_samples.block<3, 1>(0, i);
        Eigen::Vector3d end = a_samples.block<3, 1>(0, i + 1);
        if (costmap_mapping_->isOccupiedRayTracing(start, end))
            count++;
    }
    return count;
}

double BaselinePathScorer::steadyStatePower(const Eigen::Vector3d& a_direction) const
{
    Eigen::Vector3d unit = a_direction.normalized();
    return analyzer_->steadyStatePower(unit, unit.z() < 0.0);
}

}; // namespace arena_benchmarks
