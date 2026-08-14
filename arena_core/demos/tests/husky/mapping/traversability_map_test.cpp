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

#include "husky/mapping/traversability_map.hpp"

#include <grid_map_core/iterators/LineIterator.hpp>

#include <cmath>
#include <limits>
#include <set>

// Mirrors the physical constants baked into TraversabilityMap's algorithms
// (traversability_map.cpp). These are not exposed via the public API, so the
// tests below re-derive expected values from them at runtime rather than
// hardcoding pre-computed numbers, keeping the test self-checking against the
// documented behavior rather than against opaque literals.
namespace
{
constexpr float kHit = 0.85f;
constexpr float kMiss = -0.40f;
constexpr float kMeasurementVariance = 0.01f;
constexpr float kProcessNoise = 1e-4f;
constexpr float kMaxConsecutiveRejections = 3.0f;
constexpr double kInflationRadius = 9.0;

int64_t packIndex(const grid_map::Index &index)
{
    return (static_cast<int64_t>(index.x()) << 32) | static_cast<uint32_t>(index.y());
}

float sigmoid(float log_odds)
{
    return 1.f / (1.f + std::exp(-log_odds));
}

grid_map::Index indexAt(const grid_map::GridMap &map, double x, double y)
{
    grid_map::Index idx;
    EXPECT_TRUE(map.getIndex(grid_map::Position(x, y), idx)) << "expected (" << x << ", " << y << ") to be inside the map";
    return idx;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> makeCloud(const std::vector<std::array<float, 3>> &points)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto &p : points)
        cloud->points.emplace_back(p[0], p[1], p[2]);
    return cloud;
}

// Feeds one simulated LiDAR scan into the map. This is the one place that
// knows TraversabilityMap::updateMap()'s calling convention, so it is the
// only line that needs editing when that signature changes.
void feedScan(arena_demos::TraversabilityMap &map,
              const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &ground,
              const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &non_ground)
{
    map.updateMap(ground, non_ground);
}

// Replicates updateElevationAtIndex's Kalman-style recursion exactly (same
// float operations, same order) so tests can compute an expected trajectory
// at runtime instead of hand-deriving/rounding intermediate values.
struct ReferenceElevationCell
{
    float mean = std::numeric_limits<float>::quiet_NaN();
    float variance = std::numeric_limits<float>::quiet_NaN();
    float num_measurements = 0.0f;
    float reject_streak = 0.0f;

    void update(float z)
    {
        if (std::isnan(mean))
        {
            mean = z;
            variance = 0.1f;
            num_measurements = 1.0f;
            reject_streak = 0.0f;
            return;
        }

        const float local_variance = variance + kProcessNoise;
        const float innovation = z - mean;
        const float S = local_variance + kMeasurementVariance;

        if (std::abs(innovation) > 3.0f * std::sqrt(S))
        {
            reject_streak += 1.0f;
            if (reject_streak < kMaxConsecutiveRejections)
                return;

            mean = z;
            variance = 0.1f;
            num_measurements = 1.0f;
            reject_streak = 0.0f;
            return;
        }

        reject_streak = 0.0f;
        const float K = local_variance / S;
        mean = mean + K * innovation;
        variance = local_variance * (1.0f - K);
        num_measurements += 1.0f;
    }
};

// TraversabilityMap holds a std::mutex member, so it is neither copyable nor
// movable -- hand it out via shared_ptr rather than by value.
std::shared_ptr<arena_demos::TraversabilityMap> makeMap()
{
    auto map = std::make_shared<arena_demos::TraversabilityMap>(arena_demos::RobotBoundingBox{1.0, 1.0}, arena_demos::TraversabilityMapConfig());
    map->moveLocalMap(Eigen::Vector2d(0.0, 0.0));
    return map;
}
} // namespace

TEST(TraversabilityMapRegression, FreshGroundCellSingleMeasurement)
{
    auto map = makeMap();
    auto ground = makeCloud({{7.5f, 2.1f, 1.234f}});
    feedScan(*map, ground, nullptr);

    auto global = map->getGlobalMap();
    const grid_map::Index idx = indexAt(*global, 7.5, 2.1);

    EXPECT_FLOAT_EQ(global->at("elevation_mean", idx), 1.234f);
    EXPECT_FLOAT_EQ(global->at("elevation_variance", idx), 0.1f);
    EXPECT_FLOAT_EQ(global->at("elevation_num_measurements", idx), 1.0f);
    EXPECT_FLOAT_EQ(global->at("elevation_reject_streak", idx), 0.0f);

    EXPECT_FLOAT_EQ(global->at("occupancy_logodds", idx), 0.0f);
    EXPECT_FLOAT_EQ(global->at("occupancy_probability", idx), 0.0f);
    EXPECT_FLOAT_EQ(global->at("inflated_occupancy", idx), 0.0f);

    EXPECT_FALSE(global->isValid(idx, "step"));
    EXPECT_FALSE(global->isValid(idx, "slope"));

    // Immature/unknown terrain: cost falls back to unknown_cost (1.0) plus
    // the (zero) inflated-occupancy contribution.
    EXPECT_FLOAT_EQ(global->at("cost", idx), 1.0f);
}

TEST(TraversabilityMapRegression, GroundElevationKalmanAndRejectStreakReanchor)
{
    auto map = makeMap();

    const std::vector<float> z_sequence = {1.0f, 1.05f, 0.98f, 5.0f, 5.0f, 5.0f};
    std::vector<std::array<float, 3>> points;
    for (float z : z_sequence)
        points.push_back({12.3f, -4.5f, z});

    ReferenceElevationCell reference;
    for (float z : z_sequence)
        reference.update(z);

    feedScan(*map, makeCloud(points), nullptr);

    auto global = map->getGlobalMap();
    const grid_map::Index idx = indexAt(*global, 12.3, -4.5);

    // The reject-streak logic must have re-anchored the cell on the final
    // measurement rather than staying rejected or averaging it in.
    EXPECT_NEAR(global->at("elevation_mean", idx), reference.mean, 1e-4f);
    EXPECT_NEAR(global->at("elevation_variance", idx), reference.variance, 1e-4f);
    EXPECT_FLOAT_EQ(global->at("elevation_num_measurements", idx), reference.num_measurements);
    EXPECT_FLOAT_EQ(global->at("elevation_reject_streak", idx), reference.reject_streak);
    EXPECT_FLOAT_EQ(reference.mean, 5.0f);
}

TEST(TraversabilityMapRegression, NonGroundSingleRayMissAndEndpointHit)
{
    auto map = makeMap();
    auto non_ground = makeCloud({{6.0f, 1.5f, 2.0f}});
    feedScan(*map, nullptr, non_ground);

    auto global = map->getGlobalMap();
    const grid_map::Position start(0.0, 0.0);
    const grid_map::Position end(6.0, 1.5);
    const grid_map::Index endpoint_idx = indexAt(*global, 6.0, 1.5);

    grid_map::Index last_idx;
    bool any = false;
    for (grid_map::LineIterator it(*global, start, end); !it.isPastEnd(); ++it)
    {
        last_idx = *it;
        any = true;

        const bool is_endpoint = (last_idx == endpoint_idx).all();
        const float expected_log_odds = is_endpoint ? (kMiss + kHit) : kMiss;
        EXPECT_NEAR(global->at("occupancy_logodds", last_idx), expected_log_odds, 1e-5f);
        EXPECT_NEAR(global->at("occupancy_probability", last_idx), sigmoid(expected_log_odds), 1e-5f);
    }
    ASSERT_TRUE(any);
    EXPECT_TRUE((last_idx == endpoint_idx).all()) << "LineIterator did not terminate on the ray endpoint's own cell";
}

TEST(TraversabilityMapRegression, NonGroundOverlappingRaysAccumulateLogOdds)
{
    auto map = makeMap();
    // p2 is exactly p1 scaled by 2 from the (0,0) robot pose, so ray 2's line
    // shares ray 1's entire interior path (only the endpoints differ).
    auto non_ground = makeCloud({{8.0f, 2.0f, 1.0f}, {16.0f, 4.0f, 1.0f}});
    feedScan(*map, nullptr, non_ground);

    auto global = map->getGlobalMap();
    const grid_map::Position start(0.0, 0.0);
    const grid_map::Index end1_idx = indexAt(*global, 8.0, 2.0);
    const grid_map::Index end2_idx = indexAt(*global, 16.0, 4.0);

    std::set<int64_t> ray1_interior;
    for (grid_map::LineIterator it(*global, start, grid_map::Position(8.0, 2.0)); !it.isPastEnd(); ++it)
        if (!(*it == end1_idx).all())
            ray1_interior.insert(packIndex(*it));

    int64_t shared_key = 0;
    bool found_shared = false;
    for (grid_map::LineIterator it(*global, start, grid_map::Position(16.0, 4.0)); !it.isPastEnd(); ++it)
    {
        if ((*it == end2_idx).all())
            continue;
        const int64_t key = packIndex(*it);
        if (ray1_interior.count(key) != 0)
        {
            shared_key = key;
            found_shared = true;
            break;
        }
    }
    ASSERT_TRUE(found_shared) << "test setup did not produce a cell shared by both ray interiors";

    grid_map::Index shared_idx(static_cast<int>(shared_key >> 32), static_cast<int>(shared_key & 0xffffffff));

    // The shared cell is interior to both rays (never an endpoint of
    // either), so it must have accumulated exactly two misses.
    EXPECT_NEAR(global->at("occupancy_logodds", shared_idx), 2.0f * kMiss, 1e-5f);
    EXPECT_NEAR(global->at("occupancy_probability", shared_idx), sigmoid(2.0f * kMiss), 1e-5f);
}

TEST(TraversabilityMapRegression, GroundPointOverridesImmatureOccupancy)
{
    auto map = makeMap();
    const float x = 9.0f, y = -2.4f;

    std::vector<std::array<float, 3>> seed_points(6, std::array<float, 3>{x, y, 0.0f});
    feedScan(*map, nullptr, makeCloud(seed_points));

    auto global = map->getGlobalMap();
    const grid_map::Index idx = indexAt(*global, x, y);

    const float seeded_log_odds = 6.0f * (kMiss + kHit);
    ASSERT_GT(global->at("occupancy_probability", idx), 0.9f)
        << "test setup failed to seed occupancy above the reconciliation threshold";
    EXPECT_NEAR(global->at("occupancy_logodds", idx), seeded_log_odds, 1e-4f);

    // Elevation is still immature (6 measurements, well under the reliable
    // threshold), so the ground return must clear the occupancy belief.
    feedScan(*map, makeCloud({{x, y, 0.5f}}), nullptr);

    EXPECT_FLOAT_EQ(global->at("occupancy_logodds", idx), 0.0f);
    EXPECT_FLOAT_EQ(global->at("occupancy_probability", idx), 0.0f);
    EXPECT_FLOAT_EQ(global->at("inflated_occupancy", idx), 0.0f);
}

TEST(TraversabilityMapRegression, MapAndRobotBoundaryPoints)
{
    auto map = makeMap();
    auto global = map->getGlobalMap();

    const grid_map::Length length = global->getLength();
    const grid_map::Position center = global->getPosition();
    const double half_x = length.x() / 2.0;

    const float x_inside_map = static_cast<float>(center.x() + half_x - 0.05);
    const float x_outside_map = static_cast<float>(center.x() + half_x + 0.5);

    auto ground = makeCloud({
        {x_inside_map, static_cast<float>(center.y()), 1.0f},   // just inside the global map edge
        {x_outside_map, static_cast<float>(center.y()), 2.0f},  // just outside the global map edge
        {0.0f, 0.4f, 3.0f},                                     // inside the 1x1 robot bounding box
        {0.0f, 0.6f, 4.0f},                                     // just outside the robot bounding box
    });
    feedScan(*map, ground, nullptr);

    grid_map::Index idx;
    ASSERT_TRUE(global->getIndex(grid_map::Position(x_inside_map, center.y()), idx));
    EXPECT_FLOAT_EQ(global->at("elevation_mean", idx), 1.0f);

    EXPECT_FALSE(global->getIndex(grid_map::Position(x_outside_map, center.y()), idx));

    ASSERT_TRUE(global->getIndex(grid_map::Position(0.0, 0.4), idx));
    EXPECT_FALSE(global->isValid(idx, "elevation_mean")) << "point inside the robot bounding box should have been skipped";

    ASSERT_TRUE(global->getIndex(grid_map::Position(0.0, 0.6), idx));
    EXPECT_FLOAT_EQ(global->at("elevation_mean", idx), 4.0f);
}

TEST(TraversabilityMapRegression, InflationWavefrontFalloff)
{
    auto map = makeMap();
    const float x = 10.0f, y = 3.0f;

    std::vector<std::array<float, 3>> seed_points(6, std::array<float, 3>{x, y, 1.0f});
    feedScan(*map, nullptr, makeCloud(seed_points));

    auto global = map->getGlobalMap();
    const double resolution = global->getResolution();
    const grid_map::Index seed_idx = indexAt(*global, x, y);
    const grid_map::Position seed_pos = [&] { grid_map::Position p; global->getPosition(seed_idx, p); return p; }();

    ASSERT_GT(global->at("occupancy_probability", seed_idx), 0.9f)
        << "test setup failed to seed an obstacle for the inflation wavefront";

    const double robot_radius = 0.5 * std::sqrt(1.0 * 1.0 + 1.0 * 1.0); // RobotBoundingBox{1.0, 1.0}

    auto distanceFromSeed = [&](double offset_cells) -> std::pair<grid_map::Index, double>
    {
        grid_map::Index idx;
        EXPECT_TRUE(global->getIndex(grid_map::Position(seed_pos.x() + offset_cells * resolution, seed_pos.y()), idx));
        const double dx = (idx.x() - seed_idx.x()) * resolution;
        const double dy = (idx.y() - seed_idx.y()) * resolution;
        return {idx, std::sqrt(dx * dx + dy * dy)};
    };

    EXPECT_FLOAT_EQ(global->at("inflated_occupancy", seed_idx), 1.0f);

    // Well within the robot's own footprint radius: fully lethal.
    auto [flat_idx, flat_distance] = distanceFromSeed(2.0);
    ASSERT_LE(flat_distance, robot_radius);
    EXPECT_FLOAT_EQ(global->at("inflated_occupancy", flat_idx), 1.0f);

    // Beyond the footprint but within the inflation radius: linear falloff.
    auto [falloff_idx, falloff_distance] = distanceFromSeed(5.0);
    ASSERT_GT(falloff_distance, robot_radius);
    ASSERT_LT(falloff_distance, kInflationRadius);
    const float expected_decay = static_cast<float>(1.0 - (falloff_distance - robot_radius) / (kInflationRadius - robot_radius));
    EXPECT_NEAR(global->at("inflated_occupancy", falloff_idx), expected_decay, 1e-3f);

    // Beyond the inflation radius: never reached by the wavefront.
    auto [far_idx, far_distance] = distanceFromSeed(31.0);
    ASSERT_GT(far_distance, kInflationRadius);
    EXPECT_FLOAT_EQ(global->at("inflated_occupancy", far_idx), 0.0f);
}
