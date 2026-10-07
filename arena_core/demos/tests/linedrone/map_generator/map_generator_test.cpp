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

#include <gtest/gtest.h>

#include "linedrone/map_generator/map_generator.hpp"

#include <cmath>
#include <sstream>

using namespace arena_demos::map_generator;

namespace
{

// Small map so the brute force checks stay fast
MapGeneratorConfig makeSmallConfig(uint64_t seed)
{
    MapGeneratorConfig config;
    config.seed = seed;
    config.resolution = 0.5;
    config.origin = Eigen::Vector3d(-10.0, 5.0, 0.0);
    config.size = Eigen::Vector3d(30.0, 30.0, 15.0);
    config.density = 3.0;
    config.volume_mean = 10.0;
    config.volume_max = 60.0;
    config.min_clearance = 1.0;
    return config;
}

std::string serialize(const octomap::OcTree& tree)
{
    std::stringstream stream;
    tree.writeBinaryConst(stream);
    return stream.str();
}

std::vector<Eigen::Vector3d> occupiedVoxels(const octomap::OcTree& tree)
{
    std::vector<Eigen::Vector3d> voxels;
    const double resolution = tree.getResolution();
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
    {
        if (!tree.isNodeOccupied(*it))
            continue;

        // Pruned leaves cover several voxels
        const int n = static_cast<int>(std::lround(it.getSize() / resolution));
        const Eigen::Vector3d corner = Eigen::Vector3d(it.getX(), it.getY(), it.getZ()) - Eigen::Vector3d::Constant(it.getSize() / 2.0);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                for (int k = 0; k < n; ++k)
                    voxels.push_back(corner + resolution * Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5));
    }
    return voxels;
}

} // namespace


TEST(MapGeneratorRandom, SameSeedSameSequence)
{
    Random a(123), b(123), c(124);
    bool differs = false;
    for (int i = 0; i < 100; ++i)
    {
        double value = a.uniform();
        EXPECT_EQ(value, b.uniform());
        EXPECT_GE(value, 0.0);
        EXPECT_LT(value, 1.0);
        differs |= value != c.uniform();
    }
    EXPECT_TRUE(differs);
}

TEST(MapGeneratorRandom, LogNormalMean)
{
    Random rng(7);
    double sum = 0.0;
    const int n = 200000;
    for (int i = 0; i < n; ++i)
        sum += rng.logNormal(50.0, 0.5);
    EXPECT_NEAR(sum / n, 50.0, 0.5);
}

TEST(MapGeneratorShapes, RequestedVolumeAndContainment)
{
    Random rng(1);
    const double volume = 40.0;
    const double step = 0.1;

    for (ShapeType type : allShapeTypes())
    {
        for (int sample = 0; sample < 5; ++sample)
        {
            std::unique_ptr<Shape> shape = makeRandomShape(type, volume, 3.0, rng);
            EXPECT_NEAR(shape->getVolume(), volume, 1e-9) << shapeTypeToString(type);

            // Volume of contains() on a fine grid matches the analytic volume
            shape->setRotation(rng.rotation(OrientationMode::Full));
            shape->setCenter(Eigen::Vector3d(1.0, -2.0, 3.0));
            Eigen::AlignedBox3d box = shape->getWorldBoundingBox();

            size_t inside = 0;
            for (double x = box.min().x() + step / 2; x < box.max().x(); x += step)
                for (double y = box.min().y() + step / 2; y < box.max().y(); y += step)
                    for (double z = box.min().z() + step / 2; z < box.max().z(); z += step)
                        inside += shape->contains(Eigen::Vector3d(x, y, z));

            EXPECT_NEAR(inside * step * step * step, volume, 0.05 * volume) << shapeTypeToString(type);
        }
    }
}

TEST(MapGenerator, SameSeedSameMap)
{
    MapGenerator a(makeSmallConfig(42)), b(makeSmallConfig(42)), c(makeSmallConfig(43));
    a.generate();
    b.generate();
    c.generate();

    EXPECT_GT(a.getStats().nb_of_placed_obstacles, 0);
    EXPECT_EQ(serialize(*a.getOctree()), serialize(*b.getOctree()));
    EXPECT_NE(serialize(*a.getOctree()), serialize(*c.getOctree()));

    // Generating again gives the same map
    std::string first = serialize(*a.getOctree());
    a.generate();
    EXPECT_EQ(first, serialize(*a.getOctree()));
}

TEST(MapGenerator, HigherDensityKeepsExistingObstacles)
{
    MapGeneratorConfig sparse_config = makeSmallConfig(5);
    MapGeneratorConfig dense_config = makeSmallConfig(5);
    dense_config.density = 2.0 * sparse_config.density;

    MapGenerator sparse(sparse_config), dense(dense_config);
    sparse.generate();
    dense.generate();

    ASSERT_EQ(sparse.getStats().nb_of_failed_obstacles, 0);
    ASSERT_GT(dense.getObstacles().size(), sparse.getObstacles().size());
    for (size_t i = 0; i < sparse.getObstacles().size(); ++i)
    {
        EXPECT_EQ(sparse.getObstacles()[i].shape->getCenter(), dense.getObstacles()[i].shape->getCenter());
        EXPECT_EQ(sparse.getObstacles()[i].shape->getType(), dense.getObstacles()[i].shape->getType());
    }
}

TEST(MapGenerator, DensityAndBounds)
{
    MapGeneratorConfig config = makeSmallConfig(3);
    MapGenerator generator(config);
    generator.generate();

    // 3 obstacles per 100 m^2 on 30 m x 30 m
    EXPECT_EQ(generator.getStats().nb_of_requested_obstacles, 27);
    EXPECT_EQ(generator.getStats().nb_of_placed_obstacles + generator.getStats().nb_of_failed_obstacles, 27);

    // Every occupied voxel is inside the map
    std::vector<Eigen::Vector3d> voxels = occupiedVoxels(*generator.getOctree());
    EXPECT_EQ(voxels.size(), generator.getStats().nb_of_occupied_voxels);
    for (const Eigen::Vector3d& voxel : voxels)
    {
        EXPECT_TRUE((voxel.array() >= config.origin.array()).all());
        EXPECT_TRUE((voxel.array() <= config.getMaxCorner().array()).all());
    }

    // The leaves span the whole map (mark_bounds)
    Eigen::Vector3d low = Eigen::Vector3d::Constant(1e9), high = Eigen::Vector3d::Constant(-1e9);
    for (auto it = generator.getOctree()->begin_leafs(); it != generator.getOctree()->end_leafs(); ++it)
    {
        Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
        Eigen::Vector3d half = Eigen::Vector3d::Constant(it.getSize() / 2.0);
        low = low.cwiseMin(center - half);
        high = high.cwiseMax(center + half);
    }
    EXPECT_TRUE(low.isApprox(config.origin, 1e-6));
    EXPECT_TRUE(high.isApprox(config.getMaxCorner(), 1e-6));
}

TEST(MapGenerator, FreeZonesAndClearance)
{
    MapGeneratorConfig config = makeSmallConfig(11);
    config.free_zones.push_back({Eigen::Vector3d(5.0, 20.0, 7.0), 6.0});
    config.testbench_enabled = true;
    config.testbench_start = Eigen::Vector3d(-8.0, 7.0, 2.0);
    config.testbench_goal = Eigen::Vector3d(18.0, 33.0, 13.0);
    config.testbench_clearance = 3.0;

    MapGenerator generator(config);
    generator.generate();
    std::vector<Eigen::Vector3d> voxels = occupiedVoxels(*generator.getOctree());
    ASSERT_GT(generator.getObstacles().size(), 10u);
    ASSERT_GT(voxels.size(), 1000u);

    // No voxel cube touches a free zone
    const double half_diagonal = 0.5 * std::sqrt(3.0) * config.resolution;
    for (const FreeZone& zone : config.getAllFreeZones())
        for (const Eigen::Vector3d& voxel : voxels)
            EXPECT_GE((voxel - zone.center).norm(), zone.radius + half_diagonal);

    // Voxels of two different obstacles leave a gap of at least min_clearance (center distance >= clearance + resolution)
    std::vector<int> owner(voxels.size(), -1);
    const auto& obstacles = generator.getObstacles();
    for (size_t v = 0; v < voxels.size(); ++v)
    {
        for (size_t o = 0; o < obstacles.size() && owner[v] < 0; ++o)
        {
            if (obstacles[o].shape->contains(voxels[v]))
                owner[v] = static_cast<int>(o);
        }
        ASSERT_GE(owner[v], 0);
    }

    const double min_distance = config.min_clearance + config.resolution - 1e-6;
    for (size_t i = 0; i < voxels.size(); ++i)
    {
        for (size_t j = i + 1; j < voxels.size(); ++j)
        {
            if (owner[i] != owner[j])
            {
                ASSERT_GE((voxels[i] - voxels[j]).norm(), min_distance);
            }
        }
    }
}

TEST(MapGeneratorConfig, YamlRoundTripAndValidation)
{
    YAML::Node node = YAML::Load(
        "seed: 9\n"
        "map: {size: {x: 50, y: 40}}\n"
        "obstacles: {density: 2.5, shapes: {box: 2.0, torus: 1.0}}\n");
    MapGeneratorConfig config = MapGeneratorConfig::fromYaml(node);

    EXPECT_EQ(config.seed, 9u);
    EXPECT_EQ(config.size, Eigen::Vector3d(50.0, 40.0, 40.0)); // z keeps its default
    EXPECT_EQ(config.getTargetObstacleCount(), 50);
    EXPECT_EQ(config.getMapName(), "cluttered_map_seed9");
    for (const auto& [type, weight] : config.shape_weights)
        EXPECT_EQ(weight, type == ShapeType::Box ? 2.0 : (type == ShapeType::Torus ? 1.0 : 0.0));

    MapGeneratorConfig reloaded = MapGeneratorConfig::fromYaml(config.toYaml());
    EXPECT_EQ(reloaded.size, config.size);
    EXPECT_EQ(reloaded.density, config.density);
    EXPECT_EQ(reloaded.shape_weights, config.shape_weights);

    EXPECT_THROW(MapGeneratorConfig::fromYaml(YAML::Load("obstacles: {shapes: {hexagon: 1.0}}")), std::invalid_argument);
    EXPECT_THROW(MapGeneratorConfig::fromYaml(YAML::Load("obstacles: {orientation: diagonal}")), std::invalid_argument);
    EXPECT_THROW(MapGeneratorConfig::fromYaml(YAML::Load("map: {resolution: -1}")), std::invalid_argument);
    EXPECT_THROW(MapGeneratorConfig::fromYaml(YAML::Load("testbench: {enabled: true, start: {x: -5}}")), std::invalid_argument);
}
