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
#include "linedrone/map_generator/map_generator_config.hpp"
#include "linedrone/map_generator/shapes.hpp"

// System
#include <memory>
#include <string>
#include <vector>

// Octomap
#include <octomap/octomap.h>


namespace arena_demos
{

namespace map_generator
{

/**
 * @brief Obstacle placed in the generated map.
 */
struct Obstacle
{
    std::unique_ptr<Shape> shape;
    bool is_ground_anchored = false;
    size_t nb_of_voxels = 0;
}; // struct Obstacle


/**
 * @brief Statistics of the last generated map.
 */
struct GenerationStats
{
    int nb_of_requested_obstacles = 0;
    int nb_of_placed_obstacles = 0;
    int nb_of_failed_obstacles = 0;    // Obstacles that couldn't be placed after max_placement_attempts
    size_t nb_of_occupied_voxels = 0;
    double obstacles_volume = 0.0;     // Sum of the analytic volumes (m^3)
    double occupied_volume = 0.0;      // Volume of the occupied voxels (m^3)
    double occupancy_ratio = 0.0;      // occupied_volume / map volume
    double achieved_density = 0.0;     // Placed obstacles per density_reference_area
    double generation_time = 0.0;      // s
}; // struct GenerationStats


/**
 * @brief Generates random cluttered 3D maps as octomaps.
 *
 * Obstacles are closed volumes (see ShapeType) of random type, volume, proportions, orientation and
 * position, voxelized as solid (fully occupied) volumes in an octomap::OcTree. Everything is drawn from
 * random streams derived from the seed, so the same configuration always gives the same map.
 *
 * Every obstacle uses its own random stream (derived from the seed and the obstacle index), so obstacle i
 * is the same whatever the number of obstacles: increasing the density keeps the existing obstacles and
 * adds new ones (as long as the new obstacles don't take the place of the old ones).
 */
class MapGenerator
{
public:

    explicit MapGenerator(const MapGeneratorConfig& config);

    /** @brief Generate the map. Calling it again regenerates the same map. */
    void generate();

    /** @brief Save the map as an octomap binary file (.bt). */
    bool saveBinary(const std::string& file_path) const;

    /** @brief Save the configuration, the statistics and the list of obstacles in a YAML file. */
    void saveMetadata(const std::string& file_path, const std::string& bt_file_path) const;

    /** @brief Save the start and the goal in the format read by the testbench (config/linedrone/testbench_configs). */
    void saveTestbenchConfig(const std::string& file_path) const;

    /** @brief Human readable summary of the last generation. */
    std::string getSummary() const;

    // Getters
    const MapGeneratorConfig& getConfig() const { return config_; }
    std::shared_ptr<octomap::OcTree> getOctree() const { return octree_; }
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }
    const GenerationStats& getStats() const { return stats_; }

private:

    // User-defined methods
    std::unique_ptr<Shape> sampleObstacle(Random& rng, bool is_ground_anchored) const;
    std::vector<octomap::OcTreeKey> voxelize(const Shape& shape) const;
    bool isPlacementValid(const std::vector<octomap::OcTreeKey>& voxels) const;
    bool isInsideMap(const octomap::point3d& point) const;
    void buildClearanceStencil();
    void markBounds();

    // User-defined attributes
    MapGeneratorConfig config_;
    std::shared_ptr<octomap::OcTree> octree_;
    std::vector<Obstacle> obstacles_;
    octomap::KeySet occupied_keys_;
    std::vector<FreeZone> free_zones_;
    std::vector<Eigen::Vector3i> clearance_stencil_;   // Key offsets closer than min_clearance
    GenerationStats stats_;

}; // class MapGenerator

}; // namespace map_generator

}; // namespace arena_demos
