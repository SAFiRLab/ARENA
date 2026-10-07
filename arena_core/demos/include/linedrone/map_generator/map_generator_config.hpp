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
#include "linedrone/map_generator/random.hpp"
#include "linedrone/map_generator/shapes.hpp"

// System
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

// External Libraries
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>


namespace arena_demos
{

namespace map_generator
{

/**
 * @brief Spherical region kept free of obstacles (e.g. the start and the goal of a planning problem).
 */
struct FreeZone
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double radius = 0.0;
}; // struct FreeZone


/**
 * @brief Parameters of the random map generator.
 *
 * See config/linedrone/map_generator/map_generator_params.yaml for the description of every parameter.
 */
struct MapGeneratorConfig
{
    // General
    std::string name = "cluttered_map";
    uint64_t seed = 0;
    std::string output_directory = "/home/dev_ws/src/arena_core/demos/ressources/generated_map";

    // Map
    double resolution = 0.5;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d size = Eigen::Vector3d(100.0, 100.0, 40.0);
    bool mark_bounds = true;

    // Obstacles
    double density = 1.0;                   // Number of obstacles per density_reference_area of ground
    double density_reference_area = 100.0;  // m^2 (10 m x 10 m)
    double volume_mean = 60.0;
    double volume_std_ratio = 0.5;
    double volume_min = 2.0;
    double volume_max = 500.0;
    double max_aspect_ratio = 3.0;
    double ground_anchored_ratio = 0.3;
    OrientationMode orientation_mode = OrientationMode::Full;
    double min_clearance = 1.0;
    int max_placement_attempts = 100;
    std::vector<std::pair<ShapeType, double>> shape_weights;  // Ordered as allShapeTypes()

    // Regions kept free of obstacles
    std::vector<FreeZone> free_zones;

    // Testbench configuration written next to the map
    bool testbench_enabled = false;
    std::string testbench_config_directory = "/home/dev_ws/src/arena_core/demos/config/linedrone/testbench_configs";
    Eigen::Vector3d testbench_start = Eigen::Vector3d::Zero();
    Eigen::Vector3d testbench_goal = Eigen::Vector3d::Zero();
    double testbench_clearance = 5.0;

    MapGeneratorConfig();

    /**
     * @brief Load the configuration from a YAML file (parameters under the "map_generator" key).
     *
     * Missing parameters keep their default value. Throws std::runtime_error if the file can't be read
     * and std::invalid_argument if a parameter is invalid.
     */
    static MapGeneratorConfig fromYamlFile(const std::string& file_path);

    /** @brief Load the configuration from the "map_generator" YAML node. */
    static MapGeneratorConfig fromYaml(const YAML::Node& node);

    /** @brief Configuration as a YAML node, in the same format as the one read by fromYaml(). */
    YAML::Node toYaml() const;

    /** @brief Throws std::invalid_argument if a parameter is invalid. */
    void validate() const;

    /** @brief Number of obstacles to place, given the density and the ground area of the map. */
    int getTargetObstacleCount() const;

    /** @brief Name of the generated map: <name>_seed<seed>. */
    std::string getMapName() const;

    /** @brief Upper corner of the map. */
    Eigen::Vector3d getMaxCorner() const { return origin + size; }

    /** @brief Free zones from the config plus the testbench start and goal when the testbench is enabled. */
    std::vector<FreeZone> getAllFreeZones() const;

}; // struct MapGeneratorConfig


/**
 * @brief Double as a YAML scalar with 6 significant digits.
 *
 * YAML::Node stores doubles with 17 digits (0.6 becomes 0.59999999999999998), which makes the saved files hard to read.
 */
YAML::Node toYamlNumber(double value);

}; // namespace map_generator

}; // namespace arena_demos
