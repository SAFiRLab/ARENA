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
#include "linedrone/map_generator/map_generator_config.hpp"

// System
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>


namespace arena_demos
{

namespace map_generator
{

namespace
{

Eigen::Vector3d readVector3(const YAML::Node& node, const Eigen::Vector3d& default_value)
{
    Eigen::Vector3d value = default_value;
    if (!node)
        return value;

    if (node["x"]) value.x() = node["x"].as<double>();
    if (node["y"]) value.y() = node["y"].as<double>();
    if (node["z"]) value.z() = node["z"].as<double>();
    return value;
}

YAML::Node writeVector3(const Eigen::Vector3d& value)
{
    YAML::Node node;
    node["x"] = toYamlNumber(value.x());
    node["y"] = toYamlNumber(value.y());
    node["z"] = toYamlNumber(value.z());
    node.SetStyle(YAML::EmitterStyle::Flow);
    return node;
}

template <typename T>
void readValue(const YAML::Node& node, const std::string& key, T& value)
{
    if (node && node[key])
        value = node[key].as<T>();
}

std::string orientationModeToString(OrientationMode mode)
{
    switch (mode)
    {
        case OrientationMode::None: return "none";
        case OrientationMode::Yaw:  return "yaw";
        case OrientationMode::Full: return "full";
    }
    return "unknown";
}

OrientationMode orientationModeFromString(const std::string& name)
{
    if (name == "none") return OrientationMode::None;
    if (name == "yaw")  return OrientationMode::Yaw;
    if (name == "full") return OrientationMode::Full;
    throw std::invalid_argument("Unknown orientation mode: \"" + name + "\" (expected none, yaw or full)");
}

bool isInside(const Eigen::Vector3d& point, const Eigen::Vector3d& min, const Eigen::Vector3d& max)
{
    return (point.array() >= min.array()).all() && (point.array() <= max.array()).all();
}

} // namespace


MapGeneratorConfig::MapGeneratorConfig()
{
    for (ShapeType type : allShapeTypes())
        shape_weights.emplace_back(type, 1.0);
}

MapGeneratorConfig MapGeneratorConfig::fromYamlFile(const std::string& file_path)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(file_path);
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error("Failed to load the map generator config " + file_path + ": " + e.what());
    }

    if (!root["map_generator"])
        throw std::runtime_error("The map generator config " + file_path + " has no \"map_generator\" key.");

    return fromYaml(root["map_generator"]);
}

MapGeneratorConfig MapGeneratorConfig::fromYaml(const YAML::Node& node)
{
    MapGeneratorConfig config;

    // General
    readValue(node, "name", config.name);
    readValue(node, "seed", config.seed);
    readValue(node, "output_directory", config.output_directory);

    // Map
    const YAML::Node map = node["map"];
    readValue(map, "resolution", config.resolution);
    readValue(map, "mark_bounds", config.mark_bounds);
    if (map)
    {
        config.origin = readVector3(map["origin"], config.origin);
        config.size = readVector3(map["size"], config.size);
    }

    // Obstacles
    const YAML::Node obstacles = node["obstacles"];
    if (obstacles)
    {
        readValue(obstacles, "density", config.density);
        readValue(obstacles, "density_reference_area", config.density_reference_area);
        readValue(obstacles, "max_aspect_ratio", config.max_aspect_ratio);
        readValue(obstacles, "ground_anchored_ratio", config.ground_anchored_ratio);
        readValue(obstacles, "min_clearance", config.min_clearance);
        readValue(obstacles, "max_placement_attempts", config.max_placement_attempts);

        const YAML::Node volume = obstacles["volume"];
        readValue(volume, "mean", config.volume_mean);
        readValue(volume, "std_ratio", config.volume_std_ratio);
        readValue(volume, "min", config.volume_min);
        readValue(volume, "max", config.volume_max);

        if (obstacles["orientation"])
            config.orientation_mode = orientationModeFromString(obstacles["orientation"].as<std::string>());

        // When the shapes are given, the shapes that are not listed are not used
        const YAML::Node shapes = obstacles["shapes"];
        if (shapes)
        {
            for (const auto& entry : shapes)
                shapeTypeFromString(entry.first.as<std::string>()); // Throws on unknown shape names

            for (auto& [type, weight] : config.shape_weights)
            {
                const std::string type_name = shapeTypeToString(type);
                weight = shapes[type_name] ? shapes[type_name].as<double>() : 0.0;
            }
        }
    }

    // Free zones
    if (node["free_zones"])
    {
        for (const auto& zone_node : node["free_zones"])
        {
            FreeZone zone;
            zone.center = readVector3(zone_node, zone.center);
            readValue(zone_node, "radius", zone.radius);
            config.free_zones.push_back(zone);
        }
    }

    // Testbench
    const YAML::Node testbench = node["testbench"];
    if (testbench)
    {
        readValue(testbench, "enabled", config.testbench_enabled);
        readValue(testbench, "config_directory", config.testbench_config_directory);
        readValue(testbench, "clearance", config.testbench_clearance);
        config.testbench_start = readVector3(testbench["start"], config.testbench_start);
        config.testbench_goal = readVector3(testbench["goal"], config.testbench_goal);
    }

    config.validate();
    return config;
}

YAML::Node MapGeneratorConfig::toYaml() const
{
    YAML::Node node;
    node["name"] = name;
    node["seed"] = seed;
    node["output_directory"] = output_directory;

    node["map"]["resolution"] = toYamlNumber(resolution);
    node["map"]["origin"] = writeVector3(origin);
    node["map"]["size"] = writeVector3(size);
    node["map"]["mark_bounds"] = mark_bounds;

    YAML::Node obstacles;
    obstacles["density"] = toYamlNumber(density);
    obstacles["density_reference_area"] = toYamlNumber(density_reference_area);
    obstacles["volume"]["mean"] = toYamlNumber(volume_mean);
    obstacles["volume"]["std_ratio"] = toYamlNumber(volume_std_ratio);
    obstacles["volume"]["min"] = toYamlNumber(volume_min);
    obstacles["volume"]["max"] = toYamlNumber(volume_max);
    obstacles["max_aspect_ratio"] = toYamlNumber(max_aspect_ratio);
    obstacles["ground_anchored_ratio"] = toYamlNumber(ground_anchored_ratio);
    obstacles["orientation"] = orientationModeToString(orientation_mode);
    obstacles["min_clearance"] = toYamlNumber(min_clearance);
    obstacles["max_placement_attempts"] = max_placement_attempts;
    for (const auto& [type, weight] : shape_weights)
        obstacles["shapes"][shapeTypeToString(type)] = toYamlNumber(weight);
    node["obstacles"] = obstacles;

    node["free_zones"] = YAML::Node(YAML::NodeType::Sequence);
    for (const FreeZone& zone : free_zones)
    {
        YAML::Node zone_node = writeVector3(zone.center);
        zone_node["radius"] = toYamlNumber(zone.radius);
        node["free_zones"].push_back(zone_node);
    }

    node["testbench"]["enabled"] = testbench_enabled;
    node["testbench"]["config_directory"] = testbench_config_directory;
    node["testbench"]["clearance"] = toYamlNumber(testbench_clearance);
    node["testbench"]["start"] = writeVector3(testbench_start);
    node["testbench"]["goal"] = writeVector3(testbench_goal);

    return node;
}

void MapGeneratorConfig::validate() const
{
    auto check = [](bool condition, const std::string& message)
    {
        if (!condition)
            throw std::invalid_argument("Invalid map generator config: " + message);
    };

    check(!name.empty(), "name can't be empty");
    check(resolution > 0.0, "map.resolution must be positive");
    check((size.array() >= resolution).all(), "map.size must be at least map.resolution on every axis");
    check(density >= 0.0, "obstacles.density can't be negative");
    check(density_reference_area > 0.0, "obstacles.density_reference_area must be positive");
    check(volume_mean > 0.0, "obstacles.volume.mean must be positive");
    check(volume_std_ratio >= 0.0, "obstacles.volume.std_ratio can't be negative");
    check(volume_min > 0.0, "obstacles.volume.min must be positive");
    check(volume_max >= volume_min, "obstacles.volume.max must be greater or equal to obstacles.volume.min");
    check(max_aspect_ratio >= 1.0, "obstacles.max_aspect_ratio must be at least 1");
    check(ground_anchored_ratio >= 0.0 && ground_anchored_ratio <= 1.0, "obstacles.ground_anchored_ratio must be in [0, 1]");
    check(max_placement_attempts > 0, "obstacles.max_placement_attempts must be positive");

    double total_weight = 0.0;
    for (const auto& [type, weight] : shape_weights)
    {
        check(weight >= 0.0, "obstacles.shapes." + shapeTypeToString(type) + " can't be negative");
        total_weight += weight;
    }
    check(total_weight > 0.0 || density == 0.0, "at least one shape in obstacles.shapes needs a positive weight");

    for (const FreeZone& zone : free_zones)
        check(zone.radius >= 0.0, "free_zones radius can't be negative");

    if (testbench_enabled)
    {
        check(testbench_clearance >= 0.0, "testbench.clearance can't be negative");
        check(isInside(testbench_start, origin, getMaxCorner()), "testbench.start must be inside the map");
        check(isInside(testbench_goal, origin, getMaxCorner()), "testbench.goal must be inside the map");
    }
}

int MapGeneratorConfig::getTargetObstacleCount() const
{
    double ground_area = size.x() * size.y();
    return static_cast<int>(std::lround(density * ground_area / density_reference_area));
}

std::string MapGeneratorConfig::getMapName() const
{
    return name + "_seed" + std::to_string(seed);
}

std::vector<FreeZone> MapGeneratorConfig::getAllFreeZones() const
{
    std::vector<FreeZone> zones = free_zones;
    if (testbench_enabled)
    {
        zones.push_back({testbench_start, testbench_clearance});
        zones.push_back({testbench_goal, testbench_clearance});
    }
    return zones;
}

YAML::Node toYamlNumber(double value)
{
    std::ostringstream stream;
    stream << std::setprecision(6) << value;
    return YAML::Node(stream.str());
}

}; // namespace map_generator

}; // namespace arena_demos
