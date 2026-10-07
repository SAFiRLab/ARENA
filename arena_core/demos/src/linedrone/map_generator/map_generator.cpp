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
#include "linedrone/map_generator/map_generator.hpp"

// System
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

// External Libraries
#include <yaml-cpp/yaml.h>


namespace arena_demos
{

namespace map_generator
{

namespace
{

octomap::point3d toPoint3d(const Eigen::Vector3d& vector)
{
    return octomap::point3d(static_cast<float>(vector.x()), static_cast<float>(vector.y()), static_cast<float>(vector.z()));
}

Eigen::Vector3d toEigen(const octomap::point3d& point)
{
    return Eigen::Vector3d(point.x(), point.y(), point.z());
}

YAML::Node flowSequence(std::initializer_list<double> values)
{
    YAML::Node node(YAML::NodeType::Sequence);
    for (double value : values)
        node.push_back(toYamlNumber(value));
    node.SetStyle(YAML::EmitterStyle::Flow);
    return node;
}

void createParentDirectory(const std::string& file_path)
{
    std::filesystem::path parent = std::filesystem::path(file_path).parent_path();
    if (!parent.empty())
        std::filesystem::create_directories(parent);
}

} // namespace


MapGenerator::MapGenerator(const MapGeneratorConfig& config)
: config_(config)
{
    config_.validate();
}

void MapGenerator::generate()
{
    auto begin = std::chrono::steady_clock::now();

    octree_ = std::make_shared<octomap::OcTree>(config_.resolution);
    obstacles_.clear();
    occupied_keys_.clear();
    stats_ = GenerationStats();
    free_zones_ = config_.getAllFreeZones();
    buildClearanceStencil();

    // The whole map must be addressable by the octree keys
    octomap::OcTreeKey key;
    if (!octree_->coordToKeyChecked(toPoint3d(config_.origin), key) || !octree_->coordToKeyChecked(toPoint3d(config_.getMaxCorner()), key))
        throw std::invalid_argument("MapGenerator::generate => the map is too large for an octree at this resolution.");

    const float occupied_log_odds = octree_->getClampingThresMaxLog();
    const int nb_of_obstacles = config_.getTargetObstacleCount();
    stats_.nb_of_requested_obstacles = nb_of_obstacles;

    for (int i = 0; i < nb_of_obstacles; ++i)
    {
        // Every obstacle has its own random stream
        Random rng(Random::deriveSeed(config_.seed, static_cast<uint64_t>(i)));
        bool is_ground_anchored = rng.uniform() < config_.ground_anchored_ratio;

        bool is_placed = false;
        for (int attempt = 0; attempt < config_.max_placement_attempts && !is_placed; ++attempt)
        {
            std::unique_ptr<Shape> shape = sampleObstacle(rng, is_ground_anchored);
            if (!shape)
                continue;

            std::vector<octomap::OcTreeKey> voxels = voxelize(*shape);
            if (voxels.empty() || !isPlacementValid(voxels))
                continue;

            for (const octomap::OcTreeKey& voxel : voxels)
            {
                occupied_keys_.insert(voxel);
                octree_->setNodeValue(voxel, occupied_log_odds, true);
            }

            stats_.obstacles_volume += shape->getVolume();
            obstacles_.push_back({std::move(shape), is_ground_anchored, voxels.size()});
            is_placed = true;
        }

        if (!is_placed)
            stats_.nb_of_failed_obstacles++;
    }

    if (config_.mark_bounds)
        markBounds();

    octree_->updateInnerOccupancy();
    octree_->prune();

    // Statistics
    const double voxel_volume = std::pow(config_.resolution, 3);
    stats_.nb_of_placed_obstacles = static_cast<int>(obstacles_.size());
    stats_.nb_of_occupied_voxels = occupied_keys_.size();
    stats_.occupied_volume = occupied_keys_.size() * voxel_volume;
    stats_.occupancy_ratio = stats_.occupied_volume / config_.size.prod();
    stats_.achieved_density = stats_.nb_of_placed_obstacles * config_.density_reference_area / (config_.size.x() * config_.size.y());
    stats_.generation_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
}

std::unique_ptr<Shape> MapGenerator::sampleObstacle(Random& rng, bool is_ground_anchored) const
{
    // Type
    std::vector<double> weights;
    for (const auto& [type, weight] : config_.shape_weights)
        weights.push_back(weight);
    ShapeType type = config_.shape_weights[rng.weightedIndex(weights)].first;

    // Volume and proportions
    double volume = std::clamp(rng.logNormal(config_.volume_mean, config_.volume_std_ratio), config_.volume_min, config_.volume_max);
    std::unique_ptr<Shape> shape = makeRandomShape(type, volume, config_.max_aspect_ratio, rng);

    // Orientation. Ground anchored obstacles stay upright (cones and prisms stand on their base)
    OrientationMode orientation_mode = config_.orientation_mode;
    if (is_ground_anchored && orientation_mode == OrientationMode::Full)
        orientation_mode = OrientationMode::Yaw;
    shape->setRotation(rng.rotation(orientation_mode));

    // Position: the whole bounding box has to fit in the map
    const Eigen::Vector3d half_extents = shape->getWorldHalfExtents();
    const Eigen::Vector3d low = config_.origin + half_extents;
    const Eigen::Vector3d high = config_.getMaxCorner() - half_extents;

    Eigen::Vector3d center;
    center.x() = rng.uniform(low.x(), high.x());
    center.y() = rng.uniform(low.y(), high.y());
    center.z() = is_ground_anchored ? low.z() : rng.uniform(low.z(), high.z());
    shape->setCenter(center);

    // Rejected after drawing every value, so the number of draws of an attempt doesn't depend on the outcome
    if ((low.array() > high.array()).any())
        return nullptr; // Larger than the map

    if (shape->getMinFeatureSize() < config_.resolution)
        return nullptr; // Too thin to be a closed volume once voxelized

    return shape;
}

std::vector<octomap::OcTreeKey> MapGenerator::voxelize(const Shape& shape) const
{
    std::vector<octomap::OcTreeKey> voxels;

    Eigen::AlignedBox3d box = shape.getWorldBoundingBox();
    Eigen::Vector3d low = box.min().cwiseMax(config_.origin);
    Eigen::Vector3d high = box.max().cwiseMin(config_.getMaxCorner());
    if ((low.array() > high.array()).any())
        return voxels;

    octomap::OcTreeKey key_low = octree_->coordToKey(toPoint3d(low));
    octomap::OcTreeKey key_high = octree_->coordToKey(toPoint3d(high));

    octomap::OcTreeKey key;
    for (key[0] = key_low[0]; key[0] <= key_high[0]; ++key[0])
    {
        for (key[1] = key_low[1]; key[1] <= key_high[1]; ++key[1])
        {
            for (key[2] = key_low[2]; key[2] <= key_high[2]; ++key[2])
            {
                octomap::point3d voxel_center = octree_->keyToCoord(key);
                if (isInsideMap(voxel_center) && shape.contains(toEigen(voxel_center)))
                    voxels.push_back(key);
            }
        }
    }

    return voxels;
}

bool MapGenerator::isPlacementValid(const std::vector<octomap::OcTreeKey>& voxels) const
{
    // A voxel is in a free zone if its cube intersects the zone sphere (conservative: center distance < radius + half diagonal)
    const double half_diagonal = 0.5 * std::sqrt(3.0) * config_.resolution;
    const bool allow_overlap = config_.min_clearance < 0.0;

    for (const octomap::OcTreeKey& voxel : voxels)
    {
        if (!allow_overlap && occupied_keys_.count(voxel) > 0)
            return false;

        Eigen::Vector3d point = toEigen(octree_->keyToCoord(voxel));
        for (const FreeZone& zone : free_zones_)
        {
            double distance = zone.radius + half_diagonal;
            if ((point - zone.center).squaredNorm() < distance * distance)
                return false;
        }
    }

    if (clearance_stencil_.empty())
        return true;

    // Only the surface voxels can be the closest to another obstacle: the interior ones don't need the clearance check
    octomap::KeySet members(voxels.begin(), voxels.end());
    auto offsetKey = [](const octomap::OcTreeKey& key, int dx, int dy, int dz)
    {
        return octomap::OcTreeKey(key[0] + dx, key[1] + dy, key[2] + dz);
    };

    for (const octomap::OcTreeKey& voxel : voxels)
    {
        bool is_surface =
            !members.count(offsetKey(voxel, 1, 0, 0)) || !members.count(offsetKey(voxel, -1, 0, 0)) ||
            !members.count(offsetKey(voxel, 0, 1, 0)) || !members.count(offsetKey(voxel, 0, -1, 0)) ||
            !members.count(offsetKey(voxel, 0, 0, 1)) || !members.count(offsetKey(voxel, 0, 0, -1));
        if (!is_surface)
            continue;

        for (const Eigen::Vector3i& offset : clearance_stencil_)
        {
            if (occupied_keys_.count(offsetKey(voxel, offset.x(), offset.y(), offset.z())) > 0)
                return false;
        }
    }

    return true;
}

bool MapGenerator::isInsideMap(const octomap::point3d& point) const
{
    const Eigen::Vector3d max_corner = config_.getMaxCorner();
    return point.x() >= config_.origin.x() && point.x() <= max_corner.x() &&
           point.y() >= config_.origin.y() && point.y() <= max_corner.y() &&
           point.z() >= config_.origin.z() && point.z() <= max_corner.z();
}

void MapGenerator::buildClearanceStencil()
{
    clearance_stencil_.clear();
    if (config_.min_clearance <= 0.0)
        return;

    // Two voxels n voxels apart leave a gap of (n - 1) * resolution between them,
    // so the gap is smaller than min_clearance if their center distance is smaller than min_clearance + resolution
    const double max_distance = config_.min_clearance / config_.resolution + 1.0;
    const int range = static_cast<int>(std::ceil(max_distance));
    for (int dx = -range; dx <= range; ++dx)
    {
        for (int dy = -range; dy <= range; ++dy)
        {
            for (int dz = -range; dz <= range; ++dz)
            {
                Eigen::Vector3i offset(dx, dy, dz);
                if (offset != Eigen::Vector3i::Zero() && offset.cast<double>().norm() < max_distance - 1e-9)
                    clearance_stencil_.push_back(offset);
            }
        }
    }
}

void MapGenerator::markBounds()
{
    // The planner takes its bounds from the extent of the octree leaves (CostmapMapping::getMapBounds).
    // Free voxels at the 8 corners make these bounds match the map size, whatever the obstacles are.
    const float free_log_odds = octree_->getClampingThresMinLog();
    const Eigen::Vector3d inset = Eigen::Vector3d::Constant(config_.resolution / 2.0);
    const Eigen::Vector3d low = config_.origin + inset;
    const Eigen::Vector3d high = config_.getMaxCorner() - inset;

    for (int corner = 0; corner < 8; ++corner)
    {
        Eigen::Vector3d point((corner & 1) ? high.x() : low.x(), (corner & 2) ? high.y() : low.y(), (corner & 4) ? high.z() : low.z());
        octomap::OcTreeKey key = octree_->coordToKey(toPoint3d(point));
        if (!occupied_keys_.count(key))
            octree_->setNodeValue(key, free_log_odds, true);
    }
}

bool MapGenerator::saveBinary(const std::string& file_path) const
{
    if (!octree_)
        throw std::runtime_error("MapGenerator::saveBinary => the map has not been generated.");

    createParentDirectory(file_path);
    return octree_->writeBinary(file_path);
}

void MapGenerator::saveMetadata(const std::string& file_path, const std::string& bt_file_path) const
{
    if (!octree_)
        throw std::runtime_error("MapGenerator::saveMetadata => the map has not been generated.");

    YAML::Node root;
    root["map_name"] = config_.getMapName();
    root["bt_file"] = bt_file_path;

    YAML::Node config_node;
    config_node["map_generator"] = config_.toYaml();
    root["config"] = config_node;

    YAML::Node stats;
    stats["nb_of_requested_obstacles"] = stats_.nb_of_requested_obstacles;
    stats["nb_of_placed_obstacles"] = stats_.nb_of_placed_obstacles;
    stats["nb_of_failed_obstacles"] = stats_.nb_of_failed_obstacles;
    stats["nb_of_occupied_voxels"] = stats_.nb_of_occupied_voxels;
    stats["obstacles_volume"] = toYamlNumber(stats_.obstacles_volume);
    stats["occupied_volume"] = toYamlNumber(stats_.occupied_volume);
    stats["occupancy_ratio"] = toYamlNumber(stats_.occupancy_ratio);
    stats["achieved_density"] = toYamlNumber(stats_.achieved_density);
    root["stats"] = stats;

    root["obstacles"] = YAML::Node(YAML::NodeType::Sequence);
    for (size_t i = 0; i < obstacles_.size(); ++i)
    {
        const Obstacle& obstacle = obstacles_[i];
        const Eigen::Vector3d& center = obstacle.shape->getCenter();
        Eigen::Quaterniond orientation(obstacle.shape->getRotation());

        YAML::Node node;
        node["id"] = i;
        node["type"] = shapeTypeToString(obstacle.shape->getType());
        node["ground_anchored"] = obstacle.is_ground_anchored;
        node["center"] = flowSequence({center.x(), center.y(), center.z()});
        node["orientation_xyzw"] = flowSequence({orientation.x(), orientation.y(), orientation.z(), orientation.w()});
        node["volume"] = toYamlNumber(obstacle.shape->getVolume());
        node["nb_of_voxels"] = obstacle.nb_of_voxels;

        YAML::Node dimensions;
        for (const auto& [name, value] : obstacle.shape->getDimensions())
            dimensions[name] = toYamlNumber(value);
        dimensions.SetStyle(YAML::EmitterStyle::Flow);
        node["dimensions"] = dimensions;

        root["obstacles"].push_back(node);
    }

    YAML::Emitter emitter;
    emitter << root;

    createParentDirectory(file_path);
    std::ofstream file(file_path);
    if (!file)
        throw std::runtime_error("MapGenerator::saveMetadata => failed to open " + file_path);

    file << "# Generated by the map_generator. Regenerate this map with the config below.\n";
    file << emitter.c_str() << "\n";
}

void MapGenerator::saveTestbenchConfig(const std::string& file_path) const
{
    createParentDirectory(file_path);
    std::ofstream file(file_path);
    if (!file)
        throw std::runtime_error("MapGenerator::saveTestbenchConfig => failed to open " + file_path);

    const Eigen::Vector3d& start = config_.testbench_start;
    const Eigen::Vector3d& goal = config_.testbench_goal;

    file << std::fixed << std::setprecision(3);
    file << "# Testbench configuration of the generated map " << config_.getMapName() << ".bt\n";
    file << "# Generated by the map_generator: both points are at least " << std::defaultfloat << config_.testbench_clearance << std::fixed
         << " m away from the obstacles.\n\n";

    file << "# Start pose of the drone, published by testbench_utilities_publisher_node on /testbench/drone/odom\n";
    file << "drone:\n";
    file << "  position:\n";
    file << "    x: " << start.x() << "\n";
    file << "    y: " << start.y() << "\n";
    file << "    z: " << start.z() << "\n";
    file << "  orientation:\n";
    file << "    x: 0.0\n";
    file << "    y: 0.0\n";
    file << "    z: 0.0\n";
    file << "    w: 1.0\n\n";

    file << "# Goal sent to the planner by testbench_node\n";
    file << "planning_goal:\n";
    file << "  position:\n";
    file << "    x: " << goal.x() << "\n";
    file << "    y: " << goal.y() << "\n";
    file << "    z: " << goal.z() << "\n";
}

std::string MapGenerator::getSummary() const
{
    const Eigen::Vector3d& size = config_.size;

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(2);
    summary << "Map " << config_.getMapName() << "\n";
    summary << "  Size:          " << size.x() << " x " << size.y() << " x " << size.z() << " m (resolution " << config_.resolution << " m)\n";
    summary << "  Obstacles:     " << stats_.nb_of_placed_obstacles << " / " << stats_.nb_of_requested_obstacles << " placed";
    if (stats_.nb_of_failed_obstacles > 0)
        summary << " (" << stats_.nb_of_failed_obstacles << " couldn't be placed: lower the density, the volume or obstacles.min_clearance)";
    summary << "\n";
    summary << "  Density:       " << stats_.achieved_density << " obstacles per " << config_.density_reference_area
            << " m^2 (requested " << config_.density << ")\n";
    summary << "  Mean volume:   " << (stats_.nb_of_placed_obstacles > 0 ? stats_.obstacles_volume / stats_.nb_of_placed_obstacles : 0.0)
            << " m^3 (requested " << config_.volume_mean << ")\n";
    summary << "  Occupied:      " << stats_.nb_of_occupied_voxels << " voxels, " << stats_.occupied_volume << " m^3 ("
            << 100.0 * stats_.occupancy_ratio << " % of the map)\n";
    summary << "  Generated in:  " << std::setprecision(3) << stats_.generation_time << " s";
    return summary.str();
}

}; // namespace map_generator

}; // namespace arena_demos
