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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// grid_map
#include <grid_map_core/GridMap.hpp>

#include <mutex>
#include <memory>
#include <unordered_map>
#include <unordered_set>


namespace arena_demos
{

struct RobotBoundingBox
{
    double x_width = 0;
    double y_width = 0;

    bool isInside(const Eigen::Vector2d &local_map_pose, const grid_map::Position &point_pose)
    {
        double half_x = x_width / 2.0;
        double half_y = y_width / 2.0;

        if (point_pose.x() < (local_map_pose.x() - half_x) || point_pose.x() > (local_map_pose.x() + half_x))
            return false;
        if (point_pose.y() < (local_map_pose.y() - half_y) || point_pose.y() > (local_map_pose.y() + half_y))
            return false;

        return true;
    };
}; // struct RobotBoundingBox


struct TraversabilityMapConfig
{
    std::string root_frame = "world";

    // Map parameters
    double map_resolution = 0.3; // meters
    double local_map_size_x = 30.0; // meters
    double local_map_size_y = 30.0; // meters
    double global_map_size_x = 50.0; // meters
    double global_map_size_y = 50.0; // meters

    // Traversability constraints
    double max_slope = 0.436332313; // radians (~25 degrees)
    double max_step = 0.5; // meters
    double occupancy_threshold = 0.9;

    // Traversability specific costs
    double constraint_cost = 4.0; // Cost for violating slope/step/occupancy constraints
    double unknown_cost = 1.0; // Cost for unknown terrain

    // Traversability cost weights
    double slope_weight = 1.0;
    double step_weight = 1.0;
    double occupancy_weight = 1.0;

}; // struct TraversabilityMapConfig


class TraversabilityMap
{
public:

    TraversabilityMap();
    TraversabilityMap(const TraversabilityMapConfig &a_config);
    TraversabilityMap(const RobotBoundingBox &a_robot_bb, const TraversabilityMapConfig &a_config);
    TraversabilityMap(const RobotBoundingBox &a_robot_bb);
    
    void moveLocalMap(const Eigen::Vector2d &a_pose);
    void updateMap(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &a_ground_cloud,
                    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &a_non_ground_cloud);

    std::shared_ptr<grid_map::GridMap> getLocalMap() { return local_map_; };
    std::shared_ptr<grid_map::GridMap> getGlobalMap() { return global_map_; };
    bool isMapInitialized() { return global_map_initialized_; };

private:

    void updateMap(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &a_cloud, std::vector<grid_map::Index> &a_changed_cells, const bool is_ground = false);
    void initializeMaps(const grid_map::Position &a_center);
    void updateElevationAtIndex(const grid_map::Index &index, const float z, std::vector<grid_map::Index> &a_changed_cells);
    void updateOccupancyLogOddsAtIndex(const grid_map::Index &index, const bool occupied);
    void updateOccupancyRay(const grid_map::Position &start, const grid_map::Position &end, std::vector<grid_map::Index> &a_changed_cells);
    void updateStepAtIndex(const grid_map::Index &index);
    void updateSlopeAtIndex(const grid_map::Index &index);
    void reconcileOccupancyWithTerrain(const grid_map::Index &index);
    void computeInflatedOccupancy();
    void updateCostAtIndex(const grid_map::Index &index, double &a_cost);

    // Records index as changed if it hasn't already been recorded this
    // update (O(1) average via changed_cells_lookup_, instead of an O(n)
    // linear scan of a_changed_cells).
    void markCellChanged(const grid_map::Index &index, std::vector<grid_map::Index> &a_changed_cells);

    // User-defined attributes
    std::shared_ptr<grid_map::GridMap> global_map_;
    std::shared_ptr<grid_map::GridMap> local_map_;

    Eigen::Vector2d local_map_pose_;
    std::mutex map_mutex_;
    bool global_map_initialized_ = false;
    RobotBoundingBox robot_bb_;
    TraversabilityMapConfig config_;

    // Raw pointers into global_map_'s own layer storage, cached once in
    // initializeMaps() to avoid a hashed string lookup on every access.
    // Valid for the lifetime of global_map_, which is created exactly once
    // and never has layers added to or erased from it afterward.
    grid_map::Matrix *elevation_mean_ = nullptr;
    grid_map::Matrix *elevation_variance_ = nullptr;
    grid_map::Matrix *elevation_num_measurements_ = nullptr;
    grid_map::Matrix *elevation_reject_streak_ = nullptr;
    grid_map::Matrix *step_ = nullptr;
    grid_map::Matrix *slope_ = nullptr;
    grid_map::Matrix *occupancy_logodds_ = nullptr;
    grid_map::Matrix *occupancy_probability_ = nullptr;
    grid_map::Matrix *inflated_occupancy_ = nullptr;
    grid_map::Matrix *cost_ = nullptr;

    // Scratch buffers for updateMap(), cleared (not reallocated) at the
    // start of each call so their capacity is reused across scans.
    std::vector<grid_map::Index> changed_cells_;
    std::unordered_set<int64_t> changed_cells_lookup_;

}; // class TraversabilityMap

}; // namespace arena_demos
