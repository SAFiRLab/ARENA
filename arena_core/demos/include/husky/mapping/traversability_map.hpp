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
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/GridMap.hpp>

#include <mutex>
#include <memory>


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


class TraversabilityMap
{
public:

    TraversabilityMap();

    void moveLocalMap(const Eigen::Vector2d &a_pose);
    void updateMap(const pcl::PointCloud<pcl::PointXYZ> &a_cloud);

    std::shared_ptr<grid_map::GridMap> getLocalMap() { return local_map_; };
    std::shared_ptr<grid_map::GridMap> getGlobalMap() { return global_map_; };
    bool isMapInitialized() { return global_map_initialized_; };

private:

    void initializeMaps(const grid_map::Position &a_center);
    void updateStepAtIter(const grid_map::GridMapIterator &it);
    void updateSlopeAtIter(const grid_map::GridMapIterator &it);
    void updateOccupancyAtIter(const grid_map::GridMapIterator &it);
    void updateCostAtIter(const grid_map::GridMapIterator &it, double &a_cost);
    void normalizeCosts(const double a_max_cost);

    // User-defined attirbutes
    std::shared_ptr<grid_map::GridMap> global_map_;
    std::shared_ptr<grid_map::GridMap> local_map_;

    Eigen::Vector2d local_map_pose_;
    std::mutex map_mutex_;
    bool global_map_initialized_ = false;
    RobotBoundingBox robot_bb_;

}; // class TraversabilityMap

}; // namespace arena_demos
