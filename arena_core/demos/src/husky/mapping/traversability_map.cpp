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

#include "husky/mapping/traversability_map.hpp"

// grid_map
#include <grid_map_core/iterators/LineIterator.hpp>

// System
#include <unordered_set>
#include <queue>
#include <vector>
#include <iostream>


constexpr float HIT = 0.85f;
constexpr float MISS = -0.40f;

constexpr float MIN_L = -5.0f;
constexpr float MAX_L = 5.0f;


namespace arena_demos
{

TraversabilityMap::TraversabilityMap()
: global_map_(nullptr), local_map_(nullptr)
{
    robot_bb_.x_width = 1.0;
    robot_bb_.y_width = 1.0;
}

void TraversabilityMap::initializeMaps(const grid_map::Position &a_center)
{
    // ----- GLOBAL MAP -----
    global_map_ = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"elevation_mean"});
    global_map_->add("elevation_variance");
    global_map_->add("elevation_num_measurements");
    global_map_->add("step");
    global_map_->add("slope");
    global_map_->add("occupancy_logodds");
    global_map_->add("occupancy_probability");
    global_map_->add("inflated_occupancy");
    global_map_->add("cost");
    global_map_->setFrameId("world");

    global_map_->setGeometry(grid_map::Length(50.0, 50.0), 0.3, a_center);
    (*global_map_)["elevation_mean"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["elevation_variance"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["elevation_num_measurements"].setConstant(0.0f);
    (*global_map_)["step"].setZero();
    (*global_map_)["slope"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["occupancy_logodds"].setZero();
    (*global_map_)["occupancy_probability"].setConstant(0.5f);
    (*global_map_)["inflated_occupancy"].setConstant(0.0f);
    (*global_map_)["cost"].setConstant(0.0f);
    //(*global_map_)["cost"].setConstant(1000.0);

    // ----- LOCAL MAP -----
    local_map_ = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"elevation_mean"});
    local_map_->setFrameId("world");
    local_map_->setGeometry(grid_map::Length(30.0, 30.0), 0.3);

    global_map_initialized_ = true;
}

void TraversabilityMap::moveLocalMap(const Eigen::Vector2d &a_pose)
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    local_map_pose_ = a_pose;
    grid_map::Position center(local_map_pose_.x(), local_map_pose_.y());

    if (!global_map_initialized_)
        initializeMaps(center);

    // Extract submap from global
    bool is_success = false;
    local_map_ = std::make_shared<grid_map::GridMap>(global_map_->getSubmap(center, local_map_->getLength(), is_success));

    if (!is_success)
    {
        std::cerr << "Failed to extract local map submap at position " << center.x() << ", " << center.y() << std::endl;;
        local_map_ = nullptr;
        return;
    }
}

void TraversabilityMap::updateMap(const pcl::PointCloud<pcl::PointXYZ> &a_cloud)
{
    if (!global_map_initialized_)
        return;
    
    if (a_cloud.empty())
        return;

    std::lock_guard<std::mutex> lock(map_mutex_);
    grid_map::Position start(local_map_pose_.x(), local_map_pose_.y());
    max_step_iter_ = 0.0f;
    max_slope_iter_ = 0.0f;

    for (const auto & p : a_cloud.points)
    {
        if (!std::isfinite(p.z)) continue;

        grid_map::Position pos(p.x, p.y);
        if (!global_map_->isInside(pos)) continue;

        if (robot_bb_.isInside(local_map_pose_, pos)) continue;

        grid_map::Index index;
        global_map_->getIndex(pos, index);

        updateElevationAtIndex(index, p.z);
        updateOccupancyRay(start, pos);
    }

    double max_cost = 0.0;
    for (grid_map::GridMapIterator it(*global_map_); !it.isPastEnd(); ++it)
    {
        updateStepAtIter(it);
        updateSlopeAtIter(it);
    }

    computeInflatedOccupancy();
    normalizeLayersAndApplyCost();
}

void TraversabilityMap::updateElevationAtIndex(const grid_map::Index &index, const float z)
{
    if (!global_map_->isValid(index, "elevation_mean"))
    {
        global_map_->at("elevation_mean", index) = z;
        global_map_->at("elevation_variance", index) = 0.1f;
        global_map_->at("elevation_num_measurements", index) = 1.0f;
        return;
    }
    
    constexpr float measurement_variance = 0.01f;   // Sensor noise (R)
    constexpr float process_noise = 1e-4f;          // Terrain evolution (Q)

    float mean = global_map_->at("elevation_mean", index);
    float variance = global_map_->at("elevation_variance", index);

    // Prevent variance from collapsing to zero.
    variance += process_noise;

    // Innovation
    const float innovation = z - mean;

    // Innovation covariance
    const float S = variance + measurement_variance;

    // Optional statistical gating
    if (std::abs(innovation) > 3.0f * std::sqrt(S))
        return;

    // Kalman gain
    const float K = variance / S;

    // Bayesian update
    mean += K * innovation;
    variance *= (1.0f - K);

    global_map_->at("elevation_mean", index) = mean;
    global_map_->at("elevation_variance", index) = variance;
    global_map_->at("elevation_num_measurements", index) += 1.0f;

    //std::cout << "Updated cell at index " << index.transpose() << ": mean = " << global_map_->at("elevation_mean", index) << ", variance = " << global_map_->at("elevation_variance", index) << std::endl;
}

void TraversabilityMap::updateOccupancyLogOddsAtIndex(const grid_map::Index &index, const bool occupied)
{
    float& L = global_map_->at("occupancy_logodds", index);

    if (occupied)
        L += HIT;
    else
        L += MISS;

    L = std::clamp(L, MIN_L, MAX_L);

    global_map_->at("occupancy_probability",index) = 1.f / (1.f + std::exp(-L));
}

void TraversabilityMap::updateOccupancyRay(const grid_map::Position &start, const grid_map::Position &end)
{
    for (grid_map::LineIterator it(*global_map_, start, end); !it.isPastEnd(); ++it)
        updateOccupancyLogOddsAtIndex(*it, false);

    grid_map::Index endpoint;
    global_map_->getIndex(end, endpoint);

    if (global_map_->at("elevation_num_measurements", endpoint) > 100)
        updateOccupancyLogOddsAtIndex(endpoint, true);
}

void TraversabilityMap::updateStepAtIter(const grid_map::GridMapIterator &it)
{
    const grid_map::Index index = *it;

    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    // If center cell invalid → step invalid
    if (!global_map_->isValid(index, "elevation_mean"))
    {
        global_map_->at("step", index) = 0.0f;
        return;
    }

    if (global_map_->at("elevation_num_measurements", index) < 100)
        return;
    
    const float z_center = global_map_->at("elevation_mean", index);
    float max_step = 0.0f;

    // 8-connected neighborhood
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx == 0 && dy == 0)
                continue;

            grid_map::Index neighbor = index + grid_map::Index(dx, dy);

            global_map_->getPosition(neighbor, pos);
            if (!global_map_->isInside(pos)) continue;

            if (!global_map_->isValid(neighbor, "elevation_mean"))
                continue;

            const float z_neighbor = global_map_->at("elevation_mean", neighbor);

            const float dz = std::abs(z_center - z_neighbor);

            if (dz > max_step)
                max_step = dz;
        }
    }

    global_map_->at("step", index) = max_step;

    if (max_step > max_step_iter_)
        max_step_iter_ = max_step;
}

void TraversabilityMap::updateSlopeAtIter(const grid_map::GridMapIterator &it)
{
    const grid_map::Index index = *it;

    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    if (!global_map_->isValid(index, "elevation_mean"))
    {
        global_map_->at("slope", index) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    if (global_map_->at("elevation_num_measurements", index) < 100)
        return;

    const double resolution = global_map_->getResolution();
    const int radius_cells = 3;   // ← tune this (3 = ~1m at 0.3m resolution)

    std::vector<Eigen::Vector3d> points;
    points.reserve((2 * radius_cells + 1) * (2 * radius_cells + 1));

    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
        {
            grid_map::Index neighbor = index;
            neighbor.x() += dx;
            neighbor.y() += dy;

            global_map_->getPosition(neighbor, pos);
            if (!global_map_->isInside(pos)) continue;

            if (!global_map_->isValid(neighbor, "elevation_mean"))
                continue;

            double x = dx * resolution;
            double y = dy * resolution;
            double z = global_map_->at("elevation_mean", neighbor);

            points.emplace_back(x, y, z);
        }
    }

    if (points.size() < 6)
    {
        global_map_->at("slope", index) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    // Build least squares system
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::VectorXd b(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        A(i,0) = points[i].x();
        A(i,1) = points[i].y();
        A(i,2) = 1.0;
        b(i)   = points[i].z();
    }

    Eigen::Vector3d coeff = A.colPivHouseholderQr().solve(b);

    double a = coeff(0);
    double b_coef = coeff(1);

    double gradient_norm = std::sqrt(a*a + b_coef*b_coef);

    double slope = std::atan(gradient_norm);

    global_map_->at("slope", index) = static_cast<float>(slope);

    if (slope > max_slope_iter_)
        max_slope_iter_ = static_cast<float>(slope);
}

void TraversabilityMap::computeInflatedOccupancy()
{
    // ---- TUNABLE PARAMETERS ----
    constexpr float occupied_threshold = 0.9f;
    constexpr double inflation_radius = 9.0;   // meters

    const double resolution = global_map_->getResolution();
    const grid_map::Size map_size = global_map_->getSize();
    const int rows = map_size(0);
    const int cols = map_size(1);
    const int num_cells = rows * cols;

    grid_map::Matrix inflated = (*global_map_)["inflated_occupancy"];
    inflated.setZero();

    const grid_map::Matrix &occupancy = (*global_map_)["occupancy_probability"];

    // ---- Multi-source wavefront (brushfire-style) distance transform ----
    //
    // We seed a single priority-queue-based wavefront from ALL occupied cells at
    // once and let it expand outward together. Each cell in the map is only
    // ever finalized once, and the distance assigned to it is the true
    // Euclidean distance to the obstacle cell that reached it first, so the result 
    // is a distance-to-nearest-obstacle field. This is the same "brushfire" idea 
    // as ROS's costmap_2d inflation layer uses.
    //
    // Cost: every cell is enqueued/dequeued O(1) times -> O(N log N) over the
    // whole map

    struct QueueEntry
    {
        float distance;
        int x, y;           // cell being finalized
        int src_x, src_y;   // obstacle cell it is being propagated from
    };

    struct QueueEntryCompare
    {
        bool operator()(const QueueEntry &a, const QueueEntry &b) const
        {
            return a.distance > b.distance;   // min-heap on distance
        }
    };

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> frontier;
    std::vector<char> finalized(num_cells, 0);

    for (int x = 0; x < rows; ++x)
        for (int y = 0; y < cols; ++y)
            if (occupancy(x, y) >= occupied_threshold)
                frontier.push({0.0f, x, y, x, y});

    if (frontier.empty())
        return;   // No obstacles this cycle -> inflated layer stays at zero.

    static constexpr int kNeighborOffsets[8][2] =
    {
        {-1, -1}, {-1, 0}, {-1, 1},
        { 0, -1},          { 0, 1},
        { 1, -1}, { 1, 0}, { 1, 1}
    };

    while (!frontier.empty())
    {
        const QueueEntry current = frontier.top();
        frontier.pop();

        const int lin = current.x * cols + current.y;
        if (finalized[lin])
            continue;   // A closer source already claimed this cell; stale entry.
        finalized[lin] = 1;

        if (current.distance > inflation_radius)
            continue;   // Outside the inflation footprint; don't keep expanding it.

        const float decay = 1.0f - static_cast<float>(current.distance / inflation_radius);
        inflated(current.x, current.y) = std::max(inflated(current.x, current.y), decay);
        (*global_map_)["inflated_occupancy"](current.x, current.y) = inflated(current.x, current.y);

        for (const auto &offset : kNeighborOffsets)
        {
            const int nx = current.x + offset[0];
            const int ny = current.y + offset[1];

            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols)
                continue;

            const int neighbor_lin = nx * cols + ny;
            if (finalized[neighbor_lin])
                continue;

            const double ddx = (nx - current.src_x) * resolution;
            const double ddy = (ny - current.src_y) * resolution;
            const float d = static_cast<float>(std::sqrt(ddx * ddx + ddy * ddy));

            if (d <= inflation_radius)
                frontier.push({d, nx, ny, current.src_x, current.src_y});
        }
    }
}

void TraversabilityMap::updateCostAtIter(const grid_map::GridMapIterator &it, double &a_cost)
{
    const grid_map::Index index = *it;

    // Hard occupancy: a directly-sensed obstacle is always maximally costly.
    if (global_map_->isValid(index, "occupancy_probability"))
    {
        if (global_map_->at("occupancy_probability", index) > 0.9f)
        {
            global_map_->at("cost", index) = 2.0;
            return;
        }
    }

    float slope = 0.0f;
    float step  = 0.0f;
    float inflated_occupancy = 0.0f;

    if (global_map_->isValid(index, "slope"))
        slope = global_map_->at("slope", index);

    if (global_map_->isValid(index, "step"))
        step = global_map_->at("step", index);

    if (global_map_->isValid(index, "inflated_occupancy"))
        inflated_occupancy = global_map_->at("inflated_occupancy", index);

    if (step < 0.0f)
        std::cerr << "Warning: Negative step value at index " << index.transpose() << std::endl;

    // ---- TUNABLE WEIGHTS ----
    const float w_slope = 1.0f;
    const float w_step  = 1.0f;
    const float w_occupancy = 1.0f;

    a_cost = w_slope * slope + w_step * step + w_occupancy * inflated_occupancy;
        
    float current_cost = global_map_->at("cost", index);
    global_map_->at("cost", index) = std::max(current_cost, static_cast<float>(a_cost));
}

void TraversabilityMap::normalizeLayersAndApplyCost()
{
    for (grid_map::GridMapIterator it(*global_map_); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index = *it;

        if (global_map_->isValid(index, "step") && max_step_iter_ > 0.0f)
        {
            float step = global_map_->at("step", index);
            global_map_->at("step", index) = step / max_step_iter_;
        }

        if (global_map_->isValid(index, "slope") && max_slope_iter_ > 0.0f)
        {
            float slope = global_map_->at("slope", index);
            global_map_->at("slope", index) = slope / max_slope_iter_;
        }

        double cost = 0.0;
        updateCostAtIter(it, cost);
    }
}

}; // namespace arena_demos
