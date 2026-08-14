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

// A cell must accumulate this many elevation measurements before its
// step/slope layers are considered statistically reliable 
constexpr float kReliableMeasurementCount = 15.0f;


namespace arena_demos
{

TraversabilityMap::TraversabilityMap(const TraversabilityMapConfig &a_config)
: global_map_(nullptr), local_map_(nullptr), robot_bb_(), config_(a_config)
{}

TraversabilityMap::TraversabilityMap(const RobotBoundingBox &a_robot_bb)
: global_map_(nullptr), local_map_(nullptr), robot_bb_(a_robot_bb), config_()
{}

TraversabilityMap::TraversabilityMap()
: global_map_(nullptr), local_map_(nullptr), robot_bb_(), config_()
{}

TraversabilityMap::TraversabilityMap(const RobotBoundingBox &a_robot_bb, const TraversabilityMapConfig &a_config)
: global_map_(nullptr), local_map_(nullptr), robot_bb_(a_robot_bb), config_(a_config)
{}

void TraversabilityMap::initializeMaps(const grid_map::Position &a_center)
{
    // ----- GLOBAL MAP -----
    global_map_ = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"elevation_mean"});
    global_map_->add("elevation_variance");
    global_map_->add("elevation_num_measurements");
    global_map_->add("elevation_reject_streak");
    global_map_->add("step");
    global_map_->add("slope");
    global_map_->add("occupancy_logodds");
    global_map_->add("occupancy_probability");
    global_map_->add("inflated_occupancy");
    global_map_->add("cost");
    global_map_->setFrameId(config_.root_frame);

    global_map_->setGeometry(grid_map::Length(config_.global_map_size_x, config_.global_map_size_y), config_.map_resolution, a_center);
    (*global_map_)["elevation_mean"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["elevation_variance"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["elevation_num_measurements"].setConstant(0.0f);
    (*global_map_)["elevation_reject_streak"].setConstant(0.0f);
    (*global_map_)["step"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["slope"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["occupancy_logodds"].setZero();
    (*global_map_)["occupancy_probability"].setConstant(0.0f);
    (*global_map_)["inflated_occupancy"].setConstant(0.0f);
    (*global_map_)["cost"].setConstant(static_cast<float>(config_.unknown_cost));

    // ----- LOCAL MAP -----
    local_map_ = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"elevation_mean"});
    local_map_->setFrameId(config_.root_frame);
    local_map_->setGeometry(grid_map::Length(config_.local_map_size_x, config_.local_map_size_y), config_.map_resolution);

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

void TraversabilityMap::updateMap(std::unordered_map<std::string, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> &a_clouds)
{
    if (!global_map_initialized_)
        return;
    
    if (a_clouds.empty())
        return;

    // vector of changed cells to update layers after processing all points
    std::vector<grid_map::Index> changed_cells;

    auto ground_it = a_clouds.find("ground");
    if (ground_it != a_clouds.end() && ground_it->second)
        updateMap(ground_it->second, changed_cells, true);

    auto non_ground_it = a_clouds.find("non_ground");
    if (non_ground_it != a_clouds.end() && non_ground_it->second)
        updateMap(non_ground_it->second, changed_cells);

    for (const auto &index : changed_cells)
    {
        updateStepAtIndex(index);
        updateSlopeAtIndex(index);
        reconcileOccupancyWithTerrain(index);
    }

    computeInflatedOccupancy();
    for (grid_map::GridMapIterator it(*global_map_); !it.isPastEnd(); ++it)
    {
        grid_map::Index index = *it;
        double cost = 0.0;
        updateCostAtIndex(index, cost);
    }
}

void TraversabilityMap::updateMap(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &a_cloud, std::vector<grid_map::Index> &a_changed_cells, const bool is_ground)
{
    if (!global_map_initialized_)
        return;

    if (a_cloud->empty())
        return;

    std::lock_guard<std::mutex> lock(map_mutex_);
    grid_map::Position start(local_map_pose_.x(), local_map_pose_.y());

    for (const auto &p : a_cloud->points)
    {
        if (!std::isfinite(p.z)) continue;

        grid_map::Position pos(p.x, p.y);
        if (!global_map_->isInside(pos)) continue;

        if (robot_bb_.isInside(local_map_pose_, pos)) continue;

        grid_map::Index index;
        global_map_->getIndex(pos, index);

        if (!is_ground)
            updateOccupancyRay(start, pos, a_changed_cells);
        else
        {
            const bool terrain_reliable = global_map_->at("elevation_num_measurements", index) >= kReliableMeasurementCount;
            const bool step_confirms_obstacle = global_map_->isValid(index, "step") && global_map_->at("step", index) >= config_.max_step;
            const bool slope_confirms_obstacle = global_map_->isValid(index, "slope") && global_map_->at("slope", index) >= config_.max_slope;

            // A ground label is authoritative and should immediately override
            // noise-driven false occupancy, unless the terrain has already
            // reliably confirmed a real obstacle here, in which case a single
            // stray ground-labeled return is more likely to be the erroneous
            // observation and must not erase confirmed evidence.
            if (!(terrain_reliable && (step_confirms_obstacle || slope_confirms_obstacle)))
            {
                global_map_->at("occupancy_logodds", index) = 0.0f;
                global_map_->at("occupancy_probability", index) = 0.0f;
                global_map_->at("inflated_occupancy", index) = 0.0f;
            }
        }

        updateElevationAtIndex(index, p.z, a_changed_cells);
    }
}

void TraversabilityMap::updateElevationAtIndex(const grid_map::Index &index, const float z, std::vector<grid_map::Index> &a_changed_cells)
{
    if (!global_map_->isValid(index, "elevation_mean"))
    {
        global_map_->at("elevation_mean", index) = z;
        global_map_->at("elevation_variance", index) = 0.1f;
        global_map_->at("elevation_num_measurements", index) = 1.0f;
        global_map_->at("elevation_reject_streak", index) = 0.0f;

        // A brand-new cell needs its step/slope computed too. Otherwise a
        // cell that never accumulates another measurement (single-pass
        // coverage) would stay frozen at its layer-init "unknown" default
        // forever.
        a_changed_cells.push_back(index);
        return;
    }

    constexpr float measurement_variance = 0.01f;   // Sensor noise (R)
    constexpr float process_noise = 1e-4f;          // Terrain evolution (Q)

    // If this many consecutive measurements disagree with the established
    // mean, the mean itself is the outlier (e.g. its very first
    // observation (accepted unconditionally above, with no gating at all)
    // was a spurious return that got locked in with a tight variance).
    // Without this, every subsequent *good* measurement keeps disagreeing
    // with that wrong baseline and gets rejected forever by the very gate
    // meant to protect it, permanently corrupting this cell's elevation --
    // and with it, every step/slope value derived from it.
    constexpr float kMaxConsecutiveRejections = 3.0f;

    float mean = global_map_->at("elevation_mean", index);
    float variance = global_map_->at("elevation_variance", index);

    // Prevent variance from collapsing to zero.
    variance += process_noise;

    // Innovation
    const float innovation = z - mean;

    // Innovation covariance
    const float S = variance + measurement_variance;

    auto idx_compare = std::find_if(a_changed_cells.begin(), a_changed_cells.end(), [&](const grid_map::Index& idx)
    {
        return (idx == index).all();
    });

    // Optional statistical gating
    if (std::abs(innovation) > 3.0f * std::sqrt(S))
    {
        const float reject_streak = global_map_->at("elevation_reject_streak", index) + 1.0f;

        if (reject_streak < kMaxConsecutiveRejections)
        {
            global_map_->at("elevation_reject_streak", index) = reject_streak;
            return;
        }

        // The established mean has been consistently contradicted by new
        // data. Treat this cell as unobserved and re-anchor on the new
        // measurement rather than rejecting it too.
        global_map_->at("elevation_mean", index) = z;
        global_map_->at("elevation_variance", index) = 0.1f;
        global_map_->at("elevation_num_measurements", index) = 1.0f;
        global_map_->at("elevation_reject_streak", index) = 0.0f;

        if (idx_compare == a_changed_cells.end())
            a_changed_cells.push_back(index);
        return;
    }

    global_map_->at("elevation_reject_streak", index) = 0.0f;

    // Kalman gain
    const float K = variance / S;

    const float updated_mean = mean + (K * innovation);

    // Read the pre-increment count so we can tell whether this cell is still
    // maturing (the increment happens below).
    const float prev_num_measurements = global_map_->at("elevation_num_measurements", index);

    // A cell is flagged as "changed" (triggering a step/slope recompute) if
    // EITHER it is still maturing (below the reliability bar), OR, once mature, this update
    // is a statistically significant jump in the filtered mean (a real
    // terrain change after the cell had already settled).
    const bool is_still_maturing = prev_num_measurements <= kReliableMeasurementCount;
    const bool is_statistically_significant = !is_still_maturing && std::abs(updated_mean - mean) > 3.0f * std::sqrt(variance);

    if (is_still_maturing || is_statistically_significant)
    {
        if (idx_compare == a_changed_cells.end())
            a_changed_cells.push_back(index);
    }

    // Bayesian update
    mean += K * innovation;
    variance *= (1.0f - K);

    global_map_->at("elevation_mean", index) = mean;
    global_map_->at("elevation_variance", index) = variance;
    global_map_->at("elevation_num_measurements", index) += 1.0f;
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

void TraversabilityMap::updateOccupancyRay(const grid_map::Position &start, const grid_map::Position &end, std::vector<grid_map::Index> &a_changed_cells)
{
    for (grid_map::LineIterator it(*global_map_, start, end); !it.isPastEnd(); ++it)
    {
        // Only add cells iter that had their log-odds updated to the changed_cells vector
        float current_log_odds = global_map_->at("occupancy_logodds", *it);
        updateOccupancyLogOddsAtIndex(*it, false);
        float updated_log_odds = global_map_->at("occupancy_logodds", *it);

        if (current_log_odds != updated_log_odds)
        {
            auto idx_compare = std::find_if(a_changed_cells.begin(), a_changed_cells.end(), [&](const grid_map::Index& idx)
            {
                return (idx == *it).all();
            });

            if (idx_compare == a_changed_cells.end())
                a_changed_cells.push_back(*it);
        }
    }

    grid_map::Index endpoint;
    global_map_->getIndex(end, endpoint);

    // The endpoint of a non-ground return is a HIT regardless of how many
    // elevation measurements have landed on this exact discretized cell.
    float current_log_odds = global_map_->at("occupancy_logodds", endpoint);
    updateOccupancyLogOddsAtIndex(endpoint, true);
    float updated_log_odds = global_map_->at("occupancy_logodds", endpoint);

    if (current_log_odds != updated_log_odds)
    {
        auto idx_compare = std::find_if(a_changed_cells.begin(), a_changed_cells.end(), [&](const grid_map::Index& idx)
        {
            return (idx == endpoint).all();
        });

        if (idx_compare == a_changed_cells.end())
            a_changed_cells.push_back(endpoint);
    }
}

void TraversabilityMap::updateStepAtIndex(const grid_map::Index &index)
{
    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    // If center cell invalid → step invalid
    if (!global_map_->isValid(index, "elevation_mean"))
    {
        global_map_->at("step", index) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    if (global_map_->at("elevation_num_measurements", index) < kReliableMeasurementCount)
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

            // A neighbor's own estimate must also be reliable, not merely
            // present. An immature, still-noisy neighbor (e.g. one raw
            // measurement) can otherwise inject a spurious step into an
            // already-reliable center cell's reading.
            if (global_map_->at("elevation_num_measurements", neighbor) < kReliableMeasurementCount)
                continue;

            const float z_neighbor = global_map_->at("elevation_mean", neighbor);

            const float dz = std::abs(z_center - z_neighbor);

            if (dz > max_step)
                max_step = dz;
        }
    }

    if (max_step > 10.0f)   // Unreasonably large step this reduces visualization capabilities
        max_step = 10.0f;
    
    global_map_->at("step", index) = std::abs(max_step);
}

void TraversabilityMap::updateSlopeAtIndex(const grid_map::Index &index)
{
    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    if (!global_map_->isValid(index, "elevation_mean"))
    {
        global_map_->at("slope", index) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    if (global_map_->at("elevation_num_measurements", index) < kReliableMeasurementCount)
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

            // See updateStepAtIndex: an immature, still-noisy neighbor must
            // not be allowed to inject a spurious gradient into an
            // already-reliable center cell's fitted plane.
            if (global_map_->at("elevation_num_measurements", neighbor) < kReliableMeasurementCount)
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

    global_map_->at("slope", index) = std::abs(static_cast<float>(slope));
}

void TraversabilityMap::reconcileOccupancyWithTerrain(const grid_map::Index &index)
{
    if (!global_map_->isValid(index, "occupancy_probability") || global_map_->at("occupancy_probability", index) <= config_.occupancy_threshold)
        return;

    // Not enough terrain samples yet to vouch either way. Leave the
    // occupancy belief alone rather than clearing it on incomplete evidence.
    if (global_map_->at("elevation_num_measurements", index) < kReliableMeasurementCount)
        return;

    const bool step_is_safe = global_map_->isValid(index, "step") && global_map_->at("step", index) < config_.max_step;
    const bool slope_is_safe = global_map_->isValid(index, "slope") && global_map_->at("slope", index) < config_.max_slope;

    // Only clear a HIT-driven occupancy belief when the terrain shows
    // neither a discrete step nor a hazardous slope
    if (step_is_safe && slope_is_safe)
    {
        global_map_->at("occupancy_logodds", index) = 0.0f;
        global_map_->at("occupancy_probability", index) = 0.0f;
        global_map_->at("inflated_occupancy", index) = 0.0f;
    }
}

void TraversabilityMap::computeInflatedOccupancy()
{
    // ---- TUNABLE PARAMETER ----
    constexpr double inflation_radius = 9.0;   // meters

    // Circumscribed radius of the robot's footprint: any cell within this
    // distance of a real obstacle is a position the robot's body cannot
    // occupy in ANY orientation without colliding, so it must read as just
    // as lethal as the sensed obstacle cell itself, not merely "high cost"
    // from the linear falloff below.
    const double robot_radius = 0.5 * std::sqrt(robot_bb_.x_width * robot_bb_.x_width + robot_bb_.y_width * robot_bb_.y_width);

    const double resolution = global_map_->getResolution();
    const grid_map::Size map_size = global_map_->getSize();
    const int rows = map_size(0);
    const int cols = map_size(1);
    const int num_cells = rows * cols;

    grid_map::Matrix &inflated = (*global_map_)["inflated_occupancy"];
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
            if (occupancy(x, y) >= config_.occupancy_threshold)
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

        // Flat lethal zone out to the robot's own footprint radius, then a
        // linear falloff from there out to inflation_radius
        const float decay = (current.distance <= robot_radius) ? 1.0f : 1.0f - static_cast<float>((current.distance - robot_radius) / (inflation_radius - robot_radius));
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

void TraversabilityMap::updateCostAtIndex(const grid_map::Index &index, double &a_cost)
{
    double constraint_cost = 0.0;
    // Hard occupancy: a directly-sensed obstacle is always maximally costly.
    if (global_map_->isValid(index, "occupancy_probability"))
    {
        if (global_map_->at("occupancy_probability", index) > config_.occupancy_threshold)
            constraint_cost = config_.constraint_cost;
    }

    // A cell's terrain (slope/step) is only trustworthy once it has
    // accumulated enough elevation measurements
    const bool terrain_reliable = global_map_->at("elevation_num_measurements", index) > kReliableMeasurementCount;

    float slope = 0.0f;
    float step  = 0.0f;
    float inflated_occupancy = 0.0f;

    if (terrain_reliable && global_map_->isValid(index, "slope"))
    {
        slope = global_map_->at("slope", index);
        if (slope > config_.max_slope)
            constraint_cost = config_.constraint_cost;
        slope = std::clamp(static_cast<float>(slope / config_.max_slope), 0.0f, 1.0f);
    }

    if (terrain_reliable && global_map_->isValid(index, "step"))
    {
        step = global_map_->at("step", index);
        if (step > config_.max_step)
            constraint_cost = config_.constraint_cost;
        step = std::clamp(step / config_.max_step, 0.0, 1.0);
    }

    if (global_map_->isValid(index, "inflated_occupancy"))
        inflated_occupancy = global_map_->at("inflated_occupancy", index);

    const double terrain_component = terrain_reliable ? static_cast<double>(config_.slope_weight * slope + config_.step_weight * step) : config_.unknown_cost;

    a_cost = terrain_component + config_.occupancy_weight * inflated_occupancy;

    // Hard constraint violations (near-vertical slope, un-traversable step,
    // or a directly-sensed obstacle) must dominate the soft, weighted cost.
    a_cost = std::max(a_cost, constraint_cost);
    global_map_->at("cost", index) = a_cost;
}

}; // namespace arena_demos
