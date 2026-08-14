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


// Packs a cell index into a single key for O(1) average set membership
// checks (grid_map::Index has no std::hash specialization).
int64_t packIndex(const grid_map::Index &index)
{
    return (static_cast<int64_t>(index.x()) << 32) | static_cast<uint32_t>(index.y());
}


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

    // Cache raw pointers into global_map_'s own layer storage. These stay
    // valid for the lifetime of global_map_: it is created exactly once,
    // here, and never has layers added to or erased from it afterward.
    elevation_mean_ = &global_map_->get("elevation_mean");
    elevation_variance_ = &global_map_->get("elevation_variance");
    elevation_num_measurements_ = &global_map_->get("elevation_num_measurements");
    elevation_reject_streak_ = &global_map_->get("elevation_reject_streak");
    step_ = &global_map_->get("step");
    slope_ = &global_map_->get("slope");
    occupancy_logodds_ = &global_map_->get("occupancy_logodds");
    occupancy_probability_ = &global_map_->get("occupancy_probability");
    inflated_occupancy_ = &global_map_->get("inflated_occupancy");
    cost_ = &global_map_->get("cost");

    elevation_mean_->setConstant(std::numeric_limits<float>::quiet_NaN());
    elevation_variance_->setConstant(std::numeric_limits<float>::quiet_NaN());
    elevation_num_measurements_->setConstant(0.0f);
    elevation_reject_streak_->setConstant(0.0f);
    step_->setConstant(std::numeric_limits<float>::quiet_NaN());
    slope_->setConstant(std::numeric_limits<float>::quiet_NaN());
    occupancy_logodds_->setZero();
    occupancy_probability_->setConstant(0.0f);
    inflated_occupancy_->setConstant(0.0f);
    cost_->setConstant(static_cast<float>(config_.unknown_cost));

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

void TraversabilityMap::updateMap(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &a_ground_cloud,
                                   const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &a_non_ground_cloud)
{
    if (!global_map_initialized_)
        return;

    if (!a_ground_cloud && !a_non_ground_cloud)
        return;

    // Scratch buffers reused across scans: clear() keeps prior capacity.
    changed_cells_.clear();
    changed_cells_lookup_.clear();

    if (a_ground_cloud)
        updateMap(a_ground_cloud, changed_cells_, true);

    if (a_non_ground_cloud)
        updateMap(a_non_ground_cloud, changed_cells_);

    for (const auto &index : changed_cells_)
    {
        updateStepAtIndex(index);
        updateSlopeAtIndex(index);
        reconcileOccupancyWithTerrain(index);
    }

    computeInflatedOccupancy();

    const grid_map::Size map_size = global_map_->getSize();
    for (int x = 0; x < map_size(0); ++x)
    {
        for (int y = 0; y < map_size(1); ++y)
        {
            grid_map::Index index(x, y);
            double cost = 0.0;
            updateCostAtIndex(index, cost);
        }
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
        grid_map::Index index;
        if (!global_map_->getIndex(pos, index)) continue;

        if (robot_bb_.isInside(local_map_pose_, pos)) continue;

        if (!is_ground)
            updateOccupancyRay(start, pos, a_changed_cells);
        else
        {
            const bool terrain_reliable = (*elevation_num_measurements_)(index.x(), index.y()) >= kReliableMeasurementCount;
            const bool step_confirms_obstacle = !std::isnan((*step_)(index.x(), index.y())) && (*step_)(index.x(), index.y()) >= config_.max_step;
            const bool slope_confirms_obstacle = !std::isnan((*slope_)(index.x(), index.y())) && (*slope_)(index.x(), index.y()) >= config_.max_slope;

            // A ground label is authoritative and should immediately override
            // noise-driven false occupancy, unless the terrain has already
            // reliably confirmed a real obstacle here, in which case a single
            // stray ground-labeled return is more likely to be the erroneous
            // observation and must not erase confirmed evidence.
            if (!(terrain_reliable && (step_confirms_obstacle || slope_confirms_obstacle)))
            {
                (*occupancy_logodds_)(index.x(), index.y()) = 0.0f;
                (*occupancy_probability_)(index.x(), index.y()) = 0.0f;
                (*inflated_occupancy_)(index.x(), index.y()) = 0.0f;
            }
        }

        updateElevationAtIndex(index, p.z, a_changed_cells);
    }
}

void TraversabilityMap::markCellChanged(const grid_map::Index &index, std::vector<grid_map::Index> &a_changed_cells)
{
    if (changed_cells_lookup_.insert(packIndex(index)).second)
        a_changed_cells.push_back(index);
}

void TraversabilityMap::updateElevationAtIndex(const grid_map::Index &index, const float z, std::vector<grid_map::Index> &a_changed_cells)
{
    if (std::isnan((*elevation_mean_)(index.x(), index.y())))
    {
        (*elevation_mean_)(index.x(), index.y()) = z;
        (*elevation_variance_)(index.x(), index.y()) = 0.1f;
        (*elevation_num_measurements_)(index.x(), index.y()) = 1.0f;
        (*elevation_reject_streak_)(index.x(), index.y()) = 0.0f;

        // A brand-new cell needs its step/slope computed too. Otherwise a
        // cell that never accumulates another measurement (single-pass
        // coverage) would stay frozen at its layer-init "unknown" default
        // forever. This push is deliberately unguarded: a non-ground point's
        // ray endpoint may already have inserted this same index earlier in
        // this same point's processing, and a duplicate entry here is
        // harmless since the changed-cell consumers below are idempotent.
        changed_cells_lookup_.insert(packIndex(index));
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

    float mean = (*elevation_mean_)(index.x(), index.y());
    float variance = (*elevation_variance_)(index.x(), index.y());

    // Prevent variance from collapsing to zero.
    variance += process_noise;

    // Innovation
    const float innovation = z - mean;

    // Innovation covariance
    const float S = variance + measurement_variance;

    // Optional statistical gating
    if (std::abs(innovation) > 3.0f * std::sqrt(S))
    {
        const float reject_streak = (*elevation_reject_streak_)(index.x(), index.y()) + 1.0f;

        if (reject_streak < kMaxConsecutiveRejections)
        {
            (*elevation_reject_streak_)(index.x(), index.y()) = reject_streak;
            return;
        }

        // The established mean has been consistently contradicted by new
        // data. Treat this cell as unobserved and re-anchor on the new
        // measurement rather than rejecting it too.
        (*elevation_mean_)(index.x(), index.y()) = z;
        (*elevation_variance_)(index.x(), index.y()) = 0.1f;
        (*elevation_num_measurements_)(index.x(), index.y()) = 1.0f;
        (*elevation_reject_streak_)(index.x(), index.y()) = 0.0f;

        markCellChanged(index, a_changed_cells);
        return;
    }

    (*elevation_reject_streak_)(index.x(), index.y()) = 0.0f;

    // Kalman gain
    const float K = variance / S;

    const float updated_mean = mean + (K * innovation);

    // Read the pre-increment count so we can tell whether this cell is still
    // maturing (the increment happens below).
    const float prev_num_measurements = (*elevation_num_measurements_)(index.x(), index.y());

    // A cell is flagged as "changed" (triggering a step/slope recompute) if
    // EITHER it is still maturing (below the reliability bar), OR, once mature, this update
    // is a statistically significant jump in the filtered mean (a real
    // terrain change after the cell had already settled).
    const bool is_still_maturing = prev_num_measurements <= kReliableMeasurementCount;
    const bool is_statistically_significant = !is_still_maturing && std::abs(updated_mean - mean) > 3.0f * std::sqrt(variance);

    if (is_still_maturing || is_statistically_significant)
        markCellChanged(index, a_changed_cells);

    // Bayesian update
    mean += K * innovation;
    variance *= (1.0f - K);

    (*elevation_mean_)(index.x(), index.y()) = mean;
    (*elevation_variance_)(index.x(), index.y()) = variance;
    (*elevation_num_measurements_)(index.x(), index.y()) += 1.0f;
}

void TraversabilityMap::updateOccupancyLogOddsAtIndex(const grid_map::Index &index, const bool occupied)
{
    float& L = (*occupancy_logodds_)(index.x(), index.y());

    if (occupied)
        L += HIT;
    else
        L += MISS;

    L = std::clamp(L, MIN_L, MAX_L);

    (*occupancy_probability_)(index.x(), index.y()) = 1.f / (1.f + std::exp(-L));
}

void TraversabilityMap::updateOccupancyRay(const grid_map::Position &start, const grid_map::Position &end, std::vector<grid_map::Index> &a_changed_cells)
{
    for (grid_map::LineIterator it(*global_map_, start, end); !it.isPastEnd(); ++it)
    {
        // Only add cells iter that had their log-odds updated to the changed_cells vector
        float current_log_odds = (*occupancy_logodds_)((*it).x(), (*it).y());
        updateOccupancyLogOddsAtIndex(*it, false);
        float updated_log_odds = (*occupancy_logodds_)((*it).x(), (*it).y());

        if (current_log_odds != updated_log_odds)
            markCellChanged(*it, a_changed_cells);
    }

    grid_map::Index endpoint;
    global_map_->getIndex(end, endpoint);

    // The endpoint of a non-ground return is a HIT regardless of how many
    // elevation measurements have landed on this exact discretized cell.
    float current_log_odds = (*occupancy_logodds_)(endpoint.x(), endpoint.y());
    updateOccupancyLogOddsAtIndex(endpoint, true);
    float updated_log_odds = (*occupancy_logodds_)(endpoint.x(), endpoint.y());

    if (current_log_odds != updated_log_odds)
        markCellChanged(endpoint, a_changed_cells);
}

void TraversabilityMap::updateStepAtIndex(const grid_map::Index &index)
{
    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    // If center cell invalid → step invalid
    if (std::isnan((*elevation_mean_)(index.x(), index.y())))
    {
        (*step_)(index.x(), index.y()) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    if ((*elevation_num_measurements_)(index.x(), index.y()) < kReliableMeasurementCount)
        return;

    const float z_center = (*elevation_mean_)(index.x(), index.y());
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

            if (std::isnan((*elevation_mean_)(neighbor.x(), neighbor.y())))
                continue;

            // A neighbor's own estimate must also be reliable, not merely
            // present. An immature, still-noisy neighbor (e.g. one raw
            // measurement) can otherwise inject a spurious step into an
            // already-reliable center cell's reading.
            if ((*elevation_num_measurements_)(neighbor.x(), neighbor.y()) < kReliableMeasurementCount)
                continue;

            const float z_neighbor = (*elevation_mean_)(neighbor.x(), neighbor.y());

            const float dz = std::abs(z_center - z_neighbor);

            if (dz > max_step)
                max_step = dz;
        }
    }

    if (max_step > 10.0f)   // Unreasonably large step this reduces visualization capabilities
        max_step = 10.0f;

    (*step_)(index.x(), index.y()) = std::abs(max_step);
}

void TraversabilityMap::updateSlopeAtIndex(const grid_map::Index &index)
{
    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    if (std::isnan((*elevation_mean_)(index.x(), index.y())))
    {
        (*slope_)(index.x(), index.y()) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    if ((*elevation_num_measurements_)(index.x(), index.y()) < kReliableMeasurementCount)
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

            if (std::isnan((*elevation_mean_)(neighbor.x(), neighbor.y())))
                continue;

            // See updateStepAtIndex: an immature, still-noisy neighbor must
            // not be allowed to inject a spurious gradient into an
            // already-reliable center cell's fitted plane.
            if ((*elevation_num_measurements_)(neighbor.x(), neighbor.y()) < kReliableMeasurementCount)
                continue;

            double x = dx * resolution;
            double y = dy * resolution;
            double z = (*elevation_mean_)(neighbor.x(), neighbor.y());

            points.emplace_back(x, y, z);
        }
    }

    if (points.size() < 6)
    {
        (*slope_)(index.x(), index.y()) = std::numeric_limits<float>::quiet_NaN();
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

    (*slope_)(index.x(), index.y()) = std::abs(static_cast<float>(slope));
}

void TraversabilityMap::reconcileOccupancyWithTerrain(const grid_map::Index &index)
{
    if (std::isnan((*occupancy_probability_)(index.x(), index.y())) || (*occupancy_probability_)(index.x(), index.y()) <= config_.occupancy_threshold)
        return;

    // Not enough terrain samples yet to vouch either way. Leave the
    // occupancy belief alone rather than clearing it on incomplete evidence.
    if ((*elevation_num_measurements_)(index.x(), index.y()) < kReliableMeasurementCount)
        return;

    const bool step_is_safe = !std::isnan((*step_)(index.x(), index.y())) && (*step_)(index.x(), index.y()) < config_.max_step;
    const bool slope_is_safe = !std::isnan((*slope_)(index.x(), index.y())) && (*slope_)(index.x(), index.y()) < config_.max_slope;

    // Only clear a HIT-driven occupancy belief when the terrain shows
    // neither a discrete step nor a hazardous slope
    if (step_is_safe && slope_is_safe)
    {
        (*occupancy_logodds_)(index.x(), index.y()) = 0.0f;
        (*occupancy_probability_)(index.x(), index.y()) = 0.0f;
        (*inflated_occupancy_)(index.x(), index.y()) = 0.0f;
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

    grid_map::Matrix &inflated = *inflated_occupancy_;
    inflated.setZero();

    const grid_map::Matrix &occupancy = *occupancy_probability_;

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
    if (!std::isnan((*occupancy_probability_)(index.x(), index.y())))
    {
        if ((*occupancy_probability_)(index.x(), index.y()) > config_.occupancy_threshold)
            constraint_cost = config_.constraint_cost;
    }

    // A cell's terrain (slope/step) is only trustworthy once it has
    // accumulated enough elevation measurements
    const bool terrain_reliable = (*elevation_num_measurements_)(index.x(), index.y()) > kReliableMeasurementCount;

    float slope = 0.0f;
    float step  = 0.0f;
    float inflated_occupancy = 0.0f;

    if (terrain_reliable && !std::isnan((*slope_)(index.x(), index.y())))
    {
        slope = (*slope_)(index.x(), index.y());
        if (slope > config_.max_slope)
            constraint_cost = config_.constraint_cost;
        slope = std::clamp(static_cast<float>(slope / config_.max_slope), 0.0f, 1.0f);
    }

    if (terrain_reliable && !std::isnan((*step_)(index.x(), index.y())))
    {
        step = (*step_)(index.x(), index.y());
        if (step > config_.max_step)
            constraint_cost = config_.constraint_cost;
        step = std::clamp(step / config_.max_step, 0.0, 1.0);
    }

    if (!std::isnan((*inflated_occupancy_)(index.x(), index.y())))
        inflated_occupancy = (*inflated_occupancy_)(index.x(), index.y());

    const double terrain_component = terrain_reliable ? static_cast<double>(config_.slope_weight * slope + config_.step_weight * step) : config_.unknown_cost;

    a_cost = terrain_component + config_.occupancy_weight * inflated_occupancy;

    // Hard constraint violations (near-vertical slope, un-traversable step,
    // or a directly-sensed obstacle) must dominate the soft, weighted cost.
    a_cost = std::max(a_cost, constraint_cost);
    (*cost_)(index.x(), index.y()) = a_cost;
}

}; // namespace arena_demos
