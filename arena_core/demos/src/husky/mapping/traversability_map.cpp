#include "husky/mapping/traversability_map.hpp"

// System
#include <unordered_set>


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
    global_map_ = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"elevation"});
    global_map_->add("step");
    global_map_->add("count");
    global_map_->add("sum");
    global_map_->add("slope");
    global_map_->add("occupancy");
    global_map_->add("cost");
    global_map_->setFrameId("world");

    global_map_->setGeometry(grid_map::Length(200.0, 200.0), 0.3, a_center);
    (*global_map_)["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["step"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["count"].setZero();
    (*global_map_)["sum"].setZero();
    (*global_map_)["slope"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["occupancy"].setConstant(std::numeric_limits<float>::quiet_NaN());
    (*global_map_)["cost"].setConstant(std::numeric_limits<double>::quiet_NaN());

    // ----- LOCAL MAP -----
    local_map_ = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"elevation"});
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

    for (const auto & p : a_cloud.points)
    {
        if (!std::isfinite(p.z)) continue;

        grid_map::Position pos(p.x, p.y);
        if (!global_map_->isInside(pos)) continue;

        if (robot_bb_.isInside(local_map_pose_, pos)) continue;

        grid_map::Index index;
        global_map_->getIndex(pos, index);

        global_map_->at("sum", index) += p.z;
        global_map_->at("count", index) += 1.0f;
    }

    double max_cost = 0.0;
    // Update elevation layer
    for (grid_map::GridMapIterator it(*global_map_); !it.isPastEnd(); ++it)
    {
        float count = global_map_->at("count", *it);

        if (count > 0.0f)
            global_map_->at("elevation", *it) = global_map_->at("sum", *it) / count;

        updateStepAtIter(it);
        updateSlopeAtIter(it);
        updateOccupancyAtIter(it);

        double cost = 0.0;
        updateCostAtIter(it, cost);

        if (cost > max_cost)
            max_cost = cost;
    }

    //normalizeCosts(max_cost);
}

void TraversabilityMap::updateStepAtIter(const grid_map::GridMapIterator &it)
{
    const grid_map::Index index = *it;

    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    // If center cell invalid → step invalid
    if (!global_map_->isValid(index, "elevation"))
    {
        global_map_->at("step", index) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    const float z_center = global_map_->at("elevation", index);
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

            if (!global_map_->isValid(neighbor, "elevation"))
                continue;

            const float z_neighbor = global_map_->at("elevation", neighbor);

            const float dz = std::abs(z_center - z_neighbor);

            if (dz > max_step)
                max_step = dz;
        }
    }

    global_map_->at("step", index) = max_step;
}

void TraversabilityMap::updateSlopeAtIter(const grid_map::GridMapIterator &it)
{
    const grid_map::Index index = *it;

    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    if (!global_map_->isValid(index, "elevation"))
    {
        global_map_->at("slope", index) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

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

            if (!global_map_->isValid(neighbor, "elevation"))
                continue;

            double x = dx * resolution;
            double y = dy * resolution;
            double z = global_map_->at("elevation", neighbor);

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
}

void TraversabilityMap::updateOccupancyAtIter(const grid_map::GridMapIterator &it)
{
    const grid_map::Index index = *it;

    grid_map::Position pos;
    global_map_->getPosition(index, pos);
    if (!global_map_->isInside(pos)) return;

    if (!global_map_->isValid(index, "elevation"))
    {
        global_map_->at("occupancy", index) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    const float z_center = global_map_->at("elevation", index);
    bool occupied = false;

    // Occupancy according to step
    if (global_map_->isValid(index, "step"))
    {
        const float step = global_map_->at("step", index);
        if (step > 0.5f)   // ← tune this threshold
            occupied = true;
    }

    global_map_->at("occupancy", index) = occupied ? 1.0f : 0.0f;
}

void TraversabilityMap::updateCostAtIter(const grid_map::GridMapIterator &it, double &a_cost)
{
    const grid_map::Index index = *it;

    if (!global_map_->isValid(index, "elevation"))
    {
        global_map_->at("cost", index) = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    // Hard occupancy
    if (global_map_->isValid(index, "occupancy"))
    {
        if (global_map_->at("occupancy", index) > 0.5f)
        {
            global_map_->at("cost", index) = std::numeric_limits<float>::quiet_NaN();
            return;
        }
    }

    float slope = 0.0f;
    float step  = 0.0f;

    if (global_map_->isValid(index, "slope"))
        slope = global_map_->at("slope", index);

    if (global_map_->isValid(index, "step"))
        step = global_map_->at("step", index);

    // ---- TUNABLE WEIGHTS ----
    const float w_slope = 10.0f;
    const float w_step  = 50.0f;

    a_cost = w_slope * slope + w_step * step;

    if (!std::isfinite(global_map_->at("cost", index)))
        a_cost += 1000.0;   // Penalize unknown areas

    global_map_->at("cost", index) = a_cost;
}

void TraversabilityMap::normalizeCosts(const double a_max_cost)
{
    if (a_max_cost <= 0.0)
        return;

    for (grid_map::GridMapIterator it(*global_map_); !it.isPastEnd(); ++it)
    {
        grid_map::Index index = *it;

        if (!global_map_->isValid(index, "cost"))
            continue;

        double cost = global_map_->at("cost", index);

        cost /= a_max_cost;

        global_map_->at("cost", index) = cost;
    }
}


}; // namespace arena_demos
