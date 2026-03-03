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

//Eigen
#include <Eigen/Dense>

// Local
#include "arena_core/mapping/IMapping.h"

// grid_map
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/GridMap.hpp>

// System
#include <memory>


namespace arena_demos
{

/**
 * @brief Wrapper for a grid_map.
 * 
 * This struct is used as a safe wrapper to hold a shared pointer to a grid_map
 */
struct GridMapPtrWrapper
{
    std::shared_ptr<grid_map::GridMap> global_grid_map_;
}; // struct GridMapPtrWrapper


class TraversabilityCostCapability : public arena_core::ICapability
{
public:

    TraversabilityCostCapability(std::shared_ptr<GridMapPtrWrapper> a_global_map_wrapper)
        : global_map_wrapper_(a_global_map_wrapper)
    {
        if (!global_map_wrapper_)
            throw std::runtime_error("TraversabilityCostCapability: grid_map wrapper is null.");
    };

    ~TraversabilityCostCapability() override = default;

    double getTraversabilityCost(const Eigen::Vector2d& a_point) const
    {
        if (!global_map_wrapper_->global_grid_map_)
            throw std::runtime_error("TraversabilityCostCapability::getTraversabilityCost() global grid_map is null.");

        grid_map::Position position(a_point.x(), a_point.y());
        if (!global_map_wrapper_->global_grid_map_->isInside(position))
            throw std::out_of_range("TraversabilityCostCapability::getTraversabilityCost() point is outside of the grid map. Position x: " + std::to_string(position.x()) + ", Position y: " + std::to_string(position.y()));

        // Index from position
        grid_map::Index index;
        global_map_wrapper_->global_grid_map_->getIndex(position, index);
        return double(global_map_wrapper_->global_grid_map_->at("cost", index));
    };

    double getOccupancy(const Eigen::Vector2d &a_point) const
    {
        if (!global_map_wrapper_->global_grid_map_)
            throw std::runtime_error("TraversabilityCostCapability::getOccupancy() global grid_map is null.");

        grid_map::Position position(a_point.x(), a_point.y());
        if (!global_map_wrapper_->global_grid_map_->isInside(position))
            throw std::out_of_range("TraversabilityCostCapability::getOccupancy() point is outside of the grid map.");

        // Index from position
        grid_map::Index index;
        global_map_wrapper_->global_grid_map_->getIndex(position, index);
        return double(global_map_wrapper_->global_grid_map_->at("occupancy", index));
    }

    double getElevationAt(const Eigen::Vector2d &a_point) const
    {
        if (!global_map_wrapper_->global_grid_map_)
            throw std::runtime_error("TraversabilityCostCapability::getElevationAt() global grid_map is null.");

        grid_map::Position position(a_point.x(), a_point.y());
        if (!global_map_wrapper_->global_grid_map_->isInside(position))
            throw std::out_of_range("TraversabilityCostCapability::getElevationAt() point is outside of the grid map.");

        // Index from position
        grid_map::Index index;
        global_map_wrapper_->global_grid_map_->getIndex(position, index);
        return double(global_map_wrapper_->global_grid_map_->at("elevation", index));
    }

private:

    std::shared_ptr<GridMapPtrWrapper> global_map_wrapper_;

}; // class TraversabilityCostCapability


class TraversabilityCostmap : public arena_core::IMapping
{
public:

    TraversabilityCostmap()
        : global_map_wrapper_(std::make_shared<GridMapPtrWrapper>())
    {
        global_map_wrapper_->global_grid_map_ = nullptr;
    };

    ~TraversabilityCostmap() override = default;

    bool isOccupied(const Eigen::VectorXd& a_point) const override
    {
        if (!global_map_wrapper_->global_grid_map_)
            throw std::runtime_error("TraversabilityCostmap::isOccupied() global grid_map is null.");

        grid_map::Position position(a_point.x(), a_point.y());
        // Index from position
        grid_map::Index index;
        global_map_wrapper_->global_grid_map_->getIndex(position, index);
        if (!global_map_wrapper_->global_grid_map_->isInside(position))
            throw std::out_of_range("TraversabilityCostmap::isOccupied() point is outside of the grid map.");

        if (!global_map_wrapper_->global_grid_map_->isValid(index, "occupancy"))
            return false;

        double occupancy_value = global_map_wrapper_->global_grid_map_->at("occupancy", index);
        return (occupancy_value >= 0.9f) ? true : false;
    };

    bool isClearance(const Eigen::VectorXd& a_point, double a_clearance_radius) const override
    {
        if (!global_map_wrapper_->global_grid_map_)
            throw std::runtime_error("TraversabilityMap::isClearance() global grid_map is null.");

        grid_map::Position position(a_point.x(), a_point.y());
        if (!global_map_wrapper_->global_grid_map_->isInside(position))
            throw std::out_of_range("TraversabilityMap::isClearance() point is outside of the grid map.");

        double resolution = global_map_wrapper_->global_grid_map_->getResolution();
        int radius_cells = static_cast<int>(std::ceil(a_clearance_radius / resolution));

        grid_map::Index center_index;
        global_map_wrapper_->global_grid_map_->getIndex(position, center_index);

        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            {
                grid_map::Index neighbor_index = center_index + grid_map::Index(dx, dy);

                grid_map::Position neighbor_pos;
                global_map_wrapper_->global_grid_map_->getPosition(neighbor_index, neighbor_pos);
                if (!global_map_wrapper_->global_grid_map_->isInside(neighbor_pos)) continue;

                if (!global_map_wrapper_->global_grid_map_->isValid(neighbor_index, "occupancy"))
                    continue;

                if (global_map_wrapper_->global_grid_map_->at("occupancy", neighbor_index) >= 0.9f)
                    return false;
            }
        }

        return true;
    };

    Eigen::MatrixXd getMapBounds() const override
    {
        if (!global_map_wrapper_->global_grid_map_)
            throw std::runtime_error("TraversabilityMap::getMapBounds() global grid_map is null.");

        Eigen::MatrixXd bounds(2, 2);
        bounds(0, 0) = global_map_wrapper_->global_grid_map_->getPosition().x() - global_map_wrapper_->global_grid_map_->getLength().x() / 2.0; // min_x
        bounds(0, 1) = global_map_wrapper_->global_grid_map_->getPosition().y() - global_map_wrapper_->global_grid_map_->getLength().y() / 2.0; // min_y
        bounds(1, 0) = global_map_wrapper_->global_grid_map_->getPosition().x() + global_map_wrapper_->global_grid_map_->getLength().x() / 2.0; // max_x
        bounds(1, 1) = global_map_wrapper_->global_grid_map_->getPosition().y() + global_map_wrapper_->global_grid_map_->getLength().y() / 2.0; // max_y

        return bounds;
    };

    std::shared_ptr<grid_map::GridMap> getGlobalMap() const { return global_map_wrapper_->global_grid_map_; };

    void setGlobalMap(std::shared_ptr<grid_map::GridMap> a_global_map)
    {
        if (a_global_map)
            global_map_wrapper_->global_grid_map_ = a_global_map;

        // Check if the capability is registered
        auto cap = getCapability<TraversabilityCostCapability>();
        if (!cap)
        {
            // Register the capability if not already registered
            registerCapability(std::make_shared<TraversabilityCostCapability>(global_map_wrapper_));
        }
    }

private:

    std::shared_ptr<GridMapPtrWrapper> global_map_wrapper_;

}; // class TraversabilityCostmap

}; // namespace arena_demos
