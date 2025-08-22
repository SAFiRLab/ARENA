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

#pragma once


// Local
#include "arena_core/mapping/IMapping.h"

// External Libraries
// Octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>


namespace arena_demos
{

/**
 * @brief Wrapper for a color octree.
 * 
 * This struct is used as a safe wrapper to hold a shared pointer to a color octree
 */
struct ColorOctreeWrapper
{
    std::shared_ptr<octomap::ColorOcTree> color_octree_;
}; // struct ColorOctreeWrapper


/**
 * @brief Capability for cost SDF (Signed Distance Function) calculations.
 * 
 * This capability provides methods to calculate collision costs and occupancy based on a color octree.
 * It is used to extend the mapping functionalities with cost-related capabilities.
 */
class CostSDFCapability : public arena_core::ICapability
{
public:

    CostSDFCapability(std::shared_ptr<ColorOctreeWrapper> color_octree_wrapper)
        : color_octree_wrapper_(color_octree_wrapper)
    {
        if (!color_octree_wrapper_)
            throw std::runtime_error("CostSDFCapability: Color octree wrapper is null.");
    }

    ~CostSDFCapability() override = default;

    /*********** Capability Methods ***********/
    /**
     * @brief Get the collision cost at a given point.
     * 
     * This method retrieves the collision cost at a specified point based on the color octree.
     * The cost is derived from the blue channel of the color information, where higher blue values
     * indicate lower collision risk.
     * 
     * @param point The point at which to retrieve the collision cost.
     * @return The collision cost at the specified point, ranging from 0.0 (free space) to 1.0 (high collision risk).
     */
    double getCollisionCost(const Eigen::Vector3d& point) const
    {
        if (!color_octree_wrapper_->color_octree_)
            throw std::runtime_error("CostSDFCapability::getCollisionCost => Color octree is not set.");

        unsigned int tree_depth = color_octree_wrapper_->color_octree_->getTreeDepth();
        octomap::point3d query(point[0], point[1], point[2]);
        octomap::ColorOcTreeNode* node = color_octree_wrapper_->color_octree_->search(query, tree_depth);

        if (node)
        {
            octomap::ColorOcTreeNode::Color node_color = node->getColor();
            double blue = double(uint8_t(node_color.b));
            double node_collision_cost = 1.0 - (blue / 255.0); // Assuming blue channel indicates collision risk
            return node_collision_cost;
        }

        return 0.0; // Free space has zero cost
    };

    /**
     * @brief Get the occupancy at a given point.
     * 
     * This method retrieves the occupancy at a specified point based on the color octree.
     * The occupancy value indicates what's the likelihood of the point being occupied by an obstacle. (0-100%)
     * 
     * @param point The point at which to retrieve the occupancy.
     * @return The occupancy at the specified point
     */
    double getOccupancy(const Eigen::Vector3d& point) const
    {
        if (!color_octree_wrapper_->color_octree_)
            throw std::runtime_error("CostSDFCapability::getOccupancy => Color octree is not set.");

        unsigned int tree_depth = color_octree_wrapper_->color_octree_->getTreeDepth();
        octomap::point3d query(point[0], point[1], point[2]);
        octomap::ColorOcTreeNode* node = color_octree_wrapper_->color_octree_->search(query, tree_depth);

        if (node)
            return node->getOccupancy();

        return 0.0; // Free space has zero occupancy
    };

    /**
     * @brief Get the occupancy from ray tracing between two points.
     * 
     * This method performs ray tracing between two points in the color octree to determine the maximum occupancy
     * encountered along the ray. It is useful for assessing the occupancy along a path or between two locations.
     * 
     * @param origin The starting point of the ray.
     * @param end The ending point of the ray.
     * @return The maximum occupancy encountered along the ray.
     */
    double getOccupancyFromRayTracing(const Eigen::Vector3d& origin, const Eigen::Vector3d& end) const
    {
        if (!color_octree_wrapper_->color_octree_)
            throw std::runtime_error("CostSDFCapability::getOccupancyFromRayTracing => Color octree is not set.");

        unsigned int tree_depth = color_octree_wrapper_->color_octree_->getTreeDepth();
        octomap::point3d origin_pt(origin[0], origin[1], origin[2]);
        octomap::point3d end_pt(end[0], end[1], end[2]);
        octomap::point3d_collection ray_traced_points;

        double max_occupancy = 0.0;
        if (color_octree_wrapper_->color_octree_->computeRay(origin_pt, end_pt, ray_traced_points))
        {
            for (const auto& point : ray_traced_points)
            {
                octomap::ColorOcTreeNode* ray_node = color_octree_wrapper_->color_octree_->search(point, tree_depth);

                if (ray_node)
                {
                    double occupancy = ray_node->getOccupancy();
                    if (occupancy > max_occupancy)
                        max_occupancy = occupancy;
                }
            }
        }

        return max_occupancy;
    };


private:

    std::shared_ptr<ColorOctreeWrapper> color_octree_wrapper_;

}; // class CostSDFCapability


class CostmapMapping : public arena_core::IMapping
{
public:
    CostmapMapping();
    ~CostmapMapping() override = default;


    /**** CORE Methods ****/
    /**
     * @brief Check if the given point is occupied in the costmap.
     * 
     * This method checks if a point is occupied in the costmap, which can be used for collision checking.
     * 
     * @param point The point to check for occupancy.
     * @return true if the point is occupied, false otherwise.
     */
    bool isOccupied(const Eigen::VectorXd& point) const override;

    /**
     * @brief Check if the given point is occupied in the costmap using ray tracing.
     * 
     * This method checks if a point is occupied in the costmap using ray tracing, which can be used for collision checking.
     * 
     * @param point1 The first point of the ray to check for occupancy.
     * @param point2 The second point of the ray to check for occupancy.
     * @return true if the ray intersects an occupied space, false otherwise.
     */
    bool isOccupiedRayTracing(const Eigen::VectorXd& point1, const Eigen::VectorXd& point2) const override;

    /**
     * @brief Check if the given point is clear of obstacles within a specified clearance.
     * 
     * This method checks if a point is clear of obstacles within a specified clearance distance.
     * This is useful for ensuring that a point is not only unoccupied but also has a safe distance from obstacles.
     * 
     * @param point The point to check for clearance.
     * @param clearance The clearance distance to check against obstacles.
     * @return true if the point is clear of obstacles, false otherwise.
     */
    bool isClearance(const Eigen::VectorXd& point, double clearance) const override;

    /**
     * @brief Get the bounds of the mapping.
     * 
     * This method retrieves the bounds of the mapping, which can be used for various purposes such as visualization or planning.
     * 
     * @return A matrix representing the bounds of the mapping.
     */
    Eigen::MatrixXd getMapBounds() const override;


    /**** Setters ****/

    /**
     * @brief Set the octree for the costmap.
     * 
     * This method sets the octree used for occupancy and clearance checks in the costmap.
     * 
     * @param tree A shared pointer to the octree to set.
     */
    void setOctree(std::shared_ptr<octomap::OcTree> tree)
    {
        if (tree)
            octree_ = tree;
    }

    /**
     * @brief Set the color octree for the costmap.
     * 
     * This method sets the color octree used for cost calculations in the costmap.
     * 
     * @param color_tree A shared pointer to the color octree to set.
     */
    void setColorOctree(std::shared_ptr<octomap::ColorOcTree> color_tree)
    {
        if (color_tree)
            color_octree_wrapper_->color_octree_ = color_tree;

        // Check if the capability is registered
        auto cap = getCapability<CostSDFCapability>();
        if (!cap)
        {
            // Register the capability if not already registered
            registerCapability(std::make_shared<CostSDFCapability>(color_octree_wrapper_));
        }
    }

private:

    // User-defined methods

    // User-defined attributes
    std::shared_ptr<octomap::OcTree> octree_;
    std::shared_ptr<ColorOctreeWrapper> color_octree_wrapper_;

}; // class CostmapMapping

}; // namespace arena_demos
