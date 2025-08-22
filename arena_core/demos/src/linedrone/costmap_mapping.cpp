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

#include "linedrone/costmap_mapping.h"


namespace arena_demos
{

CostmapMapping::CostmapMapping()
: octree_(nullptr), color_octree_wrapper_(std::make_shared<ColorOctreeWrapper>())
{
    color_octree_wrapper_->color_octree_ = nullptr;
}


bool CostmapMapping::isOccupied(const Eigen::VectorXd& point) const
{
    if (!octree_)
        throw std::runtime_error("CostmapMapping::isOccupied => Octree is not set. Cannot check occupancy.");

    if (point.size() != 3)
        throw std::runtime_error("CostmapMapping::isOccupied => Point must be 3-dimensional.");

    octomap::point3d query(point[0], point[1], point[2]);
    octomap::OcTreeNode* node = octree_->search(query);

    if (node == nullptr)
        return false; // Unknown space is considered free

    return octree_->isNodeOccupied(node);
}

bool CostmapMapping::isOccupiedRayTracing(const Eigen::VectorXd& point1, const Eigen::VectorXd& point2) const
{
    octomap::point3d start(point1[0], point1[1], point1[2]);
    octomap::point3d end(point2[0], point2[1], point2[2]);

    octomap::point3d_collection collection_ray;
    if (color_octree_wrapper_->color_octree_->computeRay(start, end, collection_ray))
    {
        for (const auto& point : collection_ray)
        {
            octomap::ColorOcTreeNode* node = color_octree_wrapper_->color_octree_->search(point);
            if (node)
            {
                if (color_octree_wrapper_->color_octree_->isNodeOccupied(*node))
                {
                    return true; // Occupied space found along the ray
                }
            }
        }
    }

    return false; // No occupied space found along the ray
}

bool CostmapMapping::isClearance(const Eigen::VectorXd& point, double clearance) const
{
    if (!octree_)
        throw std::runtime_error("CostmapMapping::isClearance => Octree is not set. Cannot check clearance.");

    if (point.size() != 3)
        throw std::runtime_error("CostmapMapping::isClearance => Point must be 3-dimensional.");

    if (clearance < 0)
        throw std::runtime_error("CostmapMapping::isClearance => Clearance must be non-negative.");

    octomap::point3d center(point[0], point[1], point[2]);

    // Define the bounding box around the robot's center position
    const octomap::point3d bbx_min(center.x() - clearance, center.y() - clearance, center.z() - clearance);
    const octomap::point3d bbx_max(center.x() + clearance, center.y() + clearance, center.z() + clearance);

    // Iterate over all occupied voxels in the bounding box
    for (auto it = octree_->begin_leafs_bbx(bbx_min, bbx_max); it != octree_->end_leafs_bbx(); ++it)
    {
        if (octree_->isNodeOccupied(*it))
            return false;
    }

    return true; // Clear of obstacles within the specified clearance
}

Eigen::MatrixXd CostmapMapping::getMapBounds() const
{
    if (!octree_)
        throw std::runtime_error("CostmapMapping::getMapBounds => Octree is not set. Cannot get map bounds.");
    
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it)
    {
        const octomap::point3d& pt = it.getCoordinate();

        if (pt.x() < min_x) min_x = pt.x();
        if (pt.y() < min_y) min_y = pt.y();
        if (pt.z() < min_z) min_z = pt.z();
        if (pt.x() > max_x) max_x = pt.x();
        if (pt.y() > max_y) max_y = pt.y();
        if (pt.z() > max_z) max_z = pt.z();
    }

    Eigen::MatrixXd bounds(2, 3);
    bounds << min_x, min_y, min_z,
              max_x, max_y, max_z;

    return bounds;
}


}; // namespace arena_demos
