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
#include "arena_core/math/utils/pcl_utils.h"

// External Libraries
// PCL
// PCL
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
// Eigen
#include <Eigen/Dense>


namespace arena_core
{

/**
 * \brief Wrapper for an oriented bounding box (OBB) that provides methods to compute the OBB from a point cloud and calculate distances from points to the OBB.
 * This class encapsulates the centroid, center, dimensions, and rotational matrix of the OBB.
 */
struct OrientedBoundingBoxWrapper
{

    Eigen::Vector3d centroid_; // Centroid of the point cloud
    Eigen::Vector3d center_; // Center of the OBB in the original coordinate frame
    Eigen::Vector3d dimensions_; // Dimensions of the OBB (width, height, depth)
    Eigen::Matrix3d rotational_matrix_; // Rotation matrix that aligns the OBB with the principal axes of the point cloud

    /**
     * \brief Constructs the OBB from a given point cloud.
     * \param cloud The input point cloud from which to compute the OBB.
     * \throws std::runtime_error if the input cloud is empty.
     */
    void constructOBBFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        pcl::computeCentroidAndOBB(cloud, centroid_, center_, dimensions_, rotational_matrix_);
    }

    /**
     * \brief Computes the distance from a point to the OBB and returns the closest point on the OBB's surface.
     * \param point The input point for which to compute the distance.
     * \param closest_point_on_obb Output parameter that will hold the closest point on the OBB's surface.
     * \return The positive distance between the input point and the closest point on the OBB's surface.
     */
    double distanceFromOBB(const Eigen::Vector3d& point, Eigen::Vector3d& closest_point_on_obb) const
    {
        // Transform the point into the local coordinate system of the OBB
        Eigen::Vector3d local_point = rotational_matrix_.transpose() * (point - center_);

        // Check if the point is inside the OBB
        bool inside_obb = true;
        for (int i = 0; i < 3; ++i) {
            if (std::abs(local_point[i]) > dimensions_[i] / 2) {
                inside_obb = false;
                break;
            }
        }

        if (inside_obb)
        {
            // If the point is inside the OBB, calculate the distance to the nearest face of the OBB
            // TODO: Implement this
            /*Eigen::Vector3d closest_point = center_;
            for (int i = 0; i < 3; ++i)
            {
                double dist = local_point.dot(rotational_matrix_.col(i));
                if (dist > dimensions_[i] / 2)
                    dist = dimensions_[i] / 2;

                if (dist < -dimensions_[i] / 2)
                    dist = -dimensions_[i] / 2;

                // Transform the closest point back into the global coordinate system
                closest_point += dist * rotational_matrix_.col(i);
            }

            closest_point_on_obb = closest_point;

            std::cout << "Distance: " << (point - closest_point).norm() << std::endl;
            // Return the positive distance between the input point and the closest point on the OBB's surface
            return (point - closest_point).norm();*/
            return 0.0;
        }
        else
        {
            // If the point is outside the OBB, proceed as before and calculate the distance to the closest point on the OBB's surface
            Eigen::Vector3d closest_point;
            for (int i = 0; i < 3; ++i)
            {
                double value = std::min(std::max(local_point[i], -dimensions_[i] / 2), dimensions_[i] / 2);
                closest_point(i) = value;
            }

            // Transform the closest point back into the global coordinate system
            closest_point = center_ + rotational_matrix_ * closest_point;
            closest_point_on_obb = closest_point;

            // Return the positive distance between the input point and the closest point on the OBB's surface
            return (point - closest_point).norm();
        }
    }

}; // OrientedBoundingBoxWrapper

}; // arena_core
