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


// System
#include <memory>
#include <queue>
#include <sstream>
#include <iostream>
#include <stdlib.h>

// Octomap
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>


namespace arena_demos
{

struct OcTreeKeyCompare 
{
    bool operator()(const octomap::OcTreeKey& key1, const octomap::OcTreeKey& key2) const 
    {
        if (key1[0] != key2[0])
            return key1[0] < key2[0];
        if (key1[1] != key2[1])
            return key1[1] < key2[1];
        return key1[2] < key2[2];
    }
};

struct OctreeKeyPairCompare 
{
    using value_type = std::pair<double, octomap::OcTreeKey>; // Define value_type

    bool operator()(const value_type& a, const value_type& b) const 
    {
        return a.first > b.first;  // Compare based on the double part of the pair
    }
};

class Costmap3D
{
public:
    Costmap3D();
    ~Costmap3D();

    // Getters
    std::shared_ptr<octomap::OcTree> getOctree() const { return octree_; };
    std::shared_ptr<octomap::ColorOcTree> getColorOctree() const { return color_octree_; };
    float getInflationRadius() const { return inflation_radius_; };
    float getMinInfluenceRadius() const { return min_influence_radius_; };
    float getMaxInfluenceRadius() const { return max_influence_radius_; };

    // Setters
    void setOctree(std::shared_ptr<octomap::OcTree> tree);
    void setInflationRadius(float inflation_radius) { inflation_radius_ = inflation_radius; }
    void setMinInfluenceRadius(float min_influence_radius) { min_influence_radius_ = min_influence_radius; }
    void setMaxInfluenceRadius(float max_influence_radius) { max_influence_radius_ = max_influence_radius; }
    
private:

    // User-defined methods
    void inflateOctree(float inflation_radius, float min_influence_radius, float max_influence_radius);
    float computeCollisionCost(float node_distance, float min_influence_radius, float max_influence_radius);
    
    // User-defined attributes
    std::shared_ptr<octomap::OcTree> octree_;
    std::shared_ptr<octomap::ColorOcTree> color_octree_;
    float inflation_radius_ = 1.5;
    float min_influence_radius_ = 1.0;
    float max_influence_radius_ = 2.0;

}; // class Costmap3D

}; // namespace arena_demos
