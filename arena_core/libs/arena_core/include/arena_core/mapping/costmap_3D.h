#pragma once

// System
#include <memory>
#include <queue>
#include <sstream>
#include <iostream>
#include <stdlib.h>

// Octomap
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>


namespace arena_core
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


    // Functions
    void inflateOctree(float inflation_radius, float min_influence_radius, float max_influence_radius);
    float computeCollisionCost(float node_distance, float min_influence_radius, float max_influence_radius);
    
    // User-defined variables
    std::shared_ptr<octomap::OcTree> octree_;
    std::shared_ptr<octomap::ColorOcTree> color_octree_;

    // Parameters
    float inflation_radius_ = 1.5;
    float min_influence_radius_ = 1.0;
    float max_influence_radius_ = 2.0;

}; // class Costmap3D

}; // namespace arena_core
