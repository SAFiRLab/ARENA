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

#include "linedrone/costmap_3D.h"


namespace arena_demos
{

Costmap3D::Costmap3D()
: octree_(nullptr),
color_octree_(nullptr)
{
    // Initialize the controller
}

Costmap3D::~Costmap3D()
{}

void Costmap3D::setOctree(std::shared_ptr<octomap::OcTree> tree)
{
    if (tree)
    {
        octree_ = tree; 

        // Serialize the input tree to a binary stream
        std::stringstream serializedTree;
        tree->writeBinary(serializedTree);

        // Deserialize the binary stream into a ColorOcTree
        color_octree_ = std::make_shared<octomap::ColorOcTree>(tree->getResolution());
        color_octree_->readBinary(serializedTree);

        if (color_octree_ == nullptr) 
            std::cerr << "Error: Color_octree ptr is null." << std::endl;

        inflateOctree(inflation_radius_, min_influence_radius_, max_influence_radius_); 
    }
}

void Costmap3D::inflateOctree(float inflation_radius, float min_influence_radius, float max_influence_radius)
{
    if (!octree_)
        throw std::runtime_error("Costmap3D::inflateOctree => Octree is not set. Cannot inflate the octree.");

    if (!color_octree_)
        throw std::runtime_error("Costmap3D::inflateOctree => Color octree is not set. Cannot inflate the octree.");
    
    // expand the nodes of the octree that were pruned
    color_octree_->expand();

    double tree_depth = color_octree_->getTreeDepth();
    // double leaf_node_size = color_octree_->getNodeSize(tree_depth);

    // Initialize the sets of free and occupied cells
    octomap::KeySet free_cells;
    octomap::KeySet occupied_cells;
    for (octomap::ColorOcTree::leaf_iterator it = color_octree_->begin_leafs(tree_depth), end = color_octree_->end_leafs(); it != end; ++it)
    {
        if (color_octree_->isNodeOccupied(*it))
        {
            occupied_cells.insert(it.getKey());
        }
        else
        {
            free_cells.insert(it.getKey());
        }
    }

    // Initialize the distance map 
    std::map<octomap::OcTreeKey, double, OcTreeKeyCompare> distance_map; 
    std::map<octomap::OcTreeKey, octomap::OcTreeKey, OcTreeKeyCompare> parent_map;
    for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
    {
        distance_map[*it] = std::numeric_limits<double>::max(); 
        parent_map[*it] = octomap::OcTreeKey(-1, -1, -1);
    }

    // Initialize the priority queue for Dijkstra's algorithm
    std::vector<OctreeKeyPairCompare::value_type> vector;
    std::priority_queue<OctreeKeyPairCompare::value_type, std::vector<OctreeKeyPairCompare::value_type>, OctreeKeyPairCompare> pq;
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
    {
        pq.push(std::make_pair(0.0, *it)); 
        distance_map[*it] = 0.0;
        parent_map[*it] = *it; 
    }

    // Perform Dijkstra's algorithm 
    while (!pq.empty())
    {
        double dist = pq.top().first;
        octomap::OcTreeKey key = pq.top().second;
        pq.pop();

        octomap::point3d parent_point = color_octree_->keyToCoord(parent_map[key]); 

        // Get the node's neighbors 
        std::vector<octomap::OcTreeKey> neighbors;
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1], key[2]));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1], key[2]));
        neighbors.push_back(octomap::OcTreeKey(key[0], key[1] - 1, key[2]));
        neighbors.push_back(octomap::OcTreeKey(key[0], key[1] + 1, key[2]));
        neighbors.push_back(octomap::OcTreeKey(key[0], key[1], key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0], key[1], key[2] + 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1] - 1, key[2]));
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1] + 1, key[2]));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1] - 1, key[2]));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1] + 1, key[2]));
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1], key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1], key[2] + 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1], key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1], key[2] + 1));
        neighbors.push_back(octomap::OcTreeKey(key[0], key[1] - 1, key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0], key[1] - 1, key[2] + 1));
        neighbors.push_back(octomap::OcTreeKey(key[0], key[1] + 1, key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0], key[1] + 1, key[2] + 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1] - 1, key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1] - 1, key[2] + 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1] + 1, key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] - 1, key[1] + 1, key[2] + 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1] - 1, key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1] - 1, key[2] + 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1] + 1, key[2] - 1));
        neighbors.push_back(octomap::OcTreeKey(key[0] + 1, key[1] + 1, key[2] + 1));

        // Iterate through the neighbors
        for (octomap::OcTreeKey& neighbor : neighbors)
        {
            octomap::point3d neighbor_point = color_octree_->keyToCoord(neighbor);

            double new_dist = sqrt(pow(parent_point.x() - neighbor_point.x(), 2) + pow(parent_point.y() - neighbor_point.y(), 2) + pow(parent_point.z() - neighbor_point.z(), 2));

            if (color_octree_->search(neighbor, tree_depth) == nullptr)
            {
                //if (new_dist < max_influence_radius && (neighbor_point.x() < 3 && neighbor_point.y() < 3) ) // uncomment for cross-section view 
                if (new_dist < max_influence_radius) 
                {
                    // insert a new leaf node to the ColorOcTreeNode at coord neighbor_point, and set it as free
                    octomap::ColorOcTreeNode* colorNode = color_octree_->updateNode(neighbor, false);
                    colorNode->setValue(-10.0);
                    
                    distance_map[neighbor] = new_dist;
                    parent_map[neighbor] = parent_map[key];
                    pq.push(std::make_pair(new_dist, neighbor));
                }
                continue;
            }
            
            if (new_dist < distance_map[neighbor])
            {
                distance_map[neighbor] = new_dist;
                parent_map[neighbor] = parent_map[key]; 
                pq.push(std::make_pair(new_dist, neighbor));
            }
        }
    }

    // Set the distance map values and store it as a color
    for (octomap::ColorOcTree::leaf_iterator it = color_octree_->begin_leafs(), end = color_octree_->end_leafs(); it != end; ++it)
    {
        octomap::OcTreeKey key = it.getKey();

        double node_distance = distance_map[key];

        double safety_cost = computeCollisionCost(node_distance, min_influence_radius, max_influence_radius);
            
        int white = 255 * (1 - safety_cost); // safety_cost = 1 - white / 255
        
        color_octree_->setNodeColor(key, 255 - white, 0, white);

        if(white == 0)
        {
            color_octree_->setNodeColor(key, 0, 0, 0); // set obstacles in black
        }
        else if (white == 255)
        {
            color_octree_->setNodeColor(key, 0, 255, 255); // set free cells in cyan
        }

        if(node_distance < inflation_radius)
        {
            octomap::ColorOcTreeNode* colorNode = color_octree_->updateNode(key, true);
            octomap::OcTreeNode* octNode = octree_->updateNode(key, true);
            colorNode->setValue(3.5);
            octNode->setValue(3.5);
        }
    }

    // Update the octree to ensure consistency
    color_octree_->updateInnerOccupancy();
}

float Costmap3D::computeCollisionCost(float node_distance, float min_influence_radius, float max_influence_radius)
{
    if (node_distance > max_influence_radius)
    {
        return 0.0; 
    }
    if (node_distance < min_influence_radius)
        return 1.0;

    // Inversely proportional scaling
    // It penalizes way more the nodes that are closer to the obstacles than the ones that are further away
    float l_scaling = min_influence_radius / (max_influence_radius - min_influence_radius);
    return l_scaling * (max_influence_radius / node_distance - 1);

    // Linear scaling
    //return (max_influence_radius - node_distance) / (max_influence_radius - min_influence_radius);
}

}; // namespace arena_demos
