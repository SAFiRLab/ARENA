/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, David-Alexandre Poissant, Université de Sherbrooke
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
#include "testbench/benchmarks/common/native_trajectory.h"

// System
#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_benchmarks
{

/**
 * @brief Parameters of the kinodynamic path searching of Fast-Planner (names of its search/* ROS parameters, values of
 * kino_algorithm.xml as defaults).
 */
struct KinodynamicAstarConfig
{
    double max_tau = 0.6;           // s, duration of the motion primitives
    double init_max_tau = 0.8;      // s, duration of the primitives of the first search loop
    double max_vel = 3.0;           // m/s, limit of every velocity component
    double max_acc = 2.0;           // m/s^2, limit of every acceleration component (inputs)
    double w_time = 10.0;           // rho, weight of the time in the cost J = int ||u||^2 dt + rho T
    double horizon = 7.0;           // m, the search stops at this distance from the start (local planning)
    double resolution_astar = 0.1;  // m, voxels of the search (one node per voxel)
    double lambda_heu = 5.0;        // Weight of the heuristic
    int allocate_num = 100000;      // Maximum number of nodes
    int check_num = 5;              // Collision checks along a primitive
    double vel_margin = 0.0;        // Added to max_vel
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();   // Region of the map (sdf_map getRegion: origin and size)
    Eigen::Vector3d map_size = Eigen::Vector3d::Ones();
}; // struct KinodynamicAstarConfig


/**
 * @brief Kinodynamic path searching of Zhou, Gao, Wang, Liu and Shen ("Robust and Efficient Quadrotor Trajectory
 * Generation for Fast Autonomous Flight", RA-L 2019), the front-end of Fast-Planner: hybrid-state A* on a voxel grid with
 * double integrator motion primitives (Algorithm 1), cost J = int ||u||^2 dt + rho T, heuristic and analytic expansion of
 * the minimum-cost trajectory to the goal state (Pontryagin's minimum principle).
 *
 * Port of KinodynamicAstar (fast_planner/path_searching/src/kinodynamic_astar.cpp) without ROS: same search loop, inputs
 * {-max_acc, -max_acc / 2, 0, max_acc / 2, max_acc}^3, primitive duration max_tau, per-component velocity check, check_num
 * collision checks per primitive, pruning of the primitives ending in the same voxel, termination near the goal (within
 * ceil(1 / resolution) voxels) with the analytic expansion ("shot" trajectory, checked for collisions and map bounds
 * only, as in the code). The inflated occupancy of the SDF map is replaced by a_occupied (true: occupied), the nodes are
 * allocated when needed instead of a preallocated pool (same limit allocate_num). The static (non-dynamic) search mode
 * is used, as in the replanning of Fast-Planner.
 */
class KinodynamicAstar
{
public:

    enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };
    using OccupancyFn = std::function<bool(const Eigen::Vector3d&)>;

    KinodynamicAstar(const KinodynamicAstarConfig& a_config, OccupancyFn a_occupied);
    ~KinodynamicAstar();

    void reset();
    int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
               Eigen::Vector3d end_vel, bool init);

    /**
     * @brief Trajectory of the last search (primitives, then the shot trajectory if the analytic expansion succeeded),
     * sampled every a_delta_t (and at the end of every primitive), with the velocities.
     */
    NativeTrajectory getTrajectory(double a_delta_t) const;

    int getNbOfNodes() const { return use_node_num_; }
    int getNbOfIterations() const { return iter_num_; }

private:

    struct PathNode
    {
        Eigen::Vector3i index;
        Eigen::Matrix<double, 6, 1> state;
        double g_score, f_score;
        Eigen::Vector3d input;
        double duration;
        PathNode* parent = nullptr;
        char node_state;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    using PathNodePtr = PathNode*;

    struct NodeComparator
    {
        bool operator()(PathNodePtr node1, PathNodePtr node2) { return node1->f_score > node2->f_score; }
    };

    struct IndexHash
    {
        std::size_t operator()(const Eigen::Vector3i& matrix) const
        {
            size_t seed = 0;
            for (int i = 0; i < 3; ++i)
                seed ^= std::hash<int>()(matrix[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    PathNodePtr newNode();
    Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
    void retrievePath(PathNodePtr end_node);
    std::vector<double> cubic(double a, double b, double c, double d);
    std::vector<double> quartic(double a, double b, double c, double d, double e);
    bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
    double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
    void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um, double tau);

    KinodynamicAstarConfig config_;
    OccupancyFn occupied_;

    std::vector<std::unique_ptr<PathNode>> path_node_pool_;
    int use_node_num_ = 0, iter_num_ = 0;
    std::unordered_map<Eigen::Vector3i, PathNodePtr, IndexHash> expanded_nodes_;
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
    std::vector<PathNodePtr> path_nodes_;

    Eigen::Vector3d start_vel_, end_vel_, start_acc_;
    Eigen::Matrix<double, 6, 6> phi_;
    bool is_shot_succ_ = false;
    Eigen::MatrixXd coef_shot_;
    double t_shot_ = 0.0;

    double max_tau_, init_max_tau_, max_vel_, max_acc_, w_time_, horizon_, lambda_heu_;
    int allocate_num_, check_num_;
    double tie_breaker_;
    double resolution_, inv_resolution_;
    Eigen::Vector3d origin_, map_size_3d_;

}; // class KinodynamicAstar

}; // namespace arena_benchmarks
