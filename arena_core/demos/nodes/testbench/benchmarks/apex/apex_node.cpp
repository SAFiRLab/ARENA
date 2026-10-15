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

/**
 * A*pex benchmark planner: the approximate multi-objective A* of Zhang, Salzman, Kumar, Felner, Hernandez Ulloa and Koenig
 * ("A*pex: Efficient Approximate Multi-Objective Search on Graphs", ICAPS 2022), on the graph of the MOAR-3D benchmark.
 *
 * The graph and its costs are the ones of MOAR-3D (MoarGraphSearch, same parameters moar_3d.*): states (cell, arrival
 * direction) of a 3D grid, 26 motion primitives, cells valid if clear of the inflated octomap. The three terms of the MOAR
 * cost function are kept as separate objectives instead of being weighted (Eq. 7 without the coefficients):
 *     time TC = d / v(d_obs, theta), safety SC * d, energy EC = ||v - v_prev|| / (2 v_max)
 * The destination is a virtual vertex reached from every state of the goal cell with the energy of the stop (static
 * action). As in the paper, the heuristics of the time and safety costs are backward Dijkstra searches from the goal (on the
 * cells, these costs don't depend on the previous direction), the heuristic of the energy is the velocity change to stop at
 * the goal (Eq. 8 of MOAR), all consistent.
 *
 * A*pex returns an epsilon-approximate Pareto-optimal solution set (apex.epsilon, the same for the three objectives): every
 * Pareto-optimal path of the graph is epsilon-dominated by one of the returned paths. With apex.algorithm: toa_star, the
 * exact Tri-Objective A* (Ren et al., EMOA*) is used instead, intractable on the maps of the tests (the Pareto front of the
 * continuous costs is too large). Every path is flown as planned with the speeds of its states, like MOAR-3D (cell centers,
 * instantaneous changes of speed, evaluated by BenchmarkPlannerNode::scoreTrajectory without acceleration check), and
 * reported as a solution of the front. Not in the papers: the search stops after apex.max_time (the solutions found so far are kept, the front is
 * then incomplete).
 */

// Local
#include "common/benchmark_planner_node.h"
#include "testbench/benchmarks/moar_3d/moar_graph_search.h"
#include "testbench/benchmarks/moar_3d/moar_graph_setup.h"
#include "testbench/benchmarks/apex/apex_search.h"
#include "testbench/benchmarks/toa_star/toa_star_search.h"

// System
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>


namespace arena_benchmarks
{

class ApexNode : public BenchmarkPlannerNode
{
public:

    explicit ApexNode(const rclcpp::NodeOptions& a_options)
    : BenchmarkPlannerNode("linedrone_test_node", a_options)
    {}

protected:

    void plan(const Eigen::Vector3d& a_start, const Eigen::Vector3d& a_goal, PlanningResult& a_result) override
    {
        const double clearance = param<double>("moar_3d.clearance", 0.8);
        const double sdf_d_min = param<double>("moar_3d.sdf_min_distance", 2.0);
        const double sdf_d_max = param<double>("moar_3d.sdf_max_distance", 9.0);
        a_result.rrt_range = param<double>("moar_3d.grid_resolution", 1.0);

        const std::string algorithm = param<std::string>("apex.algorithm", "apex");
        const double epsilon = param<double>("apex.epsilon", 0.01);
        const double max_time = param<double>("apex.max_time", 600.0);
        const size_t max_nodes = static_cast<size_t>(param<int64_t>("apex.max_nodes", 60000000));

        const MoarSpeeds speeds = moarSpeeds(linedrone_config_.robot_max_speed_,
                                             [this](const std::string& name, double value) { return param<double>(name, value); });

        auto sdf = costmap_mapping_->getCapability<arena_demos::CostSDFCapability>();
        if (!sdf)
            throw std::runtime_error("The SDF octomap is not available");

        // Same graph as MOAR-3D on the map bounds, rebuilt if the grid changed
        MoarGraphConfig config;
        config.resolution = param<double>("moar_3d.grid_resolution", 1.0);
        Eigen::MatrixXd bounds = costmap_mapping_->getMapBounds();
        config.min_bounds = bounds.row(0).transpose();
        config.max_bounds = bounds.row(1).transpose();
        if (!graph_ || config.resolution != graph_->getConfig().resolution ||
            !config.min_bounds.isApprox(graph_->getConfig().min_bounds) || !config.max_bounds.isApprox(graph_->getConfig().max_bounds))
            graph_ = std::make_unique<MoarGraphSearch>(config, moarCellFn(costmap_mapping_, sdf, clearance, sdf_d_min, sdf_d_max));
        else
            graph_->clearCache(); // The map is received again before every planning

        const auto start_time = std::chrono::steady_clock::now();
        int64_t start_state;
        Eigen::Vector3i goal_cell;
        if (!graph_->startAndGoal(a_start, a_goal, start_state, goal_cell))
        {
            RCLCPP_ERROR(get_logger(), "A*pex: no valid cell close to the start or the goal");
            return;
        }

        // Heuristics: backward Dijkstra of the time and safety costs
        std::vector<double> h_time, h_safety;
        graph_->costToGo(goal_cell, speeds, h_time, h_safety);
        const double heuristic_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

        constexpr int64_t kDestination = -2;
        std::vector<MoarGraphSearch::Edge> edges;
        auto successors = [&](int64_t a_state, std::vector<std::pair<int64_t, ToaCost>>& a_successors)
        {
            a_successors.clear();
            graph_->successors(a_state, speeds, edges);
            for (const auto& edge : edges)
                a_successors.push_back({edge.state, edge.cost});
            // Stop at the goal (static action): velocity change to 0
            if (graph_->stateCell(a_state) == goal_cell)
                a_successors.push_back({kDestination, {0.0, 0.0, graph_->stateSpeed(a_state, speeds) / (2.0 * speeds.v_max)}});
        };
        auto heuristic = [&](int64_t a_state) -> ToaCost
        {
            if (a_state == kDestination)
                return {0.0, 0.0, 0.0};
            // Energy: velocity change to stop at the goal (MOAR, Eq. 8), consistent by the triangle inequality
            const int64_t index = graph_->cellIndexOf(graph_->stateCell(a_state));
            return {h_time[index], h_safety[index], graph_->stateSpeed(a_state, speeds) / (2.0 * speeds.v_max)};
        };

        std::vector<ToaStarSearch::Solution> solutions;
        size_t nb_of_expansions = 0, nb_of_nodes = 0;
        bool complete = true;
        if (algorithm == "toa_star")
        {
            ToaStarConfig config_toa;
            config_toa.max_time = max_time;
            config_toa.max_labels = max_nodes;
            ToaStarStats stats;
            solutions = ToaStarSearch(config_toa).search(start_state, kDestination, successors, heuristic, &stats);
            nb_of_expansions = stats.nb_of_expansions;
            nb_of_nodes = stats.nb_of_labels;
            complete = stats.complete;
        }
        else
        {
            ApexSearchConfig config_apex;
            config_apex.epsilon = {epsilon, epsilon, epsilon};
            config_apex.max_time = max_time;
            config_apex.max_nodes = max_nodes;
            ApexSearchStats stats;
            solutions = ApexSearch(config_apex).search(start_state, kDestination, successors, heuristic, &stats);
            nb_of_expansions = stats.nb_of_expansions;
            nb_of_nodes = stats.nb_of_apex_path_pairs;
            complete = stats.complete;
        }
        a_result.initialization_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        RCLCPP_INFO(get_logger(), "%s (epsilon %.3f): %zu paths in %.3f s (heuristics %.3f s), %zu expansions, %zu nodes%s",
                    algorithm == "toa_star" ? "TOA*" : "A*pex", algorithm == "toa_star" ? 0.0 : epsilon, solutions.size(),
                    a_result.initialization_time, heuristic_time, nb_of_expansions, nb_of_nodes,
                    complete ? "" : ", STOPPED BEFORE THE END (incomplete front)");

        // Every Pareto-optimal path flown with the speeds of its states, like MOAR-3D
        for (const ToaStarSearch::Solution& solution : solutions)
        {
            std::vector<int64_t> states(solution.vertices.begin(), solution.vertices.end() - 1); // Without the destination
            std::vector<MoarPathNode> path;
            graph_->pathOfStates(states, a_start, a_goal, speeds, path);
            a_result.solutions.push_back(scoreTrajectory(moarTrajectory(path), false));
            a_result.nb_of_control_points = static_cast<int>(path.size());
        }
    }

private:
    std::unique_ptr<MoarGraphSearch> graph_;

}; // class ApexNode

}; // namespace arena_benchmarks


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<arena_benchmarks::ApexNode>(options));
    rclcpp::shutdown();
    return 0;
}
