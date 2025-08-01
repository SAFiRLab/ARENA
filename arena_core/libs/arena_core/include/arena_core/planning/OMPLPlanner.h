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
#include <cassert>

// External
// Eigen
#include <Eigen/Dense>
// OMPL
#include <ompl/geometric/SimpleSetup.h>
// Octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>


namespace arena_core
{

/**
    * @brief IPlanningInitializer is an interface for initializing planning parameters.
    * It can be extended to provide specific initialization methods for different planning algorithms.
    *
    * @note This interface is designed to be used with the OMPL library and requires to use the OMPL library's planners.
*/
struct IPlanningInitializer
{
    ompl::base::StateValidityCheckerPtr state_validity_checker_; ///< The state validity checker for the planning problem
    ompl::base::OptimizationObjectivePtr optimization_objective_; ///< The optimization objective for the planning problem
    ompl::base::CostToGoHeuristic cost_to_go_heuristic_; ///< The cost-to-go heuristic for the planning problem
    ompl::base::PlannerPtr planner_; ///< The planner for the planning problem

    bool isInitialized() const
    {
        // optimization objective and cost-to-go heuristic can be null, but planner and state validity checker must be set
        return state_validity_checker_ != nullptr && planner_ != nullptr;
    };

}; // IPlanningInitializer


/**
    * @brief OMPLPlanner is a class that provides an interface for planning using the OMPL library.
    * It allows setting and getting planning parameters, such as start and goal states, octree for environment representation,
    * and bounds for the planning problem. It also provides a method to perform the planning operation.
    *
    * @note This class is designed to be used with the OMPL library and requires proper initialization of the environment.
*/
class OMPLPlanner : public ompl::geometric::SimpleSetup
{
public:
    OMPLPlanner();
    ~OMPLPlanner();

    /************* Getters *************/
    /**
     * @brief Get the planning goal.
     * @return A Eigen::VectorXd of the planning goal.
     */
    Eigen::VectorXd getGoal() const { return goal_; };

    /**
     * @brief Get the planning start.
     * @return A Eigen::VectorXd of the planning start.
     */
    Eigen::VectorXd getStart() const { return start_; };

    /**
     * @brief Get the Octree used for planning.
     * @return A shared pointer to the Octree.
     */
    std::shared_ptr<octomap::OcTree> getOctree() const { return octree_; };

    /**
     * @brief Get the planning problem bounds
     * @return A pair of Eigen::VectorXd representing the lower and upper bounds.
     */
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getBounds() const
    {
        Eigen::VectorXd lower(dim_), upper(dim_);
        for (int i = 0; i < dim_; ++i)
        {
            lower[i] = bounds_.low[i];
            upper[i] = bounds_.high[i];
        }
        return std::make_pair(lower, upper);
    };

    /**
     * @brief Get the problem dimensions.
     * @return An integer representing the number of dimensions.
     */
    int getProblemDimensions() const { return dim_; };

    /**
     * @brief Get the initializer for planning parameters.
     * @return A shared pointer to the IPlanningInitializer.
     */
    std::shared_ptr<IPlanningInitializer> getInitializer() const { return initializer_; };

    /**
     * @brief Get the solving timeout for the planning problem.
     * @return A double representing the timeout in seconds.
     */
    double getSolvingTimeout() const { return solving_timeout_; };

    /************* Setters *************/
    /**
    * @brief Set the planning goal.
    * @param a_goal A Eigen::VectorXd representing the goal.
    */
    void setGoal(const Eigen::VectorXd& a_goal)
    {
        assert(a_goal.size() == dim_ && "OMPLPlanner::setGoal() -> Goal size does not match problem dimensions.");
        goal_ = a_goal;
    };

    /**
     * @brief Set the planning start.
     * @param a_start A Eigen::VectorXd representing the start.
     */
    void setStart(const Eigen::VectorXd& a_start)
    {
        assert(a_start.size() == dim_ && "OMPLPlanner::setStart() -> Start size does not match problem dimensions.");
        start_ = a_start;
    };

    /**
     * @brief Set the Octree used for planning.
     * @param a_octree A shared pointer to the Octree.
     */
    void setOctree(const std::shared_ptr<octomap::OcTree>& a_octree)
    {
        assert(a_octree != nullptr && "OMPLPlanner::setOctree() -> Octree cannot be null.");
        octree_ = a_octree;
    };

    /**
     * @brief Set the problem dimensions.
     * @param a_dim An integer representing the number of dimensions.
     */
    void setProblemDimensions(int a_dim)
    {
        assert(a_dim > 0 && "OMPLPlanner::setProblemDimensions() -> Dimensions must be positive.");
        dim_ = a_dim;
    }

    /**
     * @brief Set the solving timeout for the planning problem.
     * @param a_timeout A double representing the timeout in seconds.
     */
    void setSolvingTimeout(double a_timeout)
    {
        assert(a_timeout > 0 && "OMPLPlanner::setSolvingTimeout() -> Timeout must be positive.");
        solving_timeout_ = a_timeout;
    };

     /**
     * @brief Set the bounds for the planning problem.
     * @param a_lower A Eigen::VectorXd representing the lower bounds.
     * @param a_upper A Eigen::VectorXd representing the upper bounds.
     */
    void setBounds(const Eigen::VectorXd& a_lower, const Eigen::VectorXd& a_upper)
    {
        assert(a_lower.size() == dim_ && a_upper.size() == dim_ && "OMPLPlanner::setBounds() -> Bounds size does not match problem dimensions.");
        bounds_ = ompl::base::RealVectorBounds(dim_);
        for (int i = 0; i < dim_; ++i)
        {
            bounds_.setLow(i, a_lower[i]);
            bounds_.setHigh(i, a_upper[i]);
        }
    };

    /************* User-defined methods *************/
    /**
    * @brief Plan a path from start to goal.
    * @return A ompl::base::PlannerStatus indicating the success or not of the planning.
    */
    ompl::base::PlannerStatus plan();

    /**
     * @brief Check if the bounds are valid.
     * @return A boolean indicating whether the bounds are valid.
     */
    bool checkBounds() const
    {
        try
        {
            bounds_.check();
            return true;
        }
        catch (const std::exception& e)
        {
            std::cerr << "OMPLPlanner::checkBounds() -> Bounds check failed: " << e.what() << std::endl;
            return false;
        }
    };

private:

    /************* User-defined methods *************/
    /**
    * @brief Initialize the planner with the environment.
    * This method should be called before using the planner to ensure that all necessary parameters are set.
    * It sets up the OMPL SimpleSetup with the state space, optimization objective, and other parameters.
    */
   void initializePlanner();

    /************* User-defined attributes *************/
    Eigen::VectorXd goal_; ///< The planning goal.
    Eigen::VectorXd start_; ///< The planning start.
    std::shared_ptr<octomap::OcTree> octree_; ///< The Octree used for planning.
    ompl::base::RealVectorBounds bounds_; ///< The bounds for the planning problem.
    int dim_; ///< The number of dimensions in the planning problem.
    std::shared_ptr<IPlanningInitializer> initializer_; ///< The initializer for planning parameters.
    double solving_timeout_; ///< The timeout for solving the planning problem, in seconds.

}; // OMPLPlanner

}; // arena_core
