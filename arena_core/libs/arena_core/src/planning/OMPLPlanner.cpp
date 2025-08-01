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

#include "arena_core/planning/OMPLPlanner.h"

// Eternal
// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>


namespace arena_core
{

OMPLPlanner::OMPLPlanner()
: ompl::geometric::SimpleSetup(std::make_shared<ompl::base::RealVectorStateSpace>()),
dim_(0), goal_(), start_(), octree_(nullptr), bounds_(ompl::base::RealVectorBounds(0)), 
initializer_(std::make_shared<IPlanningInitializer>()), solving_timeout_(1.0)
{}

OMPLPlanner::~OMPLPlanner()
{}

void OMPLPlanner::initializePlanner()
{
    assert(octree_ != nullptr && "OMPLPlanner::initializePlanner() -> No Octree set for the planner.");
    assert(initializer_ != nullptr && "OMPLPlanner::initializePlanner() -> No initializer set for the planner.");
    assert(initializer_->isInitialized() && "OMPLPlanner::initializePlanner() -> Initializer is not fully set up.");
    assert(dim_ > 0 && "OMPLPlanner::initializePlanner() -> Problem dimensions must be set before initializing the planner.");
    assert(bounds_.low.size() == dim_ && bounds_.high.size() == dim_ && "OMPLPlanner::initializePlanner() -> Bounds size must match problem dimensions.");

    int fallback_idx = 0;
    while (getStateSpace()->getDimension() < dim_ && fallback_idx < dim_)
    {
        getStateSpace()->as<ompl::base::RealVectorStateSpace>()->addDimension();
        fallback_idx++;
    }

    try
    {
        bounds_.check();
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(std::string("OMPLPlanner::initializePlanner() -> Bounds check failed: ") + e.what());
    }

    getStateSpace()->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds_);

    // Set the state validity checker
    setStateValidityChecker(initializer_->state_validity_checker_);

    // Create the optimization objective
    if (initializer_->optimization_objective_)
    {
        auto objective = initializer_->optimization_objective_;
        if (!initializer_->cost_to_go_heuristic_)
            throw std::runtime_error("OMPLPlanner::initializePlanner() -> Cost-to-go heuristic must be set if optimization objective is provided.");
        
        objective->setCostToGoHeuristic(initializer_->cost_to_go_heuristic_);
        setOptimizationObjective(objective);
    }
    
    // Create a planner for the defined state space
    setPlanner(initializer_->planner_);
}

ompl::base::PlannerStatus OMPLPlanner::plan()
{
    assert(getProblemDefinition() != nullptr && "OMPLPlanner::plan() -> Problem definition must be set before planning.");
    assert(goal_.size() == dim_ && "OMPLPlanner::plan() -> Goal size does not match problem dimensions.");
    assert(start_.size() == dim_ && "OMPLPlanner::plan() -> Start size does not match problem dimensions.");
    assert(solving_timeout_ > 0.0 && "OMPLPlanner::plan() -> Solving timeout must be positive.");

    initializePlanner();

    // Set the start and goal states
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start_state(getStateSpace());
    for (int i = 0; i < dim_; ++i)
        start_state[i] = start_[i];
    
    if (!getStateValidityChecker()->isValid(start_state.get()))
    {
        std::cerr << "OMPLPlanner::plan() -> Start state is not valid." << std::endl;
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::INVALID_START);
    }

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal_state(getStateSpace());
    for (int i = 0; i < dim_; ++i)
        goal_state[i] = goal_[i];

    if (!getStateValidityChecker()->isValid(goal_state.get()))
    {
        std::cerr << "OMPLPlanner::plan() -> Goal state is not valid." << std::endl;
        return ompl::base::PlannerStatus(ompl::base::PlannerStatus::INVALID_GOAL);
    }

    setStartAndGoalStates(start_state, goal_state);

    clear();
    // Perform setup steps
    setup();

    // DEBUG
    auto si = getSpaceInformation();
    auto sampler = si->allocValidStateSampler();
    int valid_count = 0;
    for (int i = 0; i < 1000; ++i) {
        ompl::base::State *state = si->allocState();
        sampler->sample(state);
        if (si->isValid(state)) valid_count++;
        si->freeState(state);
    }
    std::cout << "Valid samples out of 1000: " << valid_count << std::endl;


    // Solve the planning problem
    ompl::base::PlannerStatus status = getPlanner()->solve(solving_timeout_);

    return status;
}

}; // namespace arena_core
