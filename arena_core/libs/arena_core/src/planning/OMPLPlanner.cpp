#include "arena_core/planning/OMPLPlanner.h"

// Local


// System

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
    auto objective = initializer_->optimization_objective_;
    objective->setCostToGoHeuristic(initializer_->cost_to_go_heuristic_);
    setOptimizationObjective(objective);

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

    // Perform setup steps
    setup();

    // Solve the planning problem
    ompl::base::PlannerStatus status = getPlanner()->solve(solving_timeout_);

    return status;
}

}; // namespace arena_core
