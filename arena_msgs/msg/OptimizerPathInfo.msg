# Information about the path chosen at the end of an optimization
nav_msgs/Path path
float64[] velocities
float64 planning_time
# Duration of the RRT initialization and of the NSGA-II optimization in seconds (both included in planning_time)
float64 initialization_time
float64 optimization_time
# Number of NURBS control points of the decision vector
int32 nb_of_control_points
int32 nb_of_generations
int32 population_size
int32 nurbs_sample_size
float64 rrt_range
float64 best_time_cost
float64 best_security_cost
float64 best_energy_cost
float64 chosen_time_cost
float64 chosen_security_cost
float64 chosen_energy_cost
float64 time_coefficient
float64 security_coefficient
float64 energy_coefficient

# false when no solution respects the constraints and is safe, the path is then empty and the chosen costs are set to
# the max double. Only safe solutions can be chosen, the best costs are the best ones of the safe solutions.
bool feasible

# Objective values of the non-dominated feasible solutions of the final population (one entry per solution).
# Feasible: the constraints evaluated by the optimizer (acceleration, occupancy at the samples of the trajectory).
float64[] pareto_front_time_costs
float64[] pareto_front_security_costs
float64[] pareto_front_energy_costs

# Safety check of every solution of pareto_front_*: true when no segment of the trajectory crosses an occupied voxel
# of the inflated octomap (ray tracing between consecutive samples, same check as the one done before using a path)
bool[] pareto_front_safe

# Non-dominated set of the safe solutions of the final population. Contains the safe solutions of pareto_front_*, and
# the safe solutions only dominated by unsafe ones.
float64[] safe_front_time_costs
float64[] safe_front_security_costs
float64[] safe_front_energy_costs

# Number of solutions of the final population that respect the constraints, and among them the safe ones
int32 nb_of_feasible_solutions
int32 nb_of_safe_solutions
