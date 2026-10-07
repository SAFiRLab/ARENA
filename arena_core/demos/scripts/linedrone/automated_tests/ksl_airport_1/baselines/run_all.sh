#!/bin/bash
# Every test of the comparison with the benchmark planners, one after the other (about 2 h, mostly ARENA, RRT* 5 s and
# Spline NSGA-II). Don't run other tests at the same time: the planning times are compared.
DIR="$(dirname "$(readlink -f "$0")")"
for script in arena_default astar moar3d_sweep rrt_star_short rrt_star_long spline_nsga2 moar3d_risk_sweep arena_risk_sweep; do
    "$DIR/$script.sh" || exit 1
done
