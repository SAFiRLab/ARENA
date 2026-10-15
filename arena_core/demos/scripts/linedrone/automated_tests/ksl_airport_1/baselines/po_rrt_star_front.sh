#!/bin/bash
# One planning of PO-RRT* with every safe solution of its front exported (optimization.export_solutions_file) to
# /home/dev_ws/data_output/navigation_3d/<map>_po_rrt_star_front_solutions.csv, copy it to baselines/<map>/po_rrt_star_front/solutions.csv
# for the risk sensitivity figure (plot_risk_sensitivity.py --fronts, the risks are swept offline on this front).
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

params=$(mktemp --suffix=.yaml)
cat "$BENCHMARK_CONFIGS_DIR/po_rrt_star_params.yaml" > "$params"
cat >> "$params" <<EOS
        optimization:
            export_solutions_file: "/home/dev_ws/data_output/navigation_3d/${map_name}_po_rrt_star_front_solutions.csv"
EOS

run_planning_tests po_rrt_star "$params" po_rrt_star_front 1 \
    fixed_rrt_range:=5.0 \
    fixed_cost_time:=1.0 fixed_cost_safety:=0.0 fixed_cost_energy:=0.0
