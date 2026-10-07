#!/bin/bash
# Planning problem of every test of this folder: the generated map with seed 42.
# Source this file, don't execute it. To test another generated map, copy this folder and change the values below.
#
#   ressources/generated_map/seed_42/cluttered_map_seed42.bt       map loaded by costmap_3D_node
#   config/linedrone/testbench_configs/cluttered_map_seed42.yaml   start (drone position) and goal of the planning

SETTINGS_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
DEMOS_DIR="${SETTINGS_DIR%/scripts/*}"

# Selects the testbench config, which gives the start and the goal
map_name="cluttered_map_seed42"

# Octomap published by costmap_3D_node
bt_file="$DEMOS_DIR/ressources/generated_map/seed_42/${map_name}.bt"

# Only used to name the reports: report_3d_<world_name>_<date>.csv
world_name="$map_name"

for file in "$bt_file" "$DEMOS_DIR/config/linedrone/testbench_configs/${map_name}.yaml"; do
    if [ ! -f "$file" ]; then
        echo "Missing $file, generate the map with scripts/linedrone/map_generator/generate_map.sh -s 42"
        exit 1
    fi
done
