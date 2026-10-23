#!/usr/bin/env python3
"""Planner map from a saved octomap: the occupied and free leaves inside the bounds (expanded to the resolution), plus
free voxels at the 8 corners of the bounds (the planner bounds are the extent of the leaves), and its testbench config.

Outputs: ressources/generated_map/<name>/<name>.bt and config/linedrone/testbench_configs/<name>.yaml

Usage: crop_octomap.py <octomap .bt> <name> --x X0 X1 --y Y0 Y1 --z Z0 Z1 --start X Y Z --goal X Y Z
"""

import argparse
import os

import numpy as np
import yaml

from octomap_bt import DEMOS, RESSOURCES, TREE_MAX_KEY, read_bt, write_bt


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('octomap')
    parser.add_argument('name')
    for axis in 'xyz':
        parser.add_argument('--' + axis, nargs=2, type=float, required=True, metavar=('MIN', 'MAX'))
    parser.add_argument('--start', nargs=3, type=float, required=True)
    parser.add_argument('--goal', nargs=3, type=float, required=True)
    args = parser.parse_args()

    leaves, resolution = read_bt(args.octomap)
    to_key = lambda value: np.floor(np.asarray(value) / resolution).astype(np.int64) + TREE_MAX_KEY  # noqa: E731
    low = to_key([args.x[0], args.y[0], args.z[0]])
    high = to_key([args.x[1], args.y[1], args.z[1]]) - 1
    occupied, free = set(), set()
    for min_key, size, is_occupied in leaves:
        lo = np.maximum(np.asarray(min_key), low)
        hi = np.minimum(np.asarray(min_key) + size - 1, high)
        if np.any(lo > hi):
            continue
        grid = np.stack(np.meshgrid(*[np.arange(lo[k], hi[k] + 1) for k in range(3)], indexing='ij'), axis=-1).reshape(-1, 3)
        (occupied if is_occupied else free).update(map(tuple, grid.tolist()))
    for corner in {(x, y, z) for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])}:
        if corner not in occupied:
            free.add(tuple(int(v) for v in corner))

    folder = os.path.join(RESSOURCES, 'generated_map', args.name)
    os.makedirs(folder, exist_ok=True)
    bt_file = os.path.join(folder, args.name + '.bt')
    write_bt(bt_file, occupied, free, resolution)
    mission = {'drone': {'position': dict(zip('xyz', map(float, args.start))),
                         'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
               'planning_goal': {'position': dict(zip('xyz', map(float, args.goal)))}}
    with open(os.path.join(DEMOS, 'config', 'linedrone', 'testbench_configs', args.name + '.yaml'), 'w') as f:
        f.write('# Testbench configuration of ressources/generated_map/{0}/{0}.bt '
                '(scripts/linedrone/map_generator/rosbag_octomap/crop_octomap.py)\n'.format(args.name))
        yaml.safe_dump(mission, f, sort_keys=False)
    print('{}: {} occupied and {} free voxels in the bounds (input: {} leaves)'.format(
        bt_file, len(occupied), len(free), len(leaves)))


if __name__ == '__main__':
    main()
