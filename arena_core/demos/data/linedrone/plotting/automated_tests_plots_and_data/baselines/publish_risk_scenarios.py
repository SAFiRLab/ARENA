"""Publishes the trajectories of the risk scenarios and the raw octomap of a map, to take the screenshot of the trajectory
figure in RViz or Foxglove (MarkerArray, latched and republished every 2 s, frame "map").

Topics:
    /risk_scenarios/map     occupied voxels of the octomap (.bt, not inflated), grey cubes (one CUBE_LIST per voxel size),
                            optionally shaded by height (--height-shading) with dark edges (--voxel-edges)
    /risk_scenarios/paths   chosen trajectory of every planning of baselines/<map>/risk_scenarios/<scenario>/report_3d_*.csv,
                            one LINE_STRIP per planning, namespace = scenario (scenarios can be hidden one by one)

Colors (legend of the figure): high winds blue, low GPS green, low battery red, low battery + high winds magenta,
low battery + low GPS yellow, high winds + low GPS cyan.

Usage (ROS 2 sourced; Foxglove: ros2 launch foxglove_bridge foxglove_bridge_launch.xml):
    python3 publish_risk_scenarios.py --map CL_map_3 [--line-width 0.12] [--start-goal]
    python3 publish_risk_scenarios.py --map CL_map_3 --runs-dir <testbench outputs> --suffix _g1000_z10_int
        (test variants: the runs of a scenario in <runs-dir>/arena_scen_<scenario><suffix>/report_3d_*.csv)
    python3 publish_risk_scenarios.py --map scaffold_power_line_real --front scaffold_power_line_real/arena_front_infl0
        (whole front of one planning instead of the scenarios: every solution of <front>/paths.csv in its own namespace
        solution_<index>, colored from magenta (lowest energy cost) to green (lowest safety cost), costs in the log)
"""

import argparse
import glob
import os

import numpy as np
import pandas as pd
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

HERE = os.path.dirname(os.path.realpath(__file__))
RESSOURCES = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..', '..', 'ressources'))
OCTOMAPS = {'CL_map_3': os.path.join(RESSOURCES, 'saved_octomaps', 'map_test_3.bt'),
            'seed_42': os.path.join(RESSOURCES, 'generated_map', 'seed_42', 'cluttered_map_seed42.bt'),
            'scaffold_power_line_real': os.path.join(RESSOURCES, 'generated_map', 'scaffold_power_line_real',
                                                     'scaffold_power_line_real.bt')}
# (folder, legend label, RGB): order of the legend
SCENARIOS = [('high_winds', 'High winds', (0.0, 0.0, 1.0)),
             ('low_gps', 'Low GPS signal', (0.0, 1.0, 0.0)),
             ('low_battery', 'Low battery', (1.0, 0.0, 0.0)),
             ('battery_winds', 'Low battery + High winds', (1.0, 0.0, 1.0)),
             ('battery_gps', 'Low battery + Low GPS signal', (1.0, 1.0, 0.0)),
             ('winds_gps', 'High winds + Low GPS signal', (0.0, 1.0, 1.0))]
VOXEL_GREY = (0.35, 0.35, 0.35)
VOXEL_SHADING = (0.2, 0.75)  # Grey level of the lowest and highest voxels (--height-shading)
VOXEL_EDGE = (0.08, 0.08, 0.08)  # Color of the voxel edges (--voxel-edges)
CUBE_EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count('1') == 1]  # Corner indices (i & 1, i & 2, i & 4)
FRONT_ENERGY = np.array((1.0, 0.0, 1.0))  # Color of the front solution with the lowest energy cost (magenta)
FRONT_SAFETY = np.array((0.0, 1.0, 0.0))  # Color of the front solution with the lowest safety cost (green)
TREE_MAX_KEY = 32768  # Key of the center of the octree (16 levels)


def read_octomap(path):
    """Occupied leaves of an octomap binary file (.bt, OcTree::writeBinary): [(center xyz, edge length)].

    Binary format (OcTreeBase::readBinaryNode): every node is 2 bytes, 2 bits per child (bits 2i and 2i+1 of the bytes
    of children 0-3 and 4-7): 01 free leaf, 10 occupied leaf, 11 inner node, 00 no child; the inner children follow
    depth first in the child order. Child i is offset by (i & 1, i & 2, i & 4) along (x, y, z)."""
    with open(path, 'rb') as f:
        resolution = None
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('res'):
                resolution = float(line.split()[1])
            if line == 'data':
                break
        data = f.read()
    leaves, position = [], [0]

    def read_node(min_key, size):
        bits = data[position[0]] | (data[position[0] + 1] << 8)
        position[0] += 2
        half = size // 2
        inner = []
        for i in range(8):
            code = (bits >> (2 * i)) & 3  # bit 2i (low) and bit 2i+1 (high)
            child_min = (min_key[0] + (i & 1) * half, min_key[1] + ((i >> 1) & 1) * half, min_key[2] + ((i >> 2) & 1) * half)
            if code == 2:  # occupied leaf (bit 2i = 0, bit 2i+1 = 1)
                leaves.append(((np.array(child_min) + half / 2.0 - TREE_MAX_KEY) * resolution, half * resolution))
            elif code == 3:
                inner.append(child_min)
        for child_min in inner:
            read_node(child_min, half)

    read_node((0, 0, 0), 2 * TREE_MAX_KEY)
    return leaves, resolution


def read_paths(folder):
    """Chosen trajectory of every planning (Id) of the report of a scenario: [N x 3 array]."""
    reports = [f for f in glob.glob(os.path.join(folder, 'report_3d_*.csv')) if not f.endswith('_pareto_front.csv')]
    if not reports:
        return []
    report = pd.read_csv(reports[0])
    report = report[report.groupby('Id')['Feasible'].transform('max') == 1]
    return [rows[['Drone Position X', 'Drone Position Y', 'Drone Position Z']].to_numpy()
            for _, rows in report.groupby('Id', sort=True)]


def color(rgb, alpha=1.0):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(alpha))


class RiskScenariosPublisher(Node):

    def __init__(self, args):
        super().__init__('risk_scenarios_publisher')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(MarkerArray, '/risk_scenarios/map', latched)
        self.paths_pub = self.create_publisher(MarkerArray, '/risk_scenarios/paths', latched)

        leaves, resolution = read_octomap(OCTOMAPS[args.map])
        z_min, z_max = min(c[2] for c, _ in leaves), max(c[2] for c, _ in leaves)
        self.map_msg = MarkerArray()
        for index, size in enumerate(sorted({s for _, s in leaves})):
            marker = self.marker('octomap', index, Marker.CUBE_LIST, VOXEL_GREY)
            marker.scale.x = marker.scale.y = marker.scale.z = float(size)
            centers = [c for c, s in leaves if s == size]
            marker.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in centers]
            if args.height_shading:
                levels = [VOXEL_SHADING[0] + (VOXEL_SHADING[1] - VOXEL_SHADING[0]) * (c[2] - z_min) / max(z_max - z_min, 1e-9)
                          for c in centers]
                marker.colors = [color((level, level, level)) for level in levels]
            self.map_msg.markers.append(marker)
        if args.voxel_edges:
            # One LINE_LIST of the 12 edges of every voxel, the edges shared by neighbouring voxels drawn once
            edges = set()
            for center, size in leaves:
                corners = [tuple(np.round(center + (np.array((i & 1, (i >> 1) & 1, (i >> 2) & 1)) - 0.5) * size, 6))
                           for i in range(8)]
                edges.update(tuple(sorted((corners[a], corners[b]))) for a, b in CUBE_EDGES)
            marker = self.marker('octomap_edges', 0, Marker.LINE_LIST, VOXEL_EDGE)
            marker.scale.x = 0.06 * resolution
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for edge in edges for p in edge]
            self.map_msg.markers.append(marker)
        self.get_logger().info('{}: {} occupied leaves (resolution {} m)'.format(args.map, len(leaves), resolution))

        self.paths_msg = MarkerArray()
        starts, goals = [], []
        for folder, label, rgb in ([] if args.front else SCENARIOS):
            paths = read_paths(os.path.join(args.runs_dir, 'arena_scen_' + folder + args.suffix) if args.runs_dir else
                               os.path.join(HERE, args.map, 'risk_scenarios', folder))
            for index, path in enumerate(paths):
                marker = self.marker(folder, index, Marker.LINE_STRIP, rgb)
                marker.scale.x = args.line_width
                marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in path]
                self.paths_msg.markers.append(marker)
                starts.append(path[0])
                goals.append(path[-1])
            if paths:
                self.get_logger().info('{}: {} paths'.format(label, len(paths)))
        if args.front:
            for index, path, rgb in self.read_front(args.front):
                marker = self.marker('solution_{:02d}'.format(index), 0, Marker.LINE_STRIP, rgb)
                marker.scale.x = args.line_width
                marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in path]
                self.paths_msg.markers.append(marker)
                starts.append(path[0])
                goals.append(path[-1])
        if args.reference_path:
            reference = np.loadtxt(args.reference_path, delimiter=',', ndmin=2)
            marker = self.marker('reference_path', 0, Marker.LINE_STRIP, (1.0, 1.0, 1.0))
            marker.scale.x = 2.0 * args.line_width
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in reference]
            self.paths_msg.markers.append(marker)
            self.get_logger().info('Reference path (white): {} vertices from {}'.format(len(reference), args.reference_path))
        if args.start_goal:
            for index, (name, points) in enumerate([('start', starts), ('goal', goals)]):
                marker = self.marker('start_goal', index, Marker.SPHERE, (1.0, 1.0, 1.0))
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0
                p = np.mean(points, axis=0)
                marker.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                self.paths_msg.markers.append(marker)

        self.publish()
        self.create_timer(2.0, self.publish)

    def read_front(self, folder):
        """Solutions of <folder>/solutions.csv with their paths (<folder>/paths.csv): [(index, N x 3 array, RGB)], the color
        interpolated from magenta to green with the rank of the safety cost (energy and safety are opposed on the front)."""
        solutions = pd.read_csv(os.path.join(folder, 'solutions.csv'))
        paths = pd.read_csv(os.path.join(folder, 'paths.csv'))
        ranks = solutions['Security cost'].rank(method='first', ascending=False).to_numpy() - 1.0
        front = []
        for (_, solution), rank in zip(solutions.iterrows(), ranks):
            index = int(solution['Solution'])
            path = paths[paths['Solution'] == index].sort_values('Sample')[['X', 'Y', 'Z']].to_numpy()
            rgb = FRONT_ENERGY + (FRONT_SAFETY - FRONT_ENERGY) * rank / max(len(solutions) - 1, 1)
            front.append((index, path, rgb))
            self.get_logger().info('solution_{:02d}: time {:.1f} s, safety {:.3f}, energy {:.0f} kJ, mean clearance {:.2f} m'.format(
                index, solution['Time cost'], solution['Security cost'], solution['Energy cost'] / 1000.0,
                solution['Average Closest Obstacle Distance']))
        return front

    def marker(self, namespace, index, kind, rgb):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = namespace
        marker.id = index
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color = color(rgb)
        return marker

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for msg in (self.map_msg, self.paths_msg):
            for marker in msg.markers:
                marker.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        self.paths_pub.publish(self.paths_msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', default='CL_map_3', choices=sorted(OCTOMAPS))
    parser.add_argument('--line-width', type=float, default=0.12, help='Width of the trajectories (m)')
    parser.add_argument('--start-goal', action='store_true', help='White spheres at the start and the goal')
    parser.add_argument('--runs-dir', help='Folder of test runs (default: baselines/<map>/risk_scenarios), every scenario '
                                           'in <runs-dir>/arena_scen_<scenario><suffix>')
    parser.add_argument('--suffix', default='', help='Suffix of the scenario folders of --runs-dir (test variant)')
    parser.add_argument('--front', help='Folder with solutions.csv and paths.csv of one planning (exported front): every '
                                        'solution instead of the risk scenarios')
    parser.add_argument('--reference-path', help='CSV polyline (x,y,z per row) drawn in white, e.g. a safe shortest path')
    parser.add_argument('--height-shading', action='store_true',
                        help='Voxels shaded from dark grey (lowest) to light grey (highest) instead of uniform grey')
    parser.add_argument('--voxel-edges', action='store_true',
                        help='Dark edges on the voxels (namespace octomap_edges) to see the grid and the depth')
    args = parser.parse_args()
    rclpy.init()
    node = RiskScenariosPublisher(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
