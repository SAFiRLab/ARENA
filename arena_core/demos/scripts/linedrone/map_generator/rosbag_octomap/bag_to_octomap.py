#!/usr/bin/env python3
"""Replays the lidar of a ROS 1 bag (read with the rosbags package, no ROS 1 needed) into octomap_server.

Every lidar message, in the order of its stamp, is republished on /cloud_in in the frame "lidar" with the transform
map -> lidar of the bag TF at its stamp (map -> offset_base_link -> base_link -> velodyne, interpolated), so that
octomap_server ray-casts from the sensor origin. The next message waits for the octomap of this one (/octomap_binary at
its stamp): nothing is dropped.

Sources (--sources):
    cloud   velodyne_points (PointCloud2, VLP-16): every point moved to the pose at the stamp with its own time field
            (deskewed sweep, the stamp is the end of the sweep)
    scan    scan (LaserScan of the velodyne, the central rings flattened into the velodyne x-y plane, 10 Hz): points at
            z = 0 in the velodyne frame, up to --scan-max-range (the flattening error grows with the range)

Ground alignment (--ground-align): the altitude of the bag pose can jump (e.g. 0.65 m at the takeoff of the
2023-09-05 bag: the ground seen before the takeoff is 0.65 m below the ground seen in flight, two ground layers and the
structures seen from the ground shifted down). The ground height under every scan (5th percentile of the z of its points
2 to 15 m away horizontally, median over --ground-window s) is brought to the median ground height of the flight (lidar
more than 1.5 m above the ground) by a vertical shift of the sensor pose, interpolated in time for the clouds.

Map frame (--transform yaw x y z): the sensor poses (after the ground alignment) are moved by the rotation of yaw
(deg) around z then the translation (x, y, z), e.g. to align the map with the structures and put the ground at z = 0.

Usage: bag_to_octomap.py <bag> [--sources cloud scan] [--cloud-topic ...] [--scan-topic ...] [--scan-max-range 20]
                         [--ground-align] [--ground-window 3] [--transform yaw x y z]
"""

import argparse
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

LIDAR_CHAIN = ['map', 'offset_base_link', 'base_link', 'velodyne']
DATATYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
DESKEW_BINS = 20         # Poses per sweep for the deskew (0.1 s sweep: one pose every 5 ms)
INTEGRATION_TIMEOUT = 60.0  # s, for octomap_server to integrate one message


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def typestore(reader):
    """Noetic types plus the definitions stored in the bag (tf2_msgs and the custom messages)."""
    store = get_typestore(Stores.ROS1_NOETIC)
    types = {}
    for connection in reader.connections:
        try:
            types.update(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        except Exception:  # noqa: BLE001 (definitions already in the store or not parsable, not needed here)
            pass
    store.register(types)
    return store


class BagTF:
    """Transforms of /tf and /tf_static, interpolated at a stamp (header stamps)."""

    def __init__(self, reader, store):
        samples = {}
        connections = [c for c in reader.connections if c.topic in ('/tf', '/tf_static')]
        for connection, _, raw in reader.messages(connections=connections):
            for transform in store.deserialize_ros1(raw, connection.msgtype).transforms:
                t, r = transform.transform.translation, transform.transform.rotation
                samples.setdefault((transform.header.frame_id, transform.child_frame_id), []).append(
                    (stamp_of(transform), (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)))
        self.edges = {}
        for key, values in samples.items():
            values.sort(key=lambda v: v[0])
            stamps = np.array([v[0] for v in values])
            keep = np.concatenate([[True], np.diff(stamps) > 0])
            stamps = stamps[keep]
            positions = np.array([v[1] for v, k in zip(values, keep) if k])
            rotations = Rotation.from_quat([v[2] for v, k in zip(values, keep) if k])
            self.edges[key] = (stamps, positions, rotations, Slerp(stamps, rotations) if len(stamps) > 1 else None)

    def edge(self, parent, child, stamps):
        times, positions, rotations, slerp = self.edges[(parent, child)]
        if slerp is None:
            return Rotation.concatenate([rotations[0]] * len(stamps)), np.repeat(positions[:1], len(stamps), axis=0)
        stamps = np.clip(stamps, times[0], times[-1])
        return slerp(stamps), np.stack([np.interp(stamps, times, positions[:, k]) for k in range(3)], axis=1)

    def chain(self, frames, stamps):
        """Poses of frames[-1] in frames[0] at the stamps: rotations, positions (N x 3)."""
        stamps = np.atleast_1d(np.asarray(stamps, float))
        rotation, position = None, np.zeros((len(stamps), 3))
        for parent, child in zip(frames[:-1], frames[1:]):
            r, p = self.edge(parent, child, stamps)
            position = position + (p if rotation is None else rotation.apply(p))
            rotation = r if rotation is None else rotation * r
        return rotation, position


def cloud_points(msg, fields):
    formats = {f.name: f for f in msg.fields}
    dtype = np.dtype({'names': fields, 'formats': [DATATYPES[formats[n].datatype] for n in fields],
                      'offsets': [formats[n].offset for n in fields], 'itemsize': msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)


def deskewed_cloud(msg, tf, stamp):
    """Points of a sweep in the lidar frame at the stamp, each one moved from the pose at its own time."""
    points = cloud_points(msg, ['x', 'y', 'z', 'time'])
    xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(float)
    offsets = points['time'].astype(float)
    valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.0)
    xyz, offsets = xyz[valid], offsets[valid]
    bins = np.linspace(offsets.min(), offsets.max(), DESKEW_BINS) if len(offsets) else np.zeros(1)
    index = np.clip(np.searchsorted(bins, offsets), 0, len(bins) - 1)
    rotations, positions = tf.chain(LIDAR_CHAIN, stamp + np.append(bins, 0.0))
    end_inverse = rotations[-1].inv()
    out = np.empty_like(xyz)
    for b in np.unique(index):
        sel = index == b
        world = rotations[b].apply(xyz[sel]) + positions[b]
        out[sel] = end_inverse.apply(world - positions[-1])
    return out


def scan_points(msg, max_range):
    ranges = np.asarray(msg.ranges, float)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges > max(msg.range_min, 0.05)) & (ranges <= min(msg.range_max, max_range))
    return np.stack([ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]),
                     np.zeros(valid.sum())], axis=1)


def ground_shift(scans, tf, window):
    """Vertical shift of the sensor pose (function of the stamp) that keeps the ground at the median flight level."""
    rows = []
    for stamp, msg in scans:
        points = scan_points(msg, 20.0)
        rotation, position = tf.chain(LIDAR_CHAIN, stamp)
        world = rotation[0].apply(points) + position[0]
        horizontal = np.hypot(world[:, 0] - position[0, 0], world[:, 1] - position[0, 1])
        ground = world[(horizontal > 2.0) & (horizontal < 15.0), 2]
        if len(ground) > 20:
            rows.append((stamp, position[0, 2], np.percentile(ground, 5)))
    rows = np.array(rows)
    smoothed = np.array([np.median(rows[np.abs(rows[:, 0] - t) <= window / 2.0, 2]) for t in rows[:, 0]])
    flying = rows[:, 1] - smoothed > 1.5
    reference = np.median(smoothed[flying]) if flying.any() else np.median(smoothed)
    shifts = reference - smoothed
    return lambda stamp: float(np.interp(stamp, rows[:, 0], shifts)), reference


def to_cloud2(points, stamp_msg):
    msg = PointCloud2()
    msg.header.stamp = stamp_msg
    msg.header.frame_id = 'lidar'
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    msg.point_step, msg.row_step = 12, 12 * len(points)
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


class Replayer(Node):

    def __init__(self):
        super().__init__('bag_to_octomap')
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_in', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Octomap, '/octomap_binary', self.on_octomap, 10)
        self.condition = threading.Condition()
        self.octomap_stamp = None

    def on_octomap(self, msg):
        with self.condition:
            self.octomap_stamp = msg.header.stamp
            self.condition.notify_all()

    def integrate(self, points, rotation, position):
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'lidar'
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = (
            float(v) for v in position)
        q = rotation.as_quat()
        transform.transform.rotation.x, transform.transform.rotation.y = float(q[0]), float(q[1])
        transform.transform.rotation.z, transform.transform.rotation.w = float(q[2]), float(q[3])
        self.tf_broadcaster.sendTransform(transform)
        time.sleep(0.02)  # The transform reaches the tf buffer of octomap_server before the cloud
        self.cloud_pub.publish(to_cloud2(points, stamp))
        with self.condition:
            if not self.condition.wait_for(lambda: self.octomap_stamp == stamp, timeout=INTEGRATION_TIMEOUT):
                raise RuntimeError('Cloud not integrated by octomap_server (no /octomap_binary at its stamp)')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag')
    parser.add_argument('--sources', nargs='+', choices=['cloud', 'scan'], default=['cloud', 'scan'])
    parser.add_argument('--cloud-topic', default='/velodyne_points_throttled')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--scan-max-range', type=float, default=20.0, help='m, farther scan points dropped')
    parser.add_argument('--ground-align', action='store_true', help='Vertical shift of the poses keeping the ground level')
    parser.add_argument('--ground-window', type=float, default=3.0, help='s, median window of the ground height')
    parser.add_argument('--transform', nargs=4, type=float, metavar=('YAW', 'X', 'Y', 'Z'),
                        help='Map frame: rotation of YAW deg around z then translation, applied to the bag map frame')
    parser.add_argument('--poses-output', help='CSV of the sensor poses (stamp, x, y, z, vertical shift)')
    args = parser.parse_args()

    topics = {'cloud': args.cloud_topic, 'scan': args.scan_topic}
    with Reader(args.bag) as reader:
        store = typestore(reader)
        tf = BagTF(reader, store)
        sources = {topics[s]: s for s in args.sources}
        messages = []
        for connection, _, raw in reader.messages(connections=[c for c in reader.connections if c.topic in sources]):
            msg = store.deserialize_ros1(raw, connection.msgtype)
            messages.append((stamp_of(msg), sources[connection.topic], msg))
        if args.ground_align:
            connections = [c for c in reader.connections if c.topic == args.scan_topic]
            scans = [(stamp_of(m), m) for m in (store.deserialize_ros1(raw, c.msgtype)
                                                for c, _, raw in reader.messages(connections=connections))]
    messages.sort(key=lambda m: m[0])
    shift = lambda stamp: 0.0  # noqa: E731
    if args.ground_align:
        shift, reference = ground_shift(scans, tf, args.ground_window)
        shifts = [shift(m[0]) for m in messages]
        print('Ground alignment: ground at z = {:.2f} m (flight median), vertical shifts {:.2f} to {:.2f} m'.format(
            reference, min(shifts), max(shifts)))

    rclpy.init()
    node = Replayer()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    while node.count_subscribers('/cloud_in') == 0:
        time.sleep(0.2)
    counts, rows = {s: 0 for s in args.sources}, []
    for stamp, source, msg in messages:
        points = deskewed_cloud(msg, tf, stamp) if source == 'cloud' else scan_points(msg, args.scan_max_range)
        if len(points) == 0:
            continue
        rotation, position = tf.chain(LIDAR_CHAIN, stamp)
        position[0, 2] += shift(stamp)
        if args.transform:
            frame = Rotation.from_euler('z', args.transform[0], degrees=True)
            rotation, position = frame * rotation, frame.apply(position) + np.asarray(args.transform[1:])
        node.integrate(points, rotation[0], position[0])
        counts[source] += 1
        rows.append((stamp, *position[0], shift(stamp)))
    if args.poses_output:
        np.savetxt(args.poses_output, np.array(rows), delimiter=',', header='stamp,x,y,z,shift', comments='', fmt='%.4f')
    node.get_logger().info('Replay done: {}'.format(', '.join('{} {} messages'.format(k, v) for k, v in counts.items())))
    rclpy.try_shutdown()  # Stops the spinner before the node is destroyed
    spinner.join()
    node.destroy_node()


if __name__ == '__main__':
    main()
