#!/usr/bin/env python3
"""
Static forest obstacle publisher for EGO-Swarm v2 benchmarking.

Parses a Gazebo .world file, extracts cylinder obstacles, and publishes:
  - PointCloud2 to /SQ01s/camera/cloud (grid_map cloudCallback)
  - MarkerArray to /dynamic_forest/markers (RViz + collision detection)

The point cloud is filtered to only include obstacles within a configurable
sensing range of the drone's current position (simulating a real LiDAR).
Markers are always published for all obstacles (for RViz visualization
and collision detection).

Usage:
    python3 static_forest.py --world-file /path/to/easy_forest.world
    python3 static_forest.py --world-file /path/to/hard_forest.world --sensing-range 15.0
"""
import argparse
import xml.etree.ElementTree as ET
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header


# ── PointCloud2 helper ──────────────────────────────────────────────────

def make_pointcloud2(points, stamp, frame_id="world"):
    """Create a PointCloud2 message from an Nx3 float32 array."""
    header = Header(stamp=stamp, frame_id=frame_id)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 12
    n = points.shape[0]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.data = points.astype(np.float32).tobytes()
    msg.is_dense = True
    return msg


# ── Cylinder surface point generation ───────────────────────────────────

def cylinder_shell(radius, length, resolution=0.15):
    """
    Generate surface points for a cylinder at given resolution.
    Returns Nx3 float32 array of points on the lateral surface and end caps.
    """
    pts = set()

    # Lateral surface
    n_theta = max(8, int(round(2.0 * np.pi * radius / resolution)))
    n_z = max(2, int(round(length / resolution)))
    thetas = np.linspace(0, 2.0 * np.pi, n_theta, endpoint=False)
    zs = np.linspace(-length / 2.0, length / 2.0, n_z)

    for theta in thetas:
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        for z in zs:
            pts.add((round(x, 4), round(y, 4), round(z, 4)))

    # End caps
    n_r = max(1, int(round(radius / resolution)))
    for ri in range(n_r + 1):
        r = radius * ri / n_r
        if r < 0.001:
            pts.add((0.0, 0.0, round(-length / 2.0, 4)))
            pts.add((0.0, 0.0, round(length / 2.0, 4)))
            continue
        n_th = max(4, int(round(2.0 * np.pi * r / resolution)))
        for theta in np.linspace(0, 2.0 * np.pi, n_th, endpoint=False):
            x = round(r * np.cos(theta), 4)
            y = round(r * np.sin(theta), 4)
            pts.add((x, y, round(-length / 2.0, 4)))
            pts.add((x, y, round(length / 2.0, 4)))

    return np.array(list(pts), dtype=np.float32)


# ── World file parser ───────────────────────────────────────────────────

def parse_world_file(world_path):
    """Parse Gazebo .world file and extract cylinder obstacles.

    Returns list of dicts: {name, x, y, z, radius, length}
    """
    tree = ET.parse(world_path)
    root = tree.getroot()

    obstacles = []
    for model in root.iter('model'):
        name = model.get('name', '')
        if 'ground_plane' in name:
            continue

        # Get pose (x y z roll pitch yaw)
        pose_el = model.find('pose')
        if pose_el is None:
            # Try nested under link
            link = model.find('link')
            if link is not None:
                pose_el = link.find('pose')
        if pose_el is None:
            continue
        pose_parts = pose_el.text.strip().split()
        if len(pose_parts) < 3:
            continue
        x, y, z = float(pose_parts[0]), float(pose_parts[1]), float(pose_parts[2])

        # Get cylinder geometry (from collision or visual)
        cylinder = None
        for cyl_el in model.iter('cylinder'):
            radius_el = cyl_el.find('radius')
            length_el = cyl_el.find('length')
            if radius_el is not None and length_el is not None:
                cylinder = {
                    'radius': float(radius_el.text),
                    'length': float(length_el.text),
                }
                break

        if cylinder is None:
            continue

        obstacles.append({
            'name': name,
            'x': x, 'y': y, 'z': z,
            'radius': cylinder['radius'],
            'length': cylinder['length'],
        })

    return obstacles


# ── Main node ───────────────────────────────────────────────────────────

class StaticForest:
    def __init__(self, args):
        self.publish_rate = args.publish_rate
        self.resolution = args.resolution
        self.sensing_range = args.sensing_range

        # Parse world file
        self.obstacles = parse_world_file(args.world_file)
        rospy.loginfo(f"Loaded {len(self.obstacles)} cylinder obstacles "
                      f"from {args.world_file}")

        # Precompute surface points per obstacle (list of Nx3 arrays)
        self.obstacle_points = []
        self.obstacle_centers = np.empty((len(self.obstacles), 3), dtype=np.float32)
        for i, obs in enumerate(self.obstacles):
            shell = cylinder_shell(obs['radius'], obs['length'], self.resolution)
            shifted = shell + np.array(
                [obs['x'], obs['y'], obs['z']], dtype=np.float32)
            self.obstacle_points.append(shifted)
            self.obstacle_centers[i] = [obs['x'], obs['y'], obs['z']]

        total_pts = sum(p.shape[0] for p in self.obstacle_points)
        rospy.loginfo(f"Total surface points: {total_pts}, "
                      f"sensing range: {self.sensing_range}m")

        # Drone position (updated via odometry callback)
        self.drone_pos = np.array([0.0, 0.0, 2.0], dtype=np.float32)
        self.has_odom = False

        # Subscribers
        self.odom_sub = rospy.Subscriber(
            "/drone_0_visual_slam/odom", Odometry, self.odom_cb)

        # Publishers
        self.cloud_pub = rospy.Publisher(
            "/SQ01s/camera/cloud", PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher(
            "/dynamic_forest/markers", MarkerArray, queue_size=1)

        # Timer
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.timer_cb)

    def odom_cb(self, msg):
        self.drone_pos[0] = msg.pose.pose.position.x
        self.drone_pos[1] = msg.pose.pose.position.y
        self.drone_pos[2] = msg.pose.pose.position.z
        self.has_odom = True

    def timer_cb(self, event):
        if rospy.is_shutdown():
            return
        stamp = rospy.Time.now()

        try:
            # Publish PointCloud2 (only obstacles within sensing range)
            self._publish_cloud(stamp)
            # Publish MarkerArray (always all obstacles for RViz + collision)
            self._publish_markers(stamp)
        except rospy.ROSException:
            pass

    def _publish_cloud(self, stamp):
        """Publish point cloud for obstacles within sensing range of drone."""
        if len(self.obstacle_points) == 0:
            return

        # Compute 2D (x,y) distance from drone to each obstacle center
        dx = self.obstacle_centers[:, 0] - self.drone_pos[0]
        dy = self.obstacle_centers[:, 1] - self.drone_pos[1]
        dists = np.sqrt(dx * dx + dy * dy)

        # Select obstacles within sensing range
        nearby = dists < self.sensing_range
        if not np.any(nearby):
            return

        nearby_points = [self.obstacle_points[i]
                         for i in range(len(self.obstacle_points)) if nearby[i]]
        points = np.vstack(nearby_points)

        cloud_msg = make_pointcloud2(points, stamp)
        self.cloud_pub.publish(cloud_msg)

    def _publish_markers(self, stamp):
        ma = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        ma.markers.append(delete_marker)

        for i, obs in enumerate(self.obstacles):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "world"
            m.ns = "static"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = obs['x']
            m.pose.position.y = obs['y']
            m.pose.position.z = obs['z']
            m.pose.orientation.w = 1.0
            m.scale.x = 2.0 * obs['radius']   # diameter
            m.scale.y = 2.0 * obs['radius']   # diameter
            m.scale.z = obs['length']
            m.color.r = 0.5
            m.color.g = 0.5
            m.color.b = 0.5
            m.color.a = 0.7
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main():
    parser = argparse.ArgumentParser(
        description="Static forest obstacle publisher for EGO-Swarm v2")
    parser.add_argument("--world-file", type=str, required=True,
                        help="Path to Gazebo .world file")
    parser.add_argument("--publish-rate", type=float, default=10.0,
                        help="PointCloud2 / MarkerArray publish rate (Hz)")
    parser.add_argument("--resolution", type=float, default=0.15,
                        help="Surface point resolution (m)")
    parser.add_argument("--sensing-range", type=float, default=15.0,
                        help="Simulated sensor range in meters (default: 15.0). "
                             "Only obstacles within this 2D distance of the drone "
                             "are included in the point cloud.")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("static_forest", anonymous=False)
    forest = StaticForest(args)
    rospy.spin()


if __name__ == "__main__":
    main()
