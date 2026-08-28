#!/usr/bin/env python3
"""
Dynamic forest obstacle generator for EGO-Swarm v2 benchmarking.

Hybrid approach matching the DYNUS dynamic obstacle environment:
  - Static obstacles  → PointCloud2 to /SQ01s/camera/cloud (grid_map cloudCallback)
  - Dynamic obstacles → MINCOTraj  to /broadcast_traj_to_planner (swarm avoidance)
  - MarkerArray to /dynamic_forest/markers (RViz visualization for both)

Only dynamic obstacles within the drone's planning horizon are published as
MINCOTraj, keeping the optimizer loop efficient.

Trefoil knot formula (matching DYNUS dyn_obstacles.launch.py):
  x = (sx/6)*(sin(tt) + 2*sin(2*tt)) + x0
  y = (sy/5)*(cos(tt) - 2*cos(2*tt)) + y0
  z = (sz/2)*(-sin(3*tt)) + z0
  where tt = t/slower + offset
"""
import argparse
import json
import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from traj_utils.msg import MINCOTraj


# ── Trefoil helpers (analytic position / velocity / acceleration) ────────

def trefoil_pos(obs, t):
    """Evaluate trefoil knot position at wall-clock time t."""
    tt = t / obs["slower"] + obs["offset"]
    x = (obs["sx"] / 6.0) * (np.sin(tt) + 2.0 * np.sin(2.0 * tt)) + obs["x0"]
    y = (obs["sy"] / 5.0) * (np.cos(tt) - 2.0 * np.cos(2.0 * tt)) + obs["y0"]
    z = (obs["sz"] / 2.0) * (-np.sin(3.0 * tt)) + obs["z0"]
    return np.array([x, y, z])


def trefoil_vel(obs, t):
    """Analytic first derivative of the trefoil w.r.t. wall-clock time."""
    s = obs["slower"]
    tt = t / s + obs["offset"]
    dx = (obs["sx"] / 6.0) * (np.cos(tt) + 4.0 * np.cos(2.0 * tt)) / s
    dy = (obs["sy"] / 5.0) * (-np.sin(tt) + 4.0 * np.sin(2.0 * tt)) / s
    dz = (obs["sz"] / 2.0) * (-3.0 * np.cos(3.0 * tt)) / s
    return np.array([dx, dy, dz])


def trefoil_acc(obs, t):
    """Analytic second derivative of the trefoil w.r.t. wall-clock time."""
    s = obs["slower"]
    s2 = s * s
    tt = t / s + obs["offset"]
    ddx = (obs["sx"] / 6.0) * (-np.sin(tt) - 8.0 * np.sin(2.0 * tt)) / s2
    ddy = (obs["sy"] / 5.0) * (-np.cos(tt) + 8.0 * np.cos(2.0 * tt)) / s2
    ddz = (obs["sz"] / 2.0) * (9.0 * np.sin(3.0 * tt)) / s2
    return np.array([ddx, ddy, ddz])


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


def voxel_shell(sx, sy, sz, res):
    """
    Generate voxel center offsets for the surface shell of a box (sx x sy x sz)
    at given resolution.  Returns Nx3 float32 array.
    """
    nx = max(1, int(round(sx / res)))
    ny = max(1, int(round(sy / res)))
    nz = max(1, int(round(sz / res)))

    xs = np.linspace(-sx / 2, sx / 2, nx)
    ys = np.linspace(-sy / 2, sy / 2, ny)
    zs = np.linspace(-sz / 2, sz / 2, nz)

    pts = set()
    for xi in xs:
        for yi in ys:
            pts.add((xi, yi, zs[0]))
            pts.add((xi, yi, zs[-1]))
    for xi in xs:
        for zi in zs:
            pts.add((xi, ys[0], zi))
            pts.add((xi, ys[-1], zi))
    for yi in ys:
        for zi in zs:
            pts.add((xs[0], yi, zi))
            pts.add((xs[-1], yi, zi))

    return np.array(list(pts), dtype=np.float32)


# ── MINCOTraj builder ──────────────────────────────────────────────────

def build_minco_msg(obs, t_now, prediction_horizon, drone_id, des_clearance,
                    n_pieces=8):
    """
    Build a MINCOTraj message for a dynamic obstacle.

    Samples the trefoil at n_pieces+1 time points over [t_now, t_now+prediction_horizon],
    uses exact position/velocity/acceleration at start and end, and inner waypoint
    positions for intermediate points.  The planner's MinJerkOpt will fit a 5th-order
    polynomial that closely approximates the true trefoil.
    """
    dt = prediction_horizon / n_pieces

    p0 = trefoil_pos(obs, t_now)
    v0 = trefoil_vel(obs, t_now)
    a0 = trefoil_acc(obs, t_now)

    t_end = t_now + prediction_horizon
    pN = trefoil_pos(obs, t_end)
    vN = trefoil_vel(obs, t_end)
    aN = trefoil_acc(obs, t_end)

    # Inner waypoints (piece junctions 1 .. n_pieces-1)
    inner_x, inner_y, inner_z = [], [], []
    for i in range(1, n_pieces):
        p_i = trefoil_pos(obs, t_now + i * dt)
        inner_x.append(float(p_i[0]))
        inner_y.append(float(p_i[1]))
        inner_z.append(float(p_i[2]))

    msg = MINCOTraj()
    msg.drone_id = drone_id
    msg.traj_id = 0
    msg.start_time = rospy.Time.now()
    msg.order = 5
    msg.des_clearance = des_clearance

    msg.start_p = [float(p0[0]), float(p0[1]), float(p0[2])]
    msg.start_v = [float(v0[0]), float(v0[1]), float(v0[2])]
    msg.start_a = [float(a0[0]), float(a0[1]), float(a0[2])]
    msg.end_p   = [float(pN[0]), float(pN[1]), float(pN[2])]
    msg.end_v   = [float(vN[0]), float(vN[1]), float(vN[2])]
    msg.end_a   = [float(aN[0]), float(aN[1]), float(aN[2])]

    msg.inner_x = inner_x
    msg.inner_y = inner_y
    msg.inner_z = inner_z
    msg.duration = [float(dt)] * n_pieces

    return msg


# ── Main node ───────────────────────────────────────────────────────────

class DynamicForest:
    # Dynamic obstacle drone_id offset (avoid conflict with drone 0)
    DYN_ID_OFFSET = 100

    def __init__(self, args):
        self.rng = np.random.RandomState(args.seed)
        self.num_obstacles = args.num_obstacles
        self.dynamic_ratio = args.dynamic_ratio
        self.resolution = args.resolution
        self.publish_rate = args.publish_rate
        self.des_clearance = args.des_clearance

        # ── Read planner params from ROS parameter server ───────────
        self.planning_horizon = rospy.get_param(
            "/drone_0_ego_planner_node/manager/planning_horizon", 4.5)
        self.max_vel = rospy.get_param(
            "/drone_0_ego_planner_node/manager/max_vel", 2.0)
        # Spatial radius for filtering which obstacles get MINCOTraj.
        # Must cover: drone travel + obstacle travel over prediction window.
        self.horizon_radius = self.planning_horizon * self.max_vel * 3.0
        rospy.loginfo(f"planning_horizon={self.planning_horizon}s, "
                      f"max_vel={self.max_vel}m/s, "
                      f"horizon_radius={self.horizon_radius:.1f}m")

        # ── Load or generate obstacle parameters ────────────────────
        if args.obstacles_json:
            self._load_from_json(args.obstacles_json)
        else:
            self._generate_obstacles(args)

        n_dynamic = len(self.dynamic_obs)
        n_static = len(self.static_obs)

        # Precompute static points (they never move)
        static_pts = []
        for obs in self.static_obs:
            shifted = obs["voxels"] + np.array(
                [obs["x0"], obs["y0"], obs["z0"]], dtype=np.float32)
            static_pts.append(shifted)
        self.static_points = (np.vstack(static_pts) if static_pts
                              else np.empty((0, 3), dtype=np.float32))

        rospy.loginfo(f"Dynamic forest: {n_dynamic} dynamic (MINCOTraj) "
                      f"+ {n_static} static (pointcloud) "
                      f"= {self.num_obstacles} obstacles")
        rospy.loginfo(f"Static voxel points: {self.static_points.shape[0]}, "
                      f"publish rate: {self.publish_rate} Hz")

        # ── Drone pose subscriber ───────────────────────────────────
        self.drone_pos = None
        rospy.Subscriber("/drone_0_visual_slam/odom", Odometry,
                         self._odom_cb, queue_size=1)

        # ── Publishers ──────────────────────────────────────────────
        self.cloud_pub = rospy.Publisher(
            "/SQ01s/camera/cloud", PointCloud2, queue_size=1)
        self.traj_pub = rospy.Publisher(
            "/broadcast_traj_to_planner", MINCOTraj, queue_size=10)
        self.marker_pub = rospy.Publisher(
            "/dynamic_forest/markers", MarkerArray, queue_size=1)

        # ── Timer ───────────────────────────────────────────────────
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.timer_cb)
        self.t0 = rospy.get_time()

    def _generate_obstacles(self, args):
        """Generate obstacles from seed (original behavior)."""
        n_dynamic = int(round(self.num_obstacles * self.dynamic_ratio))
        n_static = self.num_obstacles - n_dynamic

        self.dynamic_obs = []
        for _ in range(n_dynamic):
            x0 = self.rng.uniform(args.x_min, args.x_max)
            y0 = self.rng.uniform(args.y_min, args.y_max)
            z0 = self.rng.uniform(args.z_min, args.z_max)
            sx = self.rng.uniform(2.0, 4.0)
            sy = self.rng.uniform(2.0, 4.0)
            sz = self.rng.uniform(2.0, 4.0)
            offset = self.rng.uniform(0.0, 3.0)
            slower = self.rng.uniform(4.0, 6.0)
            self.dynamic_obs.append({
                "x0": x0, "y0": y0, "z0": z0,
                "sx": sx, "sy": sy, "sz": sz,
                "offset": offset, "slower": slower,
                "bbox": (0.8, 0.8, 0.8),
            })

        self.static_obs = []
        n_static_vert = int(n_static * 0.35)
        for i in range(n_static):
            x0 = self.rng.uniform(args.x_min, args.x_max)
            y0 = self.rng.uniform(args.y_min, args.y_max)
            z0 = self.rng.uniform(args.z_min, args.z_max)
            if i < n_static_vert:
                bx, by, bz = 0.4, 0.4, 4.0
                z0 = bz / 2.0
            else:
                bx, by, bz = 0.4, 4.0, 0.4
            voxels = voxel_shell(bx, by, bz, self.resolution)
            self.static_obs.append({
                "x0": x0, "y0": y0, "z0": z0,
                "voxels": voxels,
                "bbox": (bx, by, bz),
            })

    def _load_from_json(self, json_path):
        """Load obstacles from shared benchmark JSON config."""
        rospy.loginfo(f"Loading obstacles from JSON: {json_path}")
        with open(json_path) as f:
            raw = json.load(f)

        # Support both {"metadata": ..., "obstacles": [...]} and flat list
        if isinstance(raw, dict) and "obstacles" in raw:
            obs_list = raw["obstacles"]
        else:
            obs_list = raw

        self.dynamic_obs = []
        self.static_obs = []
        for item in obs_list:
            is_dynamic = item.get("is_dynamic", item.get("scale_x", 0) > 0)
            if is_dynamic:
                self.dynamic_obs.append({
                    "x0": item["x0"], "y0": item["y0"], "z0": item["z0"],
                    "sx": item["scale_x"], "sy": item["scale_y"], "sz": item["scale_z"],
                    "offset": item["offset"], "slower": item["slower"],
                    "bbox": (item.get("size_x", 0.8),
                             item.get("size_y", 0.8),
                             item.get("size_z", 0.8)),
                })
            else:
                bx = item.get("size_x", 0.4)
                by = item.get("size_y", 0.4)
                bz = item.get("size_z", 0.4)
                voxels = voxel_shell(bx, by, bz, self.resolution)
                self.static_obs.append({
                    "x0": item["x0"], "y0": item["y0"], "z0": item["z0"],
                    "voxels": voxels,
                    "bbox": (bx, by, bz),
                })

        self.num_obstacles = len(self.dynamic_obs) + len(self.static_obs)
        rospy.loginfo(f"Loaded {len(self.dynamic_obs)} dynamic + "
                      f"{len(self.static_obs)} static obstacles from JSON")

    def _odom_cb(self, msg):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

    def timer_cb(self, event):
        if rospy.is_shutdown():
            return
        t = rospy.get_time() - self.t0
        stamp = rospy.Time.now()

        try:
            # ── Static obstacles: publish pointcloud ─────────────────
            if self.static_points.shape[0] > 0:
                cloud_msg = make_pointcloud2(self.static_points, stamp)
                self.cloud_pub.publish(cloud_msg)

            # ── Dynamic obstacles: publish MINCOTraj for nearby ones ─
            dyn_positions = []
            nearby_count = 0
            for i, obs in enumerate(self.dynamic_obs):
                pos = trefoil_pos(obs, t)
                dyn_positions.append(pos)

                # Only publish trajectory if drone pose is known and obstacle
                # is within the horizon radius
                if self.drone_pos is not None:
                    dist = np.linalg.norm(pos - self.drone_pos)
                    if dist > self.horizon_radius:
                        continue

                drone_id = self.DYN_ID_OFFSET + i
                msg = build_minco_msg(
                    obs, t,
                    prediction_horizon=self.planning_horizon,
                    drone_id=drone_id,
                    des_clearance=self.des_clearance,
                )
                self.traj_pub.publish(msg)
                nearby_count += 1

            rospy.loginfo_throttle(
                5.0, f"Publishing MINCOTraj for {nearby_count}/{len(self.dynamic_obs)} "
                     f"nearby dynamic obstacles")

            # ── MarkerArray for RViz ─────────────────────────────────
            self._publish_markers(stamp, dyn_positions)

        except rospy.ROSException:
            pass  # Node shutting down

    def _publish_markers(self, stamp, dyn_positions):
        ma = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        ma.markers.append(delete_marker)

        marker_id = 0

        # Static obstacles
        for obs in self.static_obs:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "world"
            m.ns = "static"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = obs["x0"]
            m.pose.position.y = obs["y0"]
            m.pose.position.z = obs["z0"]
            m.pose.orientation.w = 1.0
            m.scale.x = obs["bbox"][0]
            m.scale.y = obs["bbox"][1]
            m.scale.z = obs["bbox"][2]
            m.color.r = 0.5
            m.color.g = 0.5
            m.color.b = 0.5
            m.color.a = 0.7
            ma.markers.append(m)

        # Dynamic obstacles
        for i, (obs, pos) in enumerate(zip(self.dynamic_obs, dyn_positions)):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "world"
            m.ns = "dynamic"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x = obs["bbox"][0]
            m.scale.y = obs["bbox"][1]
            m.scale.z = obs["bbox"][2]
            m.color.r = 1.0
            m.color.g = 0.3
            m.color.b = 0.0
            m.color.a = 0.7
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main():
    parser = argparse.ArgumentParser(
        description="Dynamic forest obstacle generator for EGO-Swarm v2")
    parser.add_argument("--num-obstacles", type=int, default=200)
    parser.add_argument("--dynamic-ratio", type=float, default=0.65)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--publish-rate", type=float, default=50.0)
    parser.add_argument("--resolution", type=float, default=0.15)
    parser.add_argument("--des-clearance", type=float, default=0.4,
                        help="Per-obstacle clearance radius for MINCOTraj "
                             "(half of 0.8m bbox)")
    parser.add_argument("--obstacles-json", type=str, default=None,
                        help="Path to shared benchmark obstacle JSON config. "
                             "When set, overrides seed-based generation.")
    # Spawn ranges matching DYNUS benchmark
    parser.add_argument("--x-min", type=float, default=5.0)
    parser.add_argument("--x-max", type=float, default=100.0)
    parser.add_argument("--y-min", type=float, default=-6.0)
    parser.add_argument("--y-max", type=float, default=6.0)
    parser.add_argument("--z-min", type=float, default=0.5)
    parser.add_argument("--z-max", type=float, default=4.5)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("dynamic_forest", anonymous=False)
    forest = DynamicForest(args)
    rospy.spin()


if __name__ == "__main__":
    main()
