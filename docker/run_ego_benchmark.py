#!/usr/bin/env python3
"""
EGO-Swarm v2 Dynamic Environment Benchmark Runner

Runs multiple trials of EGO-Swarm v2 with dynamic obstacles and collects
performance data.  Adapted from Intent-MPC run_mpc_benchmark.py for the
EGO-Planner architecture (no Gazebo, internal simulator + dynamic_forest.py).

Usage:
    python3 run_ego_benchmark.py --num-trials 20 --num-obstacles 200

    python3 run_ego_benchmark.py --num-trials 5 --seed-start 10 --timeout 120
"""

import argparse
import csv
import json
import os
import subprocess
import signal
import sys
import time
from dataclasses import dataclass, asdict, field
from datetime import datetime
from pathlib import Path as FilePath
from typing import List, Tuple
import numpy as np
import math

import rospy
import rosgraph
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from visualization_msgs.msg import MarkerArray


@dataclass
class BenchmarkMetrics:
    """Comprehensive metrics for a single trial"""
    # Trial info
    trial_id: int = 0
    seed: int = 0
    num_obstacles: int = 0
    dynamic_ratio: float = 0.0
    timestamp: str = ""

    # Success metrics
    goal_reached: bool = False
    timeout_reached: bool = False
    collision: bool = False

    # Time metrics (seconds)
    flight_travel_time: float = 0.0

    # Path metrics
    path_length: float = 0.0
    straight_line_distance: float = 0.0
    path_efficiency: float = 0.0

    # Smoothness metrics
    jerk_integral: float = 0.0
    jerk_rms: float = 0.0
    avg_velocity: float = 0.0
    max_velocity: float = 0.0
    avg_acceleration: float = 0.0
    max_acceleration: float = 0.0

    # Constraint limits (matching DYNUS benchmark)
    vel_limit: float = 5.0
    acc_limit: float = 20.0
    jerk_limit: float = 100.0

    # Constraint violations
    vel_violation_count: int = 0
    acc_violation_count: int = 0
    jerk_violation_count: int = 0
    vel_violation_max: float = 0.0
    acc_violation_max: float = 0.0
    jerk_violation_max: float = 0.0
    vel_violation_ratio: float = 0.0  # % of timesteps violating
    acc_violation_ratio: float = 0.0
    jerk_violation_ratio: float = 0.0

    # Collision metrics
    collision_count: int = 0
    collision_penetration_max: float = 0.0
    collision_unique_obstacles: int = 0
    min_distance_to_obstacles: float = float('inf')
    collision_free_ratio: float = 1.0

    # Planner configuration
    use_linf_feas: bool = True
    weight_time: float = 10.0
    max_vel_setting: float = 5.0

    # Goal configuration (DYNUS benchmark)
    start_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 2.0])
    goal_position: List[float] = field(default_factory=lambda: [105.0, 0.0, 2.0])

    # Computation time (ms) — from planner's results CSV
    avg_init_time_ms: float = 0.0
    avg_opt_time_ms: float = 0.0
    avg_total_comp_time_ms: float = 0.0

    # Rosbag path
    bag_file: str = ""


class BenchmarkMonitor:
    """ROS1 node to monitor trial progress and collect metrics"""

    def __init__(self, trial_id: int, num_obstacles: int, dynamic_ratio: float, seed: int):
        self.metrics = BenchmarkMetrics(
            trial_id=trial_id,
            seed=seed,
            num_obstacles=num_obstacles,
            dynamic_ratio=dynamic_ratio,
            timestamp=datetime.now().strftime("%Y%m%d_%H%M%S"),
        )

        # State tracking — ACTUAL positions from odometry (for goal/collision checking)
        self.odom_data: List[Tuple[float, np.ndarray]] = []
        self.start_time = None
        self.last_position = None

        # COMMANDED trajectory from pos_cmd (for vel/acc/jerk metrics)
        self.cmd_data: List[Tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []
        self.last_cmd_acc = None
        self.last_cmd_time = None

        # Goal tracking
        self.goal_position = np.array([105.0, 0.0, 2.0])
        self.start_position = np.array([0.0, 0.0, 2.0])
        self.goal_threshold = 0.5
        self.start_threshold = 10.0
        self.waiting_for_start = True
        self.monitor_created_time = time.time()
        self.odom_callback_count = 0

        # Obstacle tracking for collision detection
        self.obstacle_positions = {}  # {marker_id: (x, y, z)}
        self.obstacle_sizes = {}      # {marker_id: (sx, sy, sz)}

        # Point mass collision model (drone_radius = 0)
        self.drone_half_extents = (0, 0, 0)

        # ROS subscribers
        self.odom_sub = rospy.Subscriber(
            '/drone_0_visual_slam/odom', Odometry, self.odom_callback)
        self.cmd_sub = rospy.Subscriber(
            '/drone_0_planning/pos_cmd', PositionCommand, self.cmd_callback)
        self.marker_sub = rospy.Subscriber(
            '/dynamic_forest/markers', MarkerArray, self.marker_callback)

        self.is_complete = False
        self.completion_reason = None

        rospy.loginfo(f"Benchmark monitor initialized for trial {trial_id}")

    def odom_callback(self, msg: Odometry):
        """Collect ACTUAL position from odometry for path length."""
        self.odom_callback_count += 1
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        speed = np.linalg.norm(vel)

        if self.waiting_for_start:
            start_distance = np.linalg.norm(pos - self.start_position)
            elapsed = time.time() - self.monitor_created_time
            if elapsed > 30.0:
                rospy.logwarn(f"Trial {self.metrics.trial_id}: Timeout waiting for start, proceeding")
                self.waiting_for_start = False
            elif start_distance > self.start_threshold:
                rospy.loginfo_throttle(
                    2.0, f"Trial {self.metrics.trial_id}: Waiting for start "
                         f"(dist={start_distance:.1f}m)")
                return
            else:
                self.waiting_for_start = False
                rospy.loginfo(f"Trial {self.metrics.trial_id}: Drone near start, monitoring begun")

        # start_time is set by cmd_callback (first command received)
        if self.start_time is None:
            return

        current_time = rospy.Time.now().to_sec() - self.start_time
        self.odom_data.append((current_time, pos))
        self.last_position = pos

        # Check goal — require near goal AND nearly stopped (< 0.1 m/s)
        goal_distance = np.linalg.norm(pos - self.goal_position)
        goal_speed_threshold = 0.1
        if (goal_distance < self.goal_threshold
                and speed < goal_speed_threshold):
            if not self.is_complete:
                self.is_complete = True
                self.completion_reason = "goal_reached"
                self.metrics.goal_reached = True
                self.metrics.flight_travel_time = current_time
                rospy.loginfo(
                    f"Trial {self.metrics.trial_id}: GOAL REACHED in "
                    f"{current_time:.2f}s (dist={goal_distance:.2f}m, "
                    f"speed={speed:.3f}m/s)")

        # Status every 5s
        rospy.loginfo_throttle(
            5.0, f"Trial {self.metrics.trial_id}: t={current_time:.0f}s, "
                 f"pos=[{pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}], "
                 f"goal_dist={goal_distance:.1f}m, speed={speed:.2f}m/s")

    def cmd_callback(self, msg: PositionCommand):
        """Collect COMMANDED pos/vel/acc from pos_cmd."""
        if self.start_time is None:
            # First command received — this defines the travel time start
            self.start_time = rospy.Time.now().to_sec()
            rospy.loginfo(f"Trial {self.metrics.trial_id}: First pos_cmd received, "
                          f"start_time set")
        current_time = rospy.Time.now().to_sec() - self.start_time
        pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        acc = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z])
        self.cmd_data.append((current_time, pos, vel, acc))
        self.last_cmd_acc = acc
        self.last_cmd_time = current_time

    def marker_callback(self, msg: MarkerArray):
        """Track obstacle positions from dynamic_forest MarkerArray."""
        for marker in msg.markers:
            if marker.action == marker.DELETEALL:
                continue
            key = (marker.ns, marker.id)
            self.obstacle_positions[key] = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            ])
            self.obstacle_sizes[key] = (
                marker.scale.x,
                marker.scale.y,
                marker.scale.z,
            )

        if self.last_position is not None:
            self.check_collisions()

    def check_collisions(self):
        if self.last_position is None:
            return
        drone_bbox = self._make_bbox(self.last_position, self.drone_half_extents)
        for key, obs_pos in self.obstacle_positions.items():
            obs_size = self.obstacle_sizes.get(key, (0.8, 0.8, 0.8))
            obs_half = (obs_size[0] / 2.0, obs_size[1] / 2.0, obs_size[2] / 2.0)
            obs_bbox = self._make_bbox(obs_pos, obs_half)

            # Point-to-AABB Euclidean distance (drone center to obstacle surface)
            pt_dist = self._point_to_aabb_distance(
                self.last_position, obs_pos, obs_half)
            self.metrics.min_distance_to_obstacles = min(
                self.metrics.min_distance_to_obstacles, pt_dist)

            if self._bbox_intersects(drone_bbox, obs_bbox):
                self.metrics.collision_count += 1
                penetration = 0.0  # pt_dist is 0 when inside
                self.metrics.collision_penetration_max = max(
                    self.metrics.collision_penetration_max, penetration)
                if not self.metrics.collision:
                    self.metrics.collision = True
                    rospy.logwarn(
                        f"Trial {self.metrics.trial_id}: Collision with "
                        f"{key[0]}_{key[1]}")

    @staticmethod
    def _make_bbox(c, h):
        return (c[0]-h[0], c[0]+h[0], c[1]-h[1], c[1]+h[1], c[2]-h[2], c[2]+h[2])

    @staticmethod
    def _bbox_intersects(a, b):
        return not (a[1]<b[0] or a[0]>b[1] or a[3]<b[2] or a[2]>b[3] or a[5]<b[4] or a[4]>b[5])

    @staticmethod
    def _bbox_distance(a, b):
        if BenchmarkMonitor._bbox_intersects(a, b):
            return 0.0
        dx = max(0.0, max(a[0], b[0]) - min(a[1], b[1]))
        dy = max(0.0, max(a[2], b[2]) - min(a[3], b[3]))
        dz = max(0.0, max(a[4], b[4]) - min(a[5], b[5]))
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    @staticmethod
    def _point_to_aabb_distance(drone_pos, obs_pos, obs_half):
        """Euclidean distance from drone center (point) to closest point on
        the obstacle AABB surface.  Returns 0 if the point is inside the box.
        """
        dx = max(0.0, abs(drone_pos[0] - obs_pos[0]) - obs_half[0])
        dy = max(0.0, abs(drone_pos[1] - obs_pos[1]) - obs_half[1])
        dz = max(0.0, abs(drone_pos[2] - obs_pos[2]) - obs_half[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def compute_final_metrics(self):
        """Compute all metrics after trial completion."""
        if not self.cmd_data:
            rospy.logwarn(f"Trial {self.metrics.trial_id}: No cmd data")
            return

        rospy.loginfo(f"Trial {self.metrics.trial_id}: Computing metrics "
                      f"({len(self.odom_data)} odom, {len(self.cmd_data)} cmd)")

        # ── Path length from COMMANDED trajectory (pos_cmd) ──────
        cmd_positions = np.array([p for _, p, _, _ in self.cmd_data])
        if len(cmd_positions) > 1:
            diffs = np.diff(cmd_positions, axis=0)
            self.metrics.path_length = float(np.sum(np.linalg.norm(diffs, axis=1)))

        actual_start = cmd_positions[0]
        actual_end = cmd_positions[-1]
        self.metrics.straight_line_distance = float(
            np.linalg.norm(actual_end - actual_start))
        if self.metrics.straight_line_distance > 0:
            self.metrics.path_efficiency = (
                self.metrics.path_length / self.metrics.straight_line_distance)

        # ── Velocity / acceleration / jerk from COMMANDED pos_cmd ─
        # Violation checking ALWAYS uses Linf norm (max absolute per-axis
        # component) regardless of the planner's constraint enforcement mode.
        if len(self.cmd_data) > 1:
            cmd_times = np.array([t for t, _, _, _ in self.cmd_data])
            velocities = np.array([v for _, _, v, _ in self.cmd_data])
            accelerations = np.array([a for _, _, _, a in self.cmd_data])

            def _norm(vecs):
                """Per-timestep Linf norm (max absolute component)."""
                return np.max(np.abs(vecs), axis=1)

            # Velocity metrics
            vel_norms = _norm(velocities)
            valid = vel_norms > 0.01
            if np.any(valid):
                self.metrics.avg_velocity = float(np.mean(vel_norms[valid]))
                self.metrics.max_velocity = float(np.max(vel_norms[valid]))

            # Velocity violations
            n_vel = len(vel_norms)
            vel_over = vel_norms - (self.metrics.vel_limit + 1e-3)
            vel_mask = vel_over > 0
            self.metrics.vel_violation_count = int(np.sum(vel_mask))
            self.metrics.vel_violation_ratio = float(np.sum(vel_mask) / n_vel * 100) if n_vel > 0 else 0.0
            if self.metrics.vel_violation_count > 0:
                self.metrics.vel_violation_max = float(np.max(vel_norms[vel_mask]))

            # Acceleration metrics
            acc_norms = _norm(accelerations)
            self.metrics.avg_acceleration = float(np.mean(acc_norms))
            self.metrics.max_acceleration = float(np.max(acc_norms))

            # Acceleration violations
            n_acc = len(acc_norms)
            acc_over = acc_norms - (self.metrics.acc_limit + 1e-3)
            acc_mask = acc_over > 0
            self.metrics.acc_violation_count = int(np.sum(acc_mask))
            self.metrics.acc_violation_ratio = float(np.sum(acc_mask) / n_acc * 100) if n_acc > 0 else 0.0
            if self.metrics.acc_violation_count > 0:
                self.metrics.acc_violation_max = float(np.max(acc_norms[acc_mask]))

            # Jerk (FD of acceleration from commanded trajectory)
            n = min(len(accelerations), len(cmd_times))
            if n > 1:
                jerk_vecs = []
                for i in range(1, n):
                    dt = cmd_times[i] - cmd_times[i-1]
                    if dt > 0.001:
                        jvec = (accelerations[i] - accelerations[i-1]) / dt
                        jerk_vecs.append(jvec)

                if jerk_vecs:
                    jerk_arr = np.array(jerk_vecs)
                    jerk_norms = _norm(jerk_arr)
                    self.metrics.jerk_rms = float(np.sqrt(np.mean(jerk_norms**2)))
                    avg_dt = float(np.mean(np.diff(cmd_times[:n])))
                    self.metrics.jerk_integral = float(np.sum(jerk_norms) * avg_dt)

                    # Jerk violations
                    n_jerk = len(jerk_norms)
                    jerk_over = jerk_norms - (self.metrics.jerk_limit + 1e-3)
                    jerk_mask = jerk_over > 0
                    self.metrics.jerk_violation_count = int(np.sum(jerk_mask))
                    self.metrics.jerk_violation_ratio = float(np.sum(jerk_mask) / n_jerk * 100) if n_jerk > 0 else 0.0
                    if self.metrics.jerk_violation_count > 0:
                        self.metrics.jerk_violation_max = float(
                            np.max(jerk_norms[jerk_mask]))

        # Collision-free ratio
        if self.metrics.collision_count > 0:
            total = len(self.odom_data)
            col_pts = min(self.metrics.collision_count, total)
            self.metrics.collision_free_ratio = 1.0 - (col_pts / total)
            self.metrics.collision_unique_obstacles = max(
                1, int(self.metrics.collision_count / 10))

        # Sanitize inf
        if self.metrics.min_distance_to_obstacles == float('inf'):
            self.metrics.min_distance_to_obstacles = -1.0

        rospy.loginfo(f"Trial {self.metrics.trial_id}: path={self.metrics.path_length:.2f}m, "
                      f"time={self.metrics.flight_travel_time:.2f}s, "
                      f"collisions={self.metrics.collision_count}")


def parse_planner_comp_times(csv_path="/home/kota/data/results_num_0.csv"):
    """Parse EGO-Planner's computation time CSV.

    The planner destructor writes: Success, Init Comp, Opt Comp (ms)
    Returns (avg_init_ms, avg_opt_ms, avg_total_ms) or (0, 0, 0) on error.
    """
    try:
        init_times = []
        opt_times = []
        with open(csv_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('Success'):
                    continue
                parts = line.split(',')
                if len(parts) >= 3:
                    try:
                        init_ms = float(parts[1].strip())
                        opt_ms = float(parts[2].strip())
                        # Skip sentinel values (-10000) from failed optimizations
                        if init_ms < 0 or opt_ms < 0:
                            continue
                        init_times.append(init_ms)
                        opt_times.append(opt_ms)
                    except ValueError:
                        continue
        if init_times:
            avg_init = sum(init_times) / len(init_times)
            avg_opt = sum(opt_times) / len(opt_times)
            return avg_init, avg_opt, avg_init + avg_opt
    except Exception as e:
        print(f"  Warning: Could not parse comp times: {e}", flush=True)
    return 0.0, 0.0, 0.0


# ── Helpers ──────────────────────────────────────────────────────────────

SCRIPTS = "/home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker"
ENV_SOURCE = ("source /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/"
              "swarm-playground/main_ws/devel/setup.bash"
              " && source /home/kota/mid360_ws/devel/setup.bash")
LAUNCH_DIR = ("/home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/"
              "swarm-playground/main_ws/src/planner/plan_manage/launch")


def run_cmd(cmd):
    return subprocess.Popen(["bash", "-c", cmd], preexec_fn=os.setsid)


def wait_for_ros_master(timeout=30):
    print("Waiting for ROS master...", flush=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            master = rosgraph.Master('/rostopic')
            master.getPid()
            print("ROS master ready", flush=True)
            return True
        except Exception:
            time.sleep(0.5)
    print("ROS master timeout", flush=True)
    return False


def start_rosbag(bag_path: FilePath, topics: List[str]):
    bag_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = ['rosbag', 'record', '-O', str(bag_path)] + topics
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    # Wait for rosbag recorder to fully initialize before planner launches,
    # ensuring no trajectory data is missed at the start of recording.
    time.sleep(10)
    return proc


def kill_ros_processes(keep_roscore=False):
    """Kill ROS processes between trials."""
    procs = ['roslaunch', 'rosout', 'rviz', 'rosbag']
    if not keep_roscore:
        procs.extend(['rosmaster', 'roscore'])

    # Kill python nodes (dynamic_forest, etc.)
    for pat in ['dynamic_forest', 'static_forest', 'ego_planner']:
        try:
            subprocess.run(['pkill', '-9', '-f', pat],
                           stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        except Exception:
            pass

    for p in procs:
        try:
            subprocess.run(['killall', '-9', p],
                           stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        except Exception:
            pass

    if keep_roscore:
        try:
            cleanup = subprocess.Popen(
                ['rosnode', 'cleanup'],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            cleanup.communicate(input=b'y\n', timeout=5)
        except Exception:
            pass

    time.sleep(3)


def patch_launch_file(launch_path, planning_horizon, is_static=False,
                      use_linf_feas=True):
    """Patch <arg default="..."> values in the launch file.

    The launch file declares args (planning_horizon, local_update_range_*,
    fading_time) that flow through the include chain into the node's
    private params.  We update the defaults before roslaunch reads them.
    """
    import re

    with open(launch_path, 'r') as f:
        content = f.read()

    def set_arg_default(xml, arg_name, value):
        return re.sub(
            rf'(<arg\s+name="{arg_name}"\s+default=")[^"]*(")',
            rf'\g<1>{value}\2',
            xml,
        )

    content = set_arg_default(content, 'planning_horizon', planning_horizon)
    content = set_arg_default(content, 'use_linf_feas',
                              'true' if use_linf_feas else 'false')

    if is_static:
        # Enlarge sensing range so the full point cloud is processed.
        # The cloud callback uses local_update_range as a box filter.
        content = set_arg_default(content, 'local_update_range_x', '50.0')
        content = set_arg_default(content, 'local_update_range_y', '20.0')
        content = set_arg_default(content, 'local_update_range_z', '6.0')
        # Static obstacles don't move — don't fade occupied cells
        content = set_arg_default(content, 'fading_time', '10000.0')

    with open(launch_path, 'w') as f:
        f.write(content)

    norm_str = "L-inf" if use_linf_feas else "L2"
    print(f"  Patched launch file: planning_horizon={planning_horizon}, "
          f"use_linf_feas={use_linf_feas} ({norm_str})"
          + (", static sensing (50m range, no fading)" if is_static else ""))


def launch_environment(num_obstacles, dynamic_ratio, seed, rviz=False,
                       weight_time=10.0, max_vel=5.0, static_world=None,
                       planning_horizon=11.25, use_linf_feas=True):
    """Launch everything EXCEPT the planner (which makes the drone move).

    Returns list of subprocess handles.  The caller must start rosbag and
    the benchmark monitor, then call launch_planner() so the full flight
    is captured from the very first odom message.

    Launch order:
      1. Copy launch file + patch it with runtime params
      2. Pre-load planner params onto param server
      3. RViz (if requested)
      4. dynamic_forest (obstacles populate the environment)
    """

    # Copy launch file and inject runtime param overrides
    launch_dst = f"{LAUNCH_DIR}/single_drone_waypoints_dynamic.launch"
    run_cmd(f"cp {SCRIPTS}/single_drone_waypoints_dynamic.launch {launch_dst}")
    patch_launch_file(launch_dst, planning_horizon, is_static=bool(static_world),
                      use_linf_feas=use_linf_feas)

    # Ensure /home/kota/data exists — the planner destructor writes computation
    # times to /home/kota/data/results_num_*.csv (hardcoded in ego_replan_fsm.cpp)
    os.makedirs("/home/kota/data", exist_ok=True)
    time.sleep(0.5)

    # Pre-load planner params so dynamic_forest.py can read planning_horizon
    # and max_vel from the param server before the planner launches.
    run_cmd(f"{ENV_SOURCE} && rosparam set /drone_0_ego_planner_node/fsm/planning_horizon {planning_horizon}")
    run_cmd(f"{ENV_SOURCE} && rosparam set /drone_0_ego_planner_node/manager/planning_horizon {planning_horizon}")
    run_cmd(f"{ENV_SOURCE} && rosparam set /drone_0_ego_planner_node/manager/max_vel {max_vel}")
    run_cmd(f"{ENV_SOURCE} && rosparam set /drone_0_ego_planner_node/optimization/weight_time {weight_time}")
    time.sleep(0.5)

    procs = []

    # RViz first (so visualization is ready)
    if rviz:
        rviz_cmd = f"{ENV_SOURCE} && roslaunch --wait ego_planner rviz.launch"
        print("  Launching RViz...", flush=True)
        procs.append(run_cmd(rviz_cmd))
        time.sleep(3)

    # Obstacle environment
    if static_world:
        forest_cmd = (
            f"{ENV_SOURCE} && python3 {SCRIPTS}/static_forest.py"
            f" --world-file {static_world}"
        )
        print(f"  Launching static_forest ({static_world})...", flush=True)
    else:
        forest_cmd = (
            f"{ENV_SOURCE} && python3 {SCRIPTS}/dynamic_forest.py"
            f" --num-obstacles {num_obstacles}"
            f" --dynamic-ratio {dynamic_ratio}"
            f" --seed {seed}"
        )
        print("  Launching dynamic_forest...", flush=True)
    procs.append(run_cmd(forest_cmd))
    time.sleep(3)

    return procs


def launch_planner():
    """Launch EGO-Planner — the drone starts moving immediately.

    Call this AFTER rosbag and the benchmark monitor are already recording.
    """
    planner_cmd = (
        f"{ENV_SOURCE} && roslaunch --wait ego_planner"
        f" single_drone_waypoints_dynamic.launch simulation_number:=0"
    )
    print("  Launching ego_planner (drone starts moving)...", flush=True)
    proc = run_cmd(planner_cmd)
    time.sleep(5)
    print("  All systems launched", flush=True)
    return proc


def run_trial(trial_id, num_obstacles, dynamic_ratio, seed,
              timeout=100.0, output_dir=None, rviz=False, weight_time=10.0,
              max_vel=5.0, static_world=None, planning_horizon=11.25,
              use_linf_feas=True):
    """Run a single benchmark trial."""

    print("=" * 80, flush=True)
    print(f"TRIAL {trial_id}: seed={seed}, obstacles={num_obstacles}, "
          f"ratio={dynamic_ratio}", flush=True)
    print("=" * 80, flush=True)

    # Pre-trial cleanup (keep roscore)
    if trial_id > 0:
        kill_ros_processes(keep_roscore=True)

    # Phase 1: Launch environment (RViz + obstacles) — drone is NOT moving yet
    sim_procs = launch_environment(num_obstacles, dynamic_ratio, seed, rviz=rviz,
                                   weight_time=weight_time, max_vel=max_vel,
                                   static_world=static_world,
                                   planning_horizon=planning_horizon,
                                   use_linf_feas=use_linf_feas)

    # Init ROS node
    try:
        rospy.init_node(f'ego_benchmark_{trial_id}', anonymous=True,
                        disable_signals=True)
    except rospy.exceptions.ROSException:
        pass  # Already initialized

    # Start rosbag BEFORE the planner so the full flight is captured
    if output_dir is None:
        output_dir = FilePath('/home/kota/data_dynamic')
    bag_dir = output_dir / "bags"
    bag_path = bag_dir / f"trial_{trial_id}.bag"

    topics = [
        '/drone_0_visual_slam/odom',
        '/drone_0_planning/pos_cmd',
        '/drone_0_ego_planner_node/optimal_list',
        '/drone_0_odom_visualization/path',
        '/drone_0_ego_planner_node/grid_map/occupancy_inflate',
    ]
    if not static_world:
        # Only record dynamic obstacle topics when there are dynamic obstacles
        topics += [
            '/dynamic_forest/markers',
            '/broadcast_traj_to_planner',
        ]
    rosbag_proc = start_rosbag(bag_path, topics)

    # Create monitor BEFORE the planner so we capture first odom
    monitor = BenchmarkMonitor(trial_id, num_obstacles, dynamic_ratio, seed)
    monitor.metrics.use_linf_feas = use_linf_feas
    monitor.metrics.weight_time = weight_time
    monitor.metrics.max_vel_setting = max_vel
    monitor.metrics.vel_limit = max_vel

    # Phase 2: Launch planner — drone starts moving NOW
    planner_proc = launch_planner()
    sim_procs.append(planner_proc)

    # Monitor trial
    start_time = time.time()
    rate = rospy.Rate(5)

    try:
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                rospy.logwarn(f"Trial {trial_id}: TIMEOUT after {elapsed:.1f}s")
                monitor.metrics.timeout_reached = True
                monitor.metrics.flight_travel_time = elapsed
                break
            if monitor.is_complete:
                rospy.loginfo(f"Trial {trial_id}: COMPLETED ({monitor.completion_reason})")
                time.sleep(2)  # Grace period
                break
            rate.sleep()
    except KeyboardInterrupt:
        rospy.logwarn(f"Trial {trial_id}: Interrupted")

    finally:
        # Stop rosbag
        try:
            rosbag_proc.send_signal(signal.SIGINT)
            rosbag_proc.wait(timeout=5)
        except Exception:
            rosbag_proc.kill()

        # Compute metrics
        monitor.compute_final_metrics()
        monitor.metrics.bag_file = str(bag_path)

        # Unregister subscribers
        try:
            monitor.odom_sub.unregister()
            monitor.cmd_sub.unregister()
            monitor.marker_sub.unregister()
        except Exception:
            pass

        # Kill simulation processes (SIGTERM triggers planner destructor
        # which writes computation times to /home/kota/data/results_num_0.csv)
        for p in sim_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        time.sleep(3)  # Wait for destructor to write CSV
        for p in sim_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass

        # Parse planner computation times and copy CSV to output dir
        comp_csv = "/home/kota/data/results_num_0.csv"
        avg_init, avg_opt, avg_total = parse_planner_comp_times(comp_csv)
        monitor.metrics.avg_init_time_ms = avg_init
        monitor.metrics.avg_opt_time_ms = avg_opt
        monitor.metrics.avg_total_comp_time_ms = avg_total
        rospy.loginfo(f"Trial {trial_id}: comp_time init={avg_init:.1f}ms, "
                      f"opt={avg_opt:.1f}ms, total={avg_total:.1f}ms")
        # Copy to output dir and clean up for next trial
        if output_dir is not None:
            import shutil
            dest = output_dir / f"comp_times_trial_{trial_id}.csv"
            try:
                shutil.copy2(comp_csv, str(dest))
                os.remove(comp_csv)
            except Exception:
                pass

        kill_ros_processes(keep_roscore=True)
        time.sleep(5)

    return monitor.metrics


def save_metrics_csv(metrics_list, output_path):
    if not metrics_list:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for m in metrics_list:
        row = asdict(m)
        row['start_position'] = str(row['start_position'])
        row['goal_position'] = str(row['goal_position'])
        rows.append(row)
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"Metrics saved to: {output_path}", flush=True)


def save_metrics_json(metrics_list, output_path):
    if not metrics_list:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump([asdict(m) for m in metrics_list], f, indent=2)
    print(f"Metrics saved to: {output_path}", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description='EGO-Swarm v2 dynamic environment benchmark runner')
    parser.add_argument('--num-trials', type=int, default=20)
    parser.add_argument('--num-obstacles', type=int, default=200)
    parser.add_argument('--dynamic-ratio', type=float, default=0.65)
    parser.add_argument('--seed-start', type=int, default=0)
    parser.add_argument('--timeout', type=float, default=100.0)
    parser.add_argument('--output-dir', type=str,
                        default='/home/kota/data_dynamic')
    parser.add_argument('--rviz', action='store_true',
                        help='Launch RViz for visualization during trials')
    parser.add_argument('--weight-time', type=float, default=10.0,
                        help='optimization/weight_time parameter (default: 10.0)')
    parser.add_argument('--max-vel', type=float, default=5.0,
                        help='manager/max_vel parameter (default: 5.0)')
    parser.add_argument('--case-name', type=str, default='hard',
                        help='Case name for output dir prefix (easy/medium/hard)')
    parser.add_argument('--planning-horizon', type=float, default=11.25,
                        help='Planning horizon in meters (default: 11.25)')
    parser.add_argument('--static-world', type=str, default='',
                        help='Path to Gazebo .world file for static environment '
                             '(launches static_forest.py instead of dynamic_forest.py)')
    parser.add_argument('--use-linf-feas', type=int, default=1, choices=[0, 1],
                        help='Use L-inf feasibility constraints (1) or L2 (0)')
    args = parser.parse_args()

    use_linf_feas = bool(args.use_linf_feas)
    norm_suffix = "linf" if use_linf_feas else "l2"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.static_world:
        output_dir = FilePath(args.output_dir) / f"{args.case_name}_static_benchmark_{timestamp}_{norm_suffix}"
    else:
        output_dir = FilePath(args.output_dir) / f"{args.case_name}_benchmark_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 80, flush=True)
    if args.static_world:
        print("EGO-SWARM V2 STATIC BENCHMARK RUNNER", flush=True)
    else:
        print("EGO-SWARM V2 DYNAMIC BENCHMARK RUNNER", flush=True)
    print("=" * 80, flush=True)
    print(f"  Trials: {args.num_trials}", flush=True)
    if args.static_world:
        print(f"  Static world: {args.static_world}", flush=True)
    else:
        print(f"  Obstacles: {args.num_obstacles}", flush=True)
        print(f"  Dynamic ratio: {args.dynamic_ratio}", flush=True)
    print(f"  Seed range: {args.seed_start} – "
          f"{args.seed_start + args.num_trials - 1}", flush=True)
    print(f"  Timeout: {args.timeout}s", flush=True)
    print(f"  Weight time: {args.weight_time}", flush=True)
    print(f"  Max velocity: {args.max_vel}", flush=True)
    print(f"  Planning horizon: {args.planning_horizon}", flush=True)
    print(f"  Use L-inf feas: {use_linf_feas} ({'L-inf' if use_linf_feas else 'L2'})", flush=True)
    print(f"  Case: {args.case_name}", flush=True)
    print(f"  Output: {output_dir}", flush=True)
    print("=" * 80, flush=True)

    # Start roscore
    roscore_running = subprocess.run(
        ['pgrep', 'rosmaster'], capture_output=True).returncode == 0
    roscore_proc = None
    if not roscore_running:
        print("Starting roscore...", flush=True)
        roscore_proc = subprocess.Popen(
            ['roscore'], preexec_fn=os.setsid,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(3)

    if not wait_for_ros_master():
        sys.exit(1)

    metrics_list = []

    for i in range(args.num_trials):
        seed = args.seed_start + i
        print(f"\nStarting Trial {i+1}/{args.num_trials}", flush=True)

        try:
            metrics = run_trial(
                trial_id=i,
                num_obstacles=args.num_obstacles,
                dynamic_ratio=args.dynamic_ratio,
                seed=seed,
                timeout=args.timeout,
                output_dir=output_dir,
                rviz=args.rviz,
                weight_time=args.weight_time,
                max_vel=args.max_vel,
                static_world=args.static_world or None,
                planning_horizon=args.planning_horizon,
                use_linf_feas=use_linf_feas,
            )
            metrics_list.append(metrics)

            # Save intermediate
            csv_path = output_dir / f"benchmark_ego_swarm_v2_{timestamp}.csv"
            save_metrics_csv(metrics_list, csv_path)

            print(f"Trial {i} done: goal={metrics.goal_reached}, "
                  f"time={metrics.flight_travel_time:.2f}s, "
                  f"path={metrics.path_length:.2f}m", flush=True)

        except Exception as e:
            print(f"ERROR in trial {i}: {e}", flush=True)
            import traceback
            traceback.print_exc()
            if metrics_list:
                csv_path = output_dir / f"benchmark_ego_swarm_v2_{timestamp}.csv"
                save_metrics_csv(metrics_list, csv_path)

        # Inter-trial wait
        if i < args.num_trials - 1:
            print("Waiting 8s before next trial...", flush=True)
            time.sleep(8)
            # Verify roscore
            if subprocess.run(['pgrep', 'rosmaster'],
                              capture_output=True).returncode != 0:
                print("rosmaster died, restarting...", flush=True)
                subprocess.Popen(['roscore'], preexec_fn=os.setsid,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                time.sleep(3)

    # Final save
    print("\n" + "=" * 80, flush=True)
    print("BENCHMARK COMPLETE", flush=True)
    print("=" * 80, flush=True)

    csv_path = output_dir / f"benchmark_ego_swarm_v2_{timestamp}.csv"
    json_path = output_dir / f"benchmark_ego_swarm_v2_{timestamp}.json"
    save_metrics_csv(metrics_list, csv_path)
    save_metrics_json(metrics_list, json_path)

    if metrics_list:
        success = sum(1 for m in metrics_list if m.goal_reached)
        collisions = sum(1 for m in metrics_list if m.collision)
        timeouts = sum(1 for m in metrics_list if m.timeout_reached)
        n = len(metrics_list)
        print(f"\nSummary:", flush=True)
        print(f"  Total trials: {n}", flush=True)
        print(f"  Successful: {success} ({100*success/n:.1f}%)", flush=True)
        print(f"  Collisions: {collisions} ({100*collisions/n:.1f}%)", flush=True)
        print(f"  Timeouts: {timeouts} ({100*timeouts/n:.1f}%)", flush=True)
        print(f"\nResults: {output_dir}", flush=True)


if __name__ == '__main__':
    main()
