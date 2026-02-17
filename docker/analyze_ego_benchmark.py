#!/usr/bin/env python3
"""
EGO-Swarm v2 Dynamic Benchmark Analysis

Single script that:
1. Post-processes collisions from rosbags (drone odom + obstacle markers)
2. Computes comprehensive statistics
3. Updates DYNUS LaTeX table
4. Generates summary CSV

Usage:
    python3 analyze_ego_benchmark.py --data-dir /home/kota/data_dynamic/benchmark_20260207_120000

    # Skip collision post-processing (just compute stats from CSV)
    python3 analyze_ego_benchmark.py --data-dir ... --skip-collision-postprocess
"""

import argparse
import sys
import math
from pathlib import Path
import pandas as pd
import numpy as np

# Try to import rosbag for collision post-processing
try:
    import rosbag
    HAS_ROSBAG = True
except ImportError:
    HAS_ROSBAG = False
    print("Warning: rosbag not available. Will skip collision post-processing.")
    print("Install with: pip install --extra-index-url https://rospypi.github.io/simple/ rosbag")


# ============================================================================
# COLLISION POST-PROCESSING (from rosbags)
# ============================================================================

class BoundingBox:
    """Axis-aligned bounding box for collision detection"""
    def __init__(self, min_x, max_x, min_y, max_y, min_z, max_z):
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.min_z, self.max_z = min_z, max_z

    @classmethod
    def from_center_and_half_extents(cls, cx, cy, cz, hx, hy, hz):
        return cls(cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz)

    def intersects(self, other):
        return not (self.max_x < other.min_x or self.min_x > other.max_x or
                    self.max_y < other.min_y or self.min_y > other.max_y or
                    self.max_z < other.min_z or self.min_z > other.max_z)

    def distance_to(self, other):
        if self.intersects(other):
            return 0.0
        dx = max(0.0, max(self.min_x, other.min_x) - min(self.max_x, other.max_x))
        dy = max(0.0, max(self.min_y, other.min_y) - min(self.max_y, other.max_y))
        dz = max(0.0, max(self.min_z, other.min_z) - min(self.max_z, other.max_z))
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def interpolate_position(times, positions, t_query):
    """Interpolate position at a given time."""
    if len(times) == 0:
        return np.zeros(3)
    if len(times) == 1:
        return positions[0]
    if t_query <= times[0]:
        return positions[0]
    if t_query >= times[-1]:
        return positions[-1]
    idx = np.searchsorted(times, t_query)
    if idx >= len(times):
        return positions[-1]
    t0, t1 = times[idx - 1], times[idx]
    p0, p1 = positions[idx - 1], positions[idx]
    alpha = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
    return p0 + alpha * (p1 - p0)


def analyze_collision_from_bag(bag_path: Path, drone_bbox=(0.1, 0.1, 0.1)):
    """
    Analyze collisions from rosbag.

    Reads drone trajectory from /drone_0_visual_slam/odom and obstacle
    positions from /dynamic_forest/markers (MarkerArray with CUBE markers
    whose scale encodes the obstacle bounding box).
    """
    if not bag_path.exists():
        return {'collision_count': 0, 'min_distance': float('inf'),
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"ERROR opening bag: {e}")
        return {'collision_count': 0, 'min_distance': -1.0,
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    bag_topics = bag.get_type_and_topic_info()[1].keys()
    print(f"topics={list(bag_topics)}", end=" ", flush=True)

    # ── Read drone trajectory ──────────────────────────────────────────
    drone_trajectory = []
    for topic, msg, t in bag.read_messages(topics=['/drone_0_visual_slam/odom']):
        drone_trajectory.append((
            t.to_sec(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ))

    # ── Read obstacle snapshots from MarkerArray ───────────────────────
    # Each MarkerArray message is a full snapshot (DELETEALL + all markers).
    # We store the latest snapshot per timestamp → build per-obstacle traj.
    obstacle_snapshots = {}  # {(ns, id): [(t, x, y, z)]}
    obstacle_sizes = {}      # {(ns, id): (sx, sy, sz)}

    for topic, msg, t in bag.read_messages(topics=['/dynamic_forest/markers']):
        t_sec = t.to_sec()
        for marker in msg.markers:
            # Skip the DELETEALL sentinel
            if marker.action == 0:  # ADD
                key = (marker.ns, marker.id)
                if key not in obstacle_snapshots:
                    obstacle_snapshots[key] = []
                    obstacle_sizes[key] = (
                        marker.scale.x, marker.scale.y, marker.scale.z)
                obstacle_snapshots[key].append((
                    t_sec,
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                ))

    bag.close()

    if not drone_trajectory:
        print("WARNING: No drone trajectory in bag!")
        return {'collision_count': 0, 'min_distance': -1.0,
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    drone_times = np.array([t for t, x, y, z in drone_trajectory])
    drone_positions = np.array([[x, y, z] for t, x, y, z in drone_trajectory])

    # Build numpy arrays for each obstacle
    obstacle_data = {}
    for key, traj in obstacle_snapshots.items():
        if traj:
            times = np.array([t for t, x, y, z in traj])
            positions = np.array([[x, y, z] for t, x, y, z in traj])
            sx, sy, sz = obstacle_sizes[key]
            obstacle_data[key] = {
                'times': times,
                'positions': positions,
                'half_extents': (sx / 2.0, sy / 2.0, sz / 2.0),
            }

    print(f"drone_pts={len(drone_trajectory)}, obstacles={len(obstacle_data)}",
          end=" ", flush=True)

    if not obstacle_data:
        print("WARNING: No obstacles found in bag!")
        return {'collision_count': 0, 'min_distance': -1.0,
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    # ── Check collisions at every drone timestamp ──────────────────────
    collisions = 0
    min_distance = float('inf')
    max_penetration = 0.0
    collision_free_segments = 0
    collided_obstacles = set()
    drone_hx, drone_hy, drone_hz = drone_bbox

    for t, px, py, pz in zip(drone_times, drone_positions[:, 0],
                              drone_positions[:, 1], drone_positions[:, 2]):
        drone_bb = BoundingBox.from_center_and_half_extents(
            px, py, pz, drone_hx, drone_hy, drone_hz)
        segment_collision_free = True

        for key, obs_data in obstacle_data.items():
            obs_pos = interpolate_position(
                obs_data['times'], obs_data['positions'], t)
            hx, hy, hz = obs_data['half_extents']
            obs_bb = BoundingBox.from_center_and_half_extents(
                obs_pos[0], obs_pos[1], obs_pos[2], hx, hy, hz)

            # Point-to-AABB Euclidean distance (drone center to obstacle surface)
            dx = max(0.0, abs(px - obs_pos[0]) - hx)
            dy = max(0.0, abs(py - obs_pos[1]) - hy)
            dz = max(0.0, abs(pz - obs_pos[2]) - hz)
            pt_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            min_distance = min(min_distance, pt_dist)

            if drone_bb.intersects(obs_bb):
                segment_collision_free = False
                collisions += 1
                collided_obstacles.add(key)

        if segment_collision_free:
            collision_free_segments += 1

    return {
        'collision_count': collisions,
        'min_distance': min_distance if min_distance != float('inf') else -1.0,
        'collision_free_ratio': (
            collision_free_segments / len(drone_positions)
            if len(drone_positions) > 0 else 1.0),
        'collision_penetration_max': max_penetration,
        'collision_unique_obstacles': len(collided_obstacles),
    }


def load_obstacle_csv(csv_path: str):
    """Load static obstacle parameters from CSV.

    CSV columns: id,shape,x,y,z,radius,width,depth,height,yaw
    Returns list of dicts with cylinder geometry.
    """
    import csv as csv_mod
    cylinders = []
    with open(csv_path, 'r') as f:
        reader = csv_mod.DictReader(f)
        for row in reader:
            cylinders.append({
                'id': int(row['id']),
                'x': float(row['x']),
                'y': float(row['y']),
                'z': float(row['z']),
                'radius': float(row['radius']),
                'height': float(row['height']),
            })
    return cylinders


def point_to_cylinder_distance(px, py, pz, cyl):
    """Euclidean distance from point (px,py,pz) to cylinder surface.

    Cylinder spans z=[0, height] with center at (cyl.x, cyl.y).
    Returns 0.0 if the point is inside the cylinder.
    """
    dx = px - cyl['x']
    dy = py - cyl['y']
    horiz_dist = math.sqrt(dx * dx + dy * dy)
    horiz_gap = max(0.0, horiz_dist - cyl['radius'])

    # Vertical gap: cylinder spans z=0 to z=height
    if pz < 0.0:
        vert_gap = -pz
    elif pz > cyl['height']:
        vert_gap = pz - cyl['height']
    else:
        vert_gap = 0.0

    return math.sqrt(horiz_gap * horiz_gap + vert_gap * vert_gap)


def analyze_collision_from_csv(bag_path: Path, csv_path: str,
                                drone_radius: float = 0):
    """Analyze collisions between drone trajectory and static CSV obstacles.

    Reads drone positions from /drone_0_visual_slam/odom in the rosbag,
    checks each against cylinders defined in the CSV file.
    Uses point mass collision model (drone_radius=0): collision occurs when
    the drone center is inside the cylinder.
    """
    if not bag_path.exists():
        return {'collision_count': 0, 'min_distance': float('inf'),
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    cylinders = load_obstacle_csv(csv_path)

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"ERROR opening bag: {e}")
        return {'collision_count': 0, 'min_distance': -1.0,
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    # Read drone trajectory
    drone_trajectory = []
    for topic, msg, t in bag.read_messages(
            topics=['/drone_0_visual_slam/odom']):
        drone_trajectory.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ))
    bag.close()

    if not drone_trajectory:
        print("WARNING: No drone trajectory in bag!")
        return {'collision_count': 0, 'min_distance': -1.0,
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    print(f"drone_pts={len(drone_trajectory)}, cylinders={len(cylinders)}",
          end=" ", flush=True)

    # Check collisions at every drone position
    collisions = 0
    min_distance = float('inf')
    max_penetration = 0.0
    collision_free_segments = 0
    collided_obstacles = set()

    for px, py, pz in drone_trajectory:
        segment_collision_free = True

        for cyl in cylinders:
            dist = point_to_cylinder_distance(px, py, pz, cyl)
            min_distance = min(min_distance, dist)

            # Collision: point mass — drone center inside the cylinder
            dx = px - cyl['x']
            dy = py - cyl['y']
            horiz_dist = math.sqrt(dx * dx + dy * dy)

            if horiz_dist <= cyl['radius'] and 0 <= pz <= cyl['height']:
                segment_collision_free = False
                collisions += 1
                collided_obstacles.add(cyl['id'])
                penetration = cyl['radius'] - horiz_dist
                max_penetration = max(max_penetration, penetration)

        if segment_collision_free:
            collision_free_segments += 1

    return {
        'collision_count': collisions,
        'min_distance': min_distance if min_distance != float('inf') else -1.0,
        'collision_free_ratio': (
            collision_free_segments / len(drone_trajectory)
            if len(drone_trajectory) > 0 else 1.0),
        'collision_penetration_max': max_penetration,
        'collision_unique_obstacles': len(collided_obstacles),
    }


def recompute_metrics_from_bag(bag_path: Path,
                               goal_position=(105.0, 0.0, 2.0),
                               goal_dist_threshold=0.5,
                               goal_speed_threshold=0.1,
                               start_pos=(0.0, 0.0, 2.0)):
    """Recompute travel time and path length from rosbag pos_cmd topic.

    Definitions:
      - travel_time start = timestamp of FIRST pos_cmd message
      - travel_time end   = first timestamp where distance_to_goal < 0.5
                            AND velocity magnitude < 0.1 m/s
      - path_length       = sum of Euclidean distances between consecutive
                            commanded positions up to the goal-arrival index

    Incomplete-bag compensation:
      If the first recorded position is far from start_pos (> 0.5 m),
      the bag likely started late.  The straight-line gap distance is
      prepended to path_length, and missing time is estimated as
      gap_dist / first_recorded_speed (fallback: gap_dist / 1.0 m/s).

    Returns dict with 'travel_time' and 'path_length', or None values
    if goal was not reached.
    """
    if not bag_path.exists():
        return {'travel_time': None, 'path_length': None}

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"ERROR opening bag for metrics: {e}")
        return {'travel_time': None, 'path_length': None}

    goal = np.array(goal_position)
    start = np.array(start_pos)
    positions = []
    first_speed = None
    start_time = None
    end_time = None

    for topic, msg, t in bag.read_messages(
            topics=['/drone_0_planning/pos_cmd']):
        t_sec = t.to_sec()
        pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])

        if start_time is None:
            start_time = t_sec
            first_speed = float(np.linalg.norm(vel))

        positions.append(pos)

        # Check goal reached: near goal AND nearly stopped
        dist_to_goal = np.linalg.norm(pos - goal)
        speed = np.linalg.norm(vel)
        if dist_to_goal < goal_dist_threshold and speed < goal_speed_threshold:
            end_time = t_sec
            break

    bag.close()

    if not positions:
        return {'travel_time': None, 'path_length': None}

    # Path length: sum of consecutive Euclidean distances (up to goal arrival)
    pos_array = np.array(positions)
    if len(pos_array) > 1:
        diffs = np.diff(pos_array, axis=0)
        path_length = float(np.sum(np.linalg.norm(diffs, axis=1)))
    else:
        path_length = 0.0

    # Detect incomplete bag: first position far from known start
    gap_dist = float(np.linalg.norm(pos_array[0] - start))
    missing_time = 0.0
    if gap_dist > 0.5:
        path_length += gap_dist
        if first_speed is not None and first_speed > 1e-3:
            missing_time = gap_dist / first_speed
        else:
            missing_time = gap_dist / 1.0
        print(f"[incomplete bag] gap={gap_dist:.2f}m, "
              f"missing_time={missing_time:.2f}s ", end="", flush=True)

    if end_time is not None and start_time is not None:
        travel_time = (end_time - start_time) + missing_time
    else:
        travel_time = None

    return {'travel_time': travel_time, 'path_length': path_length}


def recompute_metrics_from_bags(data_dir: Path, df: pd.DataFrame,
                                goal_position=(105.0, 0.0, 2.0)):
    """Recompute travel time and path length from rosbags for all trials.

    Overrides flight_travel_time and path_length in the DataFrame.
    """
    bags_dir = data_dir / "bags"

    if not bags_dir.exists() or not HAS_ROSBAG:
        if not HAS_ROSBAG:
            print("  Skipping metrics recomputation (rosbag not available)")
        else:
            print("  Skipping metrics recomputation (no bags directory)")
        return df

    print("\n" + "=" * 80)
    print("RECOMPUTING TRAVEL TIME & PATH LENGTH FROM ROSBAGS")
    print(f"  Goal position: {list(goal_position)}")
    print("=" * 80)

    updated = 0
    for idx, row in df.iterrows():
        trial_id = row['trial_id']
        bag_path = bags_dir / f"trial_{trial_id}.bag"

        print(f"\nTrial {trial_id}: ", end="", flush=True)

        if not bag_path.exists():
            print("bag not found")
            continue

        result = recompute_metrics_from_bag(bag_path,
                                            goal_position=goal_position)

        if result['travel_time'] is not None:
            df.at[idx, 'flight_travel_time'] = result['travel_time']
            df.at[idx, 'path_length'] = result['path_length']
            print(f"travel_time={result['travel_time']:.2f}s, "
                  f"path_length={result['path_length']:.2f}m")
            updated += 1
        else:
            print(f"goal not reached in bag, "
                  f"path_length={result['path_length']:.2f}m"
                  if result['path_length'] is not None
                  else "no pos_cmd data")
            # Still update path_length if we have it
            if result['path_length'] is not None:
                df.at[idx, 'path_length'] = result['path_length']

    print(f"\n\nRecomputed metrics for {updated}/{len(df)} trials")
    return df


def recompute_violations_from_bag(bag_path: Path, vel_limit=5.0,
                                   acc_limit=20.0, jerk_limit=100.0,
                                   tolerance=1e-3):
    """Recompute constraint violations from rosbag pos_cmd topic.

    Matches the DYNUS benchmark methodology:
      - Data source: /drone_0_planning/pos_cmd (PositionCommand) which
        contains velocity, acceleration, AND jerk at each timestep.
      - Norm: L-infinity (per-axis).  A violation at a single timestep
        occurs when ANY axis component exceeds limit + tolerance.
      - Rate: violating_timesteps / total_timesteps * 100%.

    Returns dict with per-type violation count, total samples, ratio (%),
    max violating value, and overall max velocity/acceleration.
    """
    result = {
        'vel_violation_ratio': 0.0, 'acc_violation_ratio': 0.0,
        'jerk_violation_ratio': 0.0,
        'vel_violation_count': 0, 'acc_violation_count': 0,
        'jerk_violation_count': 0,
        'vel_violation_total': 0, 'acc_violation_total': 0,
        'jerk_violation_total': 0,
        'vel_violation_max': 0.0, 'acc_violation_max': 0.0,
        'jerk_violation_max': 0.0,
        'max_velocity': 0.0, 'max_acceleration': 0.0,
    }
    if not bag_path.exists():
        return result

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"ERROR opening bag: {e}")
        return result

    velocities = []
    accelerations = []
    jerks = []

    for topic, msg, t in bag.read_messages(
            topics=['/drone_0_planning/pos_cmd']):
        velocities.append([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        accelerations.append([msg.acceleration.x, msg.acceleration.y,
                              msg.acceleration.z])
        jerks.append([msg.jerk.x, msg.jerk.y, msg.jerk.z])
    bag.close()

    if len(velocities) < 2:
        return result

    velocities = np.array(velocities)
    accelerations = np.array(accelerations)
    jerks = np.array(jerks)

    def _linf(vecs):
        """Per-timestep L-inf norm (max absolute component)."""
        return np.max(np.abs(vecs), axis=1)

    n_samples = len(velocities)

    # Velocity: violation if max(|vx|, |vy|, |vz|) > vel_limit + tolerance
    vel_norms = _linf(velocities)
    vel_mask = vel_norms > (vel_limit + tolerance)
    result['vel_violation_count'] = int(np.sum(vel_mask))
    result['vel_violation_total'] = n_samples
    result['vel_violation_ratio'] = float(np.sum(vel_mask) / n_samples * 100)
    if result['vel_violation_count'] > 0:
        result['vel_violation_max'] = float(np.max(vel_norms[vel_mask]))
    result['max_velocity'] = float(np.max(vel_norms))

    # Acceleration: violation if max(|ax|, |ay|, |az|) > acc_limit + tolerance
    acc_norms = _linf(accelerations)
    acc_mask = acc_norms > (acc_limit + tolerance)
    result['acc_violation_count'] = int(np.sum(acc_mask))
    result['acc_violation_total'] = n_samples
    result['acc_violation_ratio'] = float(np.sum(acc_mask) / n_samples * 100)
    if result['acc_violation_count'] > 0:
        result['acc_violation_max'] = float(np.max(acc_norms[acc_mask]))
    result['max_acceleration'] = float(np.max(acc_norms))

    # Jerk: read directly from message (same as DYNUS methodology)
    # violation if max(|jx|, |jy|, |jz|) > jerk_limit + tolerance
    jerk_norms = _linf(jerks)
    jerk_mask = jerk_norms > (jerk_limit + tolerance)
    result['jerk_violation_count'] = int(np.sum(jerk_mask))
    result['jerk_violation_total'] = n_samples
    result['jerk_violation_ratio'] = float(np.sum(jerk_mask) / n_samples * 100)
    if result['jerk_violation_count'] > 0:
        result['jerk_violation_max'] = float(np.max(jerk_norms[jerk_mask]))

    return result


def recompute_violations_from_bags(data_dir: Path, df: pd.DataFrame):
    """Recompute constraint violation ratios from rosbags for all trials.

    ALWAYS uses Linf norm for violation checking.
    Overrides violation columns in the DataFrame.
    """
    bags_dir = data_dir / "bags"

    if not bags_dir.exists() or not HAS_ROSBAG:
        if not HAS_ROSBAG:
            print("  Skipping violation recomputation (rosbag not available)")
        else:
            print("  Skipping violation recomputation (no bags directory)")
        return df

    print("\n" + "=" * 80)
    print("RECOMPUTING CONSTRAINT VIOLATIONS FROM ROSBAGS (L-inf)")
    print("=" * 80)

    updated = 0
    for idx, row in df.iterrows():
        trial_id = row['trial_id']
        bag_path = bags_dir / f"trial_{trial_id}.bag"

        vel_limit = row.get('vel_limit', 5.0)
        acc_limit = row.get('acc_limit', 20.0)
        jerk_limit = row.get('jerk_limit', 100.0)

        print(f"\nTrial {trial_id}: ", end="", flush=True)

        if not bag_path.exists():
            print("bag not found")
            continue

        result = recompute_violations_from_bag(
            bag_path, vel_limit=vel_limit, acc_limit=acc_limit,
            jerk_limit=jerk_limit)

        for key, val in result.items():
            df.at[idx, key] = val

        print(f"vel={result['vel_violation_ratio']:.2f}%, "
              f"acc={result['acc_violation_ratio']:.2f}%, "
              f"jerk={result['jerk_violation_ratio']:.2f}%")
        updated += 1

    print(f"\n\nRecomputed violations for {updated}/{len(df)} trials")
    return df


def postprocess_collisions(data_dir: Path, df: pd.DataFrame,
                            obstacle_csv: str = None):
    """Post-process collisions from rosbags.

    If obstacle_csv is provided, uses CSV-based cylinder collision checking
    (for static benchmarks). Otherwise uses marker-based AABB checking
    (for dynamic benchmarks).
    """
    bags_dir = data_dir / "bags"

    if not bags_dir.exists() or not HAS_ROSBAG:
        if not HAS_ROSBAG:
            print("  Skipping collision post-processing (rosbag not available)")
        else:
            print("  Skipping collision post-processing (no bags directory)")
        return df

    mode = "CSV obstacles" if obstacle_csv else "marker-based"
    print("\n" + "=" * 80)
    print(f"POST-PROCESSING COLLISIONS FROM ROSBAGS ({mode})")
    print("=" * 80)

    if obstacle_csv:
        print(f"  Obstacle CSV: {obstacle_csv}")

    for idx, row in df.iterrows():
        trial_id = row['trial_id']
        bag_path = bags_dir / f"trial_{trial_id}.bag"

        print(f"\nTrial {trial_id}: ", end="", flush=True)

        if not bag_path.exists():
            print("bag not found")
            continue

        if obstacle_csv:
            result = analyze_collision_from_csv(bag_path, obstacle_csv)
        else:
            result = analyze_collision_from_bag(bag_path)

        df.at[idx, 'collision_count'] = result['collision_count']
        df.at[idx, 'min_distance_to_obstacles'] = result['min_distance']
        df.at[idx, 'collision_free_ratio'] = result['collision_free_ratio']
        df.at[idx, 'collision_penetration_max'] = result['collision_penetration_max']
        df.at[idx, 'collision_unique_obstacles'] = result['collision_unique_obstacles']
        df.at[idx, 'collision'] = result['collision_count'] > 0

        min_dist_str = (f"{result['min_distance']:.3f}m"
                        if result['min_distance'] >= 0 else "N/A")
        print(f"-> collisions={result['collision_count']}, "
              f"min_dist={min_dist_str}")

    output_csv = data_dir / "benchmark_ego_swarm_v2_postprocessed.csv"
    df.to_csv(output_csv, index=False)
    print(f"\nPost-processed data saved: {output_csv}")

    return df


# ============================================================================
# STATISTICS AND ANALYSIS
# ============================================================================

def load_benchmark_data(data_dir: Path):
    """Load benchmark CSV."""
    csv_files = [f for f in data_dir.glob("benchmark_ego_swarm_v2_*.csv")
                 if 'summary' not in f.name.lower()
                 and 'postprocessed' not in f.name.lower()]

    if not csv_files:
        print(f"ERROR: No benchmark CSV found in {data_dir}")
        sys.exit(1)

    csv_file = sorted(csv_files, key=lambda f: f.stat().st_mtime)[-1]
    print(f"Loading: {csv_file.name}")
    return pd.read_csv(csv_file)


def recompute_comp_times(data_dir: Path, df: pd.DataFrame):
    """Recompute computation times from raw comp_times_trial_*.csv files.

    The planner writes per-optimization rows: Success, Init Comp, Opt Comp (ms).
    Failed optimizations are marked with -10000 sentinels which must be filtered.
    This fixes runs where sentinels were accidentally included in the average.
    """
    any_fixed = False
    for idx, row in df.iterrows():
        trial_id = row['trial_id']
        csv_path = data_dir / f"comp_times_trial_{trial_id}.csv"
        if not csv_path.exists():
            continue

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
                        if init_ms < 0 or opt_ms < 0:
                            continue
                        init_times.append(init_ms)
                        opt_times.append(opt_ms)
                    except ValueError:
                        continue

        if init_times:
            avg_init = sum(init_times) / len(init_times)
            avg_opt = sum(opt_times) / len(opt_times)
            old_opt = row.get('avg_opt_time_ms', 0)
            if old_opt <= 0 or abs(old_opt - avg_opt) > 0.1:
                df.at[idx, 'avg_init_time_ms'] = avg_init
                df.at[idx, 'avg_opt_time_ms'] = avg_opt
                df.at[idx, 'avg_total_comp_time_ms'] = avg_init + avg_opt
                any_fixed = True
                print(f"  Trial {trial_id}: recomputed comp times "
                      f"(init={avg_init:.2f}ms, opt={avg_opt:.2f}ms, "
                      f"n={len(init_times)} valid entries)")

    if any_fixed:
        print("  Comp times recomputed from raw CSVs (filtered -10000 sentinels)")
    return df


def compute_statistics(df: pd.DataFrame):
    """Compute comprehensive statistics.

    IMPORTANT:
    - Success is defined as goal_reached AND collision_free (DYNUS methodology)
    - Performance, safety, and constraint violation metrics are computed
      ONLY from successful runs (which are already collision-free by definition)
    """
    stats = {'total_trials': len(df)}

    # Determine collision-free status
    if 'collision_count' in df.columns:
        collision_free_mask = df['collision_count'] == 0
    else:
        collision_free_mask = df['collision'] == False

    # SUCCESS = goal_reached AND collision_free (DYNUS definition)
    success_mask = (df['goal_reached'] == True) & collision_free_mask
    stats['success_rate'] = success_mask.mean() * 100

    # Breakdown stats
    stats['goal_reached_rate'] = df['goal_reached'].mean() * 100
    stats['collision_free_rate'] = collision_free_mask.mean() * 100
    stats['timeout_rate'] = df['timeout_reached'].mean() * 100
    stats['collision_rate'] = (~collision_free_mask).mean() * 100

    if 'collision_count' in df.columns:
        stats['collision_count_total'] = int(df['collision_count'].sum())

    # Successful runs
    successful = df[success_mask]
    stats['n_successful'] = len(successful)
    valid_runs = successful
    stats['n_valid_runs'] = len(valid_runs)

    print(f"\n  Success defined as: goal_reached AND collision_free")
    print(f"  Computing metrics from {len(valid_runs)} successful runs")
    print(f"    - Total trials: {len(df)}")
    print(f"    - Goal reached: {(df['goal_reached'] == True).sum()}")
    print(f"    - Collision-free: {collision_free_mask.sum()}")
    print(f"    - Successful (both): {len(valid_runs)} "
          f"({stats['success_rate']:.1f}%)")

    if len(valid_runs) == 0:
        print("  WARNING: No valid runs to compute performance metrics!")
        return stats

    # Performance metrics (from valid runs only)
    for metric in ['flight_travel_time', 'path_length', 'path_efficiency',
                   'jerk_rms', 'jerk_integral', 'avg_velocity', 'max_velocity',
                   'avg_acceleration', 'max_acceleration']:
        if metric in valid_runs.columns:
            values = valid_runs[metric].dropna()
            if len(values) > 0:
                stats[f'{metric}_mean'] = float(values.mean())
                stats[f'{metric}_std'] = float(values.std())

    # Computation time (from valid runs only)
    for metric in ['avg_init_time_ms', 'avg_opt_time_ms', 'avg_total_comp_time_ms']:
        if metric in valid_runs.columns:
            values = valid_runs[metric].dropna()
            values = values[values > 0]
            if len(values) > 0:
                stats[f'{metric}_mean'] = float(values.mean())
                stats[f'{metric}_std'] = float(values.std())

    # Constraint violations (from valid runs only)
    # Overall violation rate = sum(all violations across all bags)
    #                        / sum(all total samples across all bags) * 100
    for viol_type in ['vel', 'acc', 'jerk']:
        count_col = f'{viol_type}_violation_count'
        total_col = f'{viol_type}_violation_total'
        if count_col in valid_runs.columns and total_col in valid_runs.columns:
            total_violations = valid_runs[count_col].sum()
            total_samples = valid_runs[total_col].sum()
            stats[f'{viol_type}_violation_rate'] = (
                float(total_violations / total_samples * 100.0)
                if total_samples > 0 else 0.0)
        elif count_col in valid_runs.columns:
            # Fallback: binary (% of trials with any violation)
            viol_rate = (valid_runs[count_col] > 0).mean() * 100
            stats[f'{viol_type}_violation_rate'] = viol_rate

    # Safety metrics - min distance (from valid runs only)
    if 'min_distance_to_obstacles' in valid_runs.columns:
        values = valid_runs['min_distance_to_obstacles'].dropna()
        values = values[(values != float('inf')) & (values >= 0)]
        if len(values) > 0:
            stats['min_distance_to_obstacles_mean'] = float(values.mean())
            stats['min_distance_to_obstacles_std'] = float(values.std())
        else:
            stats['min_distance_to_obstacles_mean'] = None

    return stats


def print_statistics(stats: dict):
    """Print formatted statistics."""
    print("\n" + "=" * 80)
    print("BENCHMARK ANALYSIS RESULTS")
    print("=" * 80)
    print(f"\nOVERVIEW")
    print(f"  Total trials: {stats.get('total_trials', 0)}")
    print(f"  SUCCESS RATE (R^succ): {stats.get('success_rate', 0):.1f}%")
    print(f"    (Success = goal_reached AND collision_free)")
    print(f"  Successful runs: {stats.get('n_successful', 0)}")
    print(f"\n  Breakdown:")
    print(f"    - Goal reached: {stats.get('goal_reached_rate', 0):.1f}%")
    print(f"    - Collision-free: {stats.get('collision_free_rate', 0):.1f}%")
    print(f"    - Timeout: {stats.get('timeout_rate', 0):.1f}%")

    n_valid = stats.get('n_successful', 0)
    print(f"\nNOTE: Performance metrics computed from {n_valid} "
          f"successful runs only")

    if 'avg_total_comp_time_ms_mean' in stats:
        print(f"\nCOMPUTATION TIME (per optimization, from successful runs)")
        print(f"  Init time: "
              f"{stats.get('avg_init_time_ms_mean', 0):.2f} +/- "
              f"{stats.get('avg_init_time_ms_std', 0):.2f} ms")
        print(f"  Opt time:  "
              f"{stats.get('avg_opt_time_ms_mean', 0):.2f} +/- "
              f"{stats.get('avg_opt_time_ms_std', 0):.2f} ms")
        print(f"  Total:     "
              f"{stats.get('avg_total_comp_time_ms_mean', 0):.2f} +/- "
              f"{stats.get('avg_total_comp_time_ms_std', 0):.2f} ms")

    if 'flight_travel_time_mean' in stats:
        print(f"\nPERFORMANCE (from successful runs)")
        print(f"  Travel time: "
              f"{stats['flight_travel_time_mean']:.2f} +/- "
              f"{stats.get('flight_travel_time_std', 0):.2f} s")
        print(f"  Path length: "
              f"{stats['path_length_mean']:.2f} +/- "
              f"{stats.get('path_length_std', 0):.2f} m")
        print(f"  Path efficiency: "
              f"{stats.get('path_efficiency_mean', 0):.3f}")

    if 'avg_velocity_mean' in stats:
        print(f"\nVELOCITY / ACCELERATION (from successful runs)")
        print(f"  Avg velocity: "
              f"{stats['avg_velocity_mean']:.2f} +/- "
              f"{stats.get('avg_velocity_std', 0):.2f} m/s")
        print(f"  Max velocity: "
              f"{stats.get('max_velocity_mean', 0):.2f} +/- "
              f"{stats.get('max_velocity_std', 0):.2f} m/s")
        print(f"  Avg acceleration: "
              f"{stats.get('avg_acceleration_mean', 0):.2f} +/- "
              f"{stats.get('avg_acceleration_std', 0):.2f} m/s^2")
        print(f"  Max acceleration: "
              f"{stats.get('max_acceleration_mean', 0):.2f} +/- "
              f"{stats.get('max_acceleration_std', 0):.2f} m/s^2")

    if 'jerk_integral_mean' in stats:
        print(f"\nSMOOTHNESS (from successful runs)")
        print(f"  Jerk integral: {stats['jerk_integral_mean']:.2f}")
        print(f"  Jerk RMS: {stats.get('jerk_rms_mean', 0):.2f}")

    print(f"\nSAFETY (from successful runs)")
    min_dist = stats.get('min_distance_to_obstacles_mean')
    if min_dist is not None:
        print(f"  Min distance to obstacles (d_min): {min_dist:.3f} +/- "
              f"{stats.get('min_distance_to_obstacles_std', 0):.3f} m")
    else:
        print(f"  Min distance to obstacles: N/A (no obstacle data)")

    if 'collision_count_total' in stats:
        print(f"  Total collision events: {stats['collision_count_total']}")

    print(f"\nCONSTRAINT VIOLATIONS (from successful runs)")
    for viol in ['vel', 'acc', 'jerk']:
        if f'{viol}_violation_rate' in stats:
            print(f"  {viol.upper()} violation rate: "
                  f"{stats[f'{viol}_violation_rate']:.1f}%")


def _remove_multirow_group(lines, group_name):
    """Remove all rows belonging to a multirow group, including preceding midrule."""
    result = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if '\\multirow' in line and f'{{{group_name}}}' in line:
            # Remove preceding \midrule if present
            if result and '\\midrule' in result[-1]:
                result.pop()
            # Skip this line and subsequent continuation rows
            i += 1
            while i < len(lines):
                if lines[i].strip().startswith('&'):
                    i += 1
                else:
                    break
            continue
        result.append(line)
        i += 1
    return result


def update_dynus_latex_table(stats: dict, dynus_table_path: Path,
                             algorithm_name: str = "EGO-v2",
                             multirow_group: str = None,
                             multirow_size: int = 4,
                             multirow_index: int = 3,
                             case_value: str = None):
    """Update DYNUS LaTeX table with EGO-Swarm v2 results.

    Table format (11 columns): Case | Algorithm | 9 data cols.
    Case groups use \\multirow{4}{*}{Easy/Medium/Hard}, each containing
    DYNUS, I-MPC, FAPP, EGO-v2 rows.

    Standalone mode:  \\multicolumn{2}{c}{Name} & data...
    Multirow mode:    finds \\multirow{4}{*}{Case} block, overwrites
                      the row at multirow_index (EGO-v2 = index 3).
    """
    if not dynus_table_path.exists():
        print(f"\nWarning: DYNUS table not found at {dynus_table_path}")
        return None

    success_rate = stats.get('success_rate', 0)
    # Per-optimization time: use opt_time (matches DYNUS's avg_local_traj_time)
    per_opt_time = stats.get('avg_opt_time_ms_mean', 0) or 0
    travel_time = stats.get('flight_travel_time_mean', 0)
    path_length = stats.get('path_length_mean', 0)
    jerk_integral = stats.get('jerk_integral_mean', 0)
    min_distance = stats.get('min_distance_to_obstacles_mean', 0)
    vel_viol = stats.get('vel_violation_rate', 0)
    acc_viol = stats.get('acc_violation_rate', 0)
    jerk_viol = stats.get('jerk_violation_rate', 0)

    min_dist_str = (f"{min_distance:.3f}"
                    if isinstance(min_distance, (int, float))
                    and min_distance is not None else "N/A")

    data_cols = (
        f"{success_rate:.1f} & {per_opt_time:.1f} & {travel_time:.1f} & "
        f"{path_length:.1f} & {jerk_integral:.1f} & {min_dist_str} & "
        f"{vel_viol:.1f} & {acc_viol:.1f} & {jerk_viol:.1f}"
    )

    if multirow_group and case_value is not None:
        # Case-based multirow: multirow_group is the case (Easy/Medium/Hard),
        # algorithm_name goes in the 2nd column (e.g. EGO-v2).
        if multirow_index == 0:
            prefix = (f"\\multirow{{{multirow_size}}}{{*}}"
                      f"{{{multirow_group}}} & {algorithm_name}")
        else:
            prefix = f" & {algorithm_name}"

        ego_row = f"      {prefix} & {data_cols} \\\\"
        match_key = algorithm_name + " &"
    else:
        # Standalone format (spans 2 algorithm columns)
        ego_row = (f"      \\multicolumn{{2}}{{c}}{{{algorithm_name}}}"
                   f" & {data_cols} \\\\")
        match_key = algorithm_name + " &"

    with open(dynus_table_path, 'r') as f:
        lines = f.readlines()

    new_lines = list(lines)

    if multirow_group:
        # Multirow mode: find the group's \multirow line and overwrite
        # the row at group_start + multirow_index.  This is safe because
        # it targets only the exact group by name (e.g. {EGO-v2}).
        group_start = None
        for i, line in enumerate(new_lines):
            if '\\multirow' in line and f'{{{multirow_group}}}' in line:
                group_start = i
                break

        if group_start is not None:
            # Count actual rows in the block (first row + continuation & rows)
            block_size = 1
            while (group_start + block_size < len(new_lines)
                   and new_lines[group_start + block_size].strip().startswith('&')):
                block_size += 1

            if multirow_index < block_size:
                # Overwrite existing row in-place
                new_lines[group_start + multirow_index] = ego_row + "\n"
            else:
                # Append beyond current block (shouldn't happen normally)
                new_lines.insert(group_start + block_size, ego_row + "\n")
        else:
            # Block doesn't exist yet — insert before \bottomrule
            for i, line in enumerate(new_lines):
                if '\\bottomrule' in line:
                    if multirow_index == 0:
                        new_lines.insert(i, "      \\midrule\n")
                        new_lines.insert(i + 1, ego_row + "\n")
                    else:
                        new_lines.insert(i, ego_row + "\n")
                    break
    else:
        # Standalone mode: match/replace existing row, or insert new
        existing_idx = None
        for i, line in enumerate(new_lines):
            if match_key in line:
                existing_idx = i
                break

        if existing_idx is not None:
            new_lines[existing_idx] = ego_row + "\n"
        else:
            for i, line in enumerate(new_lines):
                if '\\bottomrule' in line:
                    new_lines.insert(i, ego_row + "\n")
                    break

    with open(dynus_table_path, 'w') as f:
        f.writelines(new_lines)

    row_desc = (f"{multirow_group} / {algorithm_name}"
                if multirow_group and case_value is not None
                else algorithm_name)
    print(f"\nUpdated DYNUS table: {dynus_table_path}")
    print(f"  Added/updated {row_desc} row")
    return dynus_table_path


def update_static_latex_table(stats: dict, table_path: Path,
                               algorithm_name: str = "EGO-Swarm2",
                               multirow_group: str = None,
                               multirow_size: int = 5,
                               multirow_index: int = 0,
                               constraint_norm: str = 'linf'):
    """Update the static_benchmark.tex table with benchmark results.

    Static table format (13 columns):
      Env & Algorithm & Constr.(type) & Constr.(norm) & R_succ &
      T^total_opt & T^total_replan & T_trav & L_path & S_jerk &
      rho_vel & rho_acc & rho_jerk

    Env groups use \\multirow{5}{*}{Easy/Medium/Hard}, each containing
    EGO-Swarm2, SUPER(L2), SUPER(Linf), FASTER, DYNUS rows.

    multirow_group:  case name (Easy/Medium/Hard)
    multirow_index:  0-based position of EGO-Swarm2 within block (default 0)
    """
    if not table_path.exists():
        print(f"\nWarning: Static table not found at {table_path}")
        return None

    success_rate = stats.get('success_rate', 0)
    opt_time = stats.get('avg_opt_time_ms_mean', 0) or 0
    total_time = stats.get('avg_total_comp_time_ms_mean', 0) or 0
    travel_time = stats.get('flight_travel_time_mean', 0)
    path_length = stats.get('path_length_mean', 0)
    jerk_integral = stats.get('jerk_integral_mean', 0)
    vel_viol = stats.get('vel_violation_rate', 0)
    acc_viol = stats.get('acc_violation_rate', 0)
    jerk_viol = stats.get('jerk_violation_rate', 0)

    # Data columns: Constr.type & Constr.norm & R_succ & T_opt & T_replan &
    #               T_trav & L_path & S_jerk & rho_vel & rho_acc & rho_jerk
    if constraint_norm == 'l2':
        constr_type = '\\multirow{2}{*}{Soft}'
        norm_label = '$L_2$'
    else:
        constr_type = ''  # multirow continuation
        norm_label = '$L_\\infty$'
    data_cols = (
        f"{constr_type} & {norm_label} & {{{success_rate:.1f}}} & {{{opt_time:.1f}}} & "
        f"{{{total_time:.1f}}} & {{{travel_time:.1f}}} & "
        f"{{{path_length:.1f}}} & {{{jerk_integral:.1f}}} & "
        f"{{{vel_viol:.1f}}} & {{{acc_viol:.1f}}} & {{{jerk_viol:.1f}}}"
    )

    if multirow_group:
        if multirow_index == 0:
            prefix = (f"\\multirow{{{multirow_size}}}{{*}}"
                      f"{{{multirow_group}}} & {algorithm_name}")
        else:
            prefix = f" & {algorithm_name}"
        row = f"      {prefix} & {data_cols} \\\\"
    else:
        row = f"      {algorithm_name} & {data_cols} \\\\"

    with open(table_path, 'r') as f:
        lines = f.readlines()

    new_lines = list(lines)

    if multirow_group:
        # Find the group's \multirow line and overwrite at multirow_index
        group_start = None
        for i, line in enumerate(new_lines):
            if '\\multirow' in line and f'{{{multirow_group}}}' in line:
                group_start = i
                break

        if group_start is not None:
            block_size = 1
            while (group_start + block_size < len(new_lines)
                   and new_lines[group_start + block_size].strip().startswith('&')):
                block_size += 1

            if multirow_index < block_size:
                new_lines[group_start + multirow_index] = row + "\n"
            else:
                new_lines.insert(group_start + block_size, row + "\n")
        else:
            # Block doesn't exist — insert before \bottomrule
            for i, line in enumerate(new_lines):
                if '\\bottomrule' in line:
                    new_lines.insert(i, "      \\midrule\n")
                    new_lines.insert(i + 1, row + "\n")
                    break
    else:
        # No multirow — match/replace by algorithm name
        found = False
        for i, line in enumerate(new_lines):
            stripped = line.strip()
            if stripped.startswith(algorithm_name) and '&' in stripped:
                new_lines[i] = row + "\n"
                found = True
                break
        if not found:
            for i, line in enumerate(new_lines):
                if '\\bottomrule' in line:
                    new_lines.insert(i, row + "\n")
                    break

    with open(table_path, 'w') as f:
        f.writelines(new_lines)

    row_desc = (f"{multirow_group} / {algorithm_name}"
                if multirow_group else algorithm_name)
    print(f"\nUpdated static table: {table_path}")
    print(f"  Updated {row_desc} row")
    return table_path


def main():
    parser = argparse.ArgumentParser(
        description='EGO-Swarm v2 benchmark analysis '
                    '(post-processing + statistics + LaTeX)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument('--data-dir', type=str, required=True,
                        help='Path to benchmark data directory')
    parser.add_argument('--algorithm-name', type=str, default='EGO-v2',
                        help='Algorithm name for LaTeX table (default: EGO-v2)')
    parser.add_argument('--dynus-table-path', type=str,
                        default='/home/kkondo/paper_writing/DYNUS_v3/tables/'
                                'dynamic_benchmark.tex',
                        help='Path to DYNUS LaTeX table')
    parser.add_argument('--skip-collision-postprocess', action='store_true',
                        help='Skip collision post-processing from rosbags')
    parser.add_argument('--multirow-group', type=str, default='',
                        help='Multirow group name (e.g., EGO-v2) for sweep tables')
    parser.add_argument('--multirow-size', type=int, default=4,
                        help='Number of algorithms per case block (default: 4)')
    parser.add_argument('--multirow-index', type=int, default=3,
                        help='0-based index of EGO-v2 within case block (default: 3)')
    parser.add_argument('--case-value', type=str, default='',
                        help='Case name (easy/medium/hard) — triggers multirow mode')
    parser.add_argument('--table-format', type=str, default='dynamic',
                        choices=['dynamic', 'static'],
                        help='Table format: dynamic (default) or static')
    parser.add_argument('--obstacle-csv', type=str, default='',
                        help='Path to obstacle CSV for static collision checking '
                             '(e.g., easy_forest_obstacle_parameters.csv)')
    parser.add_argument('--constraint-norm', type=str, default='linf',
                        choices=['l2', 'linf'],
                        help='Constraint norm label in static LaTeX table '
                             '(default: linf). NOTE: violation checking '
                             'ALWAYS uses Linf regardless of this setting.')
    parser.add_argument('--goal-position', type=float, nargs=3,
                        default=[105.0, 0.0, 2.0], metavar=('X', 'Y', 'Z'),
                        help='Goal position for travel time recomputation '
                             '(default: 105.0 0.0 2.0)')
    parser.add_argument('--skip-metrics-recompute', action='store_true',
                        help='Skip recomputing travel time and path length '
                             'from rosbags')

    args = parser.parse_args()

    print("=" * 80)
    if args.table_format == 'static':
        print("EGO-SWARM V2 STATIC BENCHMARK ANALYSIS")
    else:
        print("EGO-SWARM V2 DYNAMIC BENCHMARK ANALYSIS")
    print("=" * 80)

    data_dir = Path(args.data_dir)
    if not data_dir.exists():
        print(f"ERROR: Directory not found: {data_dir}")
        sys.exit(1)

    # Load data
    df = load_benchmark_data(data_dir)
    print(f"  Trials: {len(df)}\n")

    # Recompute comp times from raw CSVs (fixes -10000 sentinel contamination)
    df = recompute_comp_times(data_dir, df)

    # Post-process collisions from rosbags
    if not args.skip_collision_postprocess:
        obstacle_csv = args.obstacle_csv if args.obstacle_csv else None
        df = postprocess_collisions(data_dir, df, obstacle_csv=obstacle_csv)

    # Recompute travel time and path length from rosbags
    if not args.skip_metrics_recompute:
        df = recompute_metrics_from_bags(
            data_dir, df,
            goal_position=tuple(args.goal_position))

    # Recompute constraint violations from rosbags (% of timesteps, always Linf)
    df = recompute_violations_from_bags(data_dir, df)

    # Compute statistics
    print("\nComputing statistics...")
    stats = compute_statistics(df)

    # Print results
    print_statistics(stats)

    # Save summary CSV
    summary_csv = data_dir / "benchmark_summary.csv"
    pd.DataFrame([stats]).to_csv(summary_csv, index=False)
    print(f"\nSummary saved: {summary_csv}")

    # Update LaTeX table
    if args.table_format == 'static':
        update_static_latex_table(
            stats, Path(args.dynus_table_path), args.algorithm_name,
            multirow_group=args.multirow_group or None,
            multirow_size=args.multirow_size,
            multirow_index=args.multirow_index,
            constraint_norm=args.constraint_norm)
    else:
        update_dynus_latex_table(
            stats, Path(args.dynus_table_path), args.algorithm_name,
            multirow_group=args.multirow_group or None,
            multirow_size=args.multirow_size,
            multirow_index=args.multirow_index,
            case_value=args.case_value or None)

    print("\n" + "=" * 80)
    print("ANALYSIS COMPLETE")
    print("=" * 80)
    print(f"\nGenerated files:")
    pp_csv = data_dir / "benchmark_ego_swarm_v2_postprocessed.csv"
    print(f"  1. {pp_csv} (if rosbags processed)")
    print(f"  2. {summary_csv}")
    print(f"  3. {args.dynus_table_path} (updated)")
    print()


if __name__ == '__main__':
    main()
