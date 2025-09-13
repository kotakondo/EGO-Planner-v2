#!/usr/bin/env python3
import os
import sys
import glob
import re
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Import custom PositionCommand message (not strictly required to read).
from quadrotor_msgs.msg import PositionCommand  # noqa: F401
from geometry_msgs.msg import PoseStamped       # noqa: F401


def compute_distance(p1, p2):
    """Compute Euclidean distance between two 3D points."""
    return np.linalg.norm(np.array(p1) - np.array(p2))


def extract_number(filename):
    """
    Extract numeric part from filename using regex.
    Assumes filename contains 'ego_swarm_v2_num_<number>'. Returns the integer number.
    """
    match = re.search(r'ego_swarm_v2_num_(\d+)', filename)
    if match:
        return int(match.group(1))
    else:
        return float('inf')  # Place files with no number at the end


def _trapz_safe(y, t):
    """
    Trapezoidal integral of y wrt t.
    - Sort by time if needed.
    - Trim to common length if lengths differ.
    - Drop NaN/Inf; return 0.0 if fewer than 2 valid samples.
    """
    y = np.asarray(y, dtype=float)
    t = np.asarray(t, dtype=float)

    n = min(y.size, t.size)
    if n < 2:
        return 0.0
    if (y.size != t.size) and n >= 2:
        y = y[:n]
        t = t[:n]

    order = np.argsort(t)
    t_sorted = t[order]
    y_sorted = y[order]

    mask = np.isfinite(t_sorted) & np.isfinite(y_sorted)
    if mask.sum() < 2:
        return 0.0

    return float(np.trapz(y_sorted[mask], t_sorted[mask]))


def _get_msg_jerk_norm(msg, fallback_acc=None, dt=None):
    """
    Prefer jerk from PositionCommand if available; otherwise finite-difference acceleration.
    Returns (jerk_norm, acc_vec_used).
    """
    # Try direct jerk field
    try:
        j = np.linalg.norm([msg.jerk.x, msg.jerk.y, msg.jerk.z])
        if np.isfinite(j):
            return float(j), np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], dtype=float)
    except Exception:
        pass

    # Fallback: FD on acceleration if we have previous acc and valid dt
    acc_vec = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], dtype=float)
    if fallback_acc is not None and dt is not None and dt > 0.0:
        j_vec = (acc_vec - fallback_acc) / dt
        return float(np.linalg.norm(j_vec)), acc_vec
    else:
        return 0.0, acc_vec


def process_bag(bag_file, tol=0.5, v_constraint=10.0, a_constraint=20.0, j_constraint=30.0):
    """
    Process a single bag file.

    Reads /move_base_simple/goal and /drone_0_planning/pos_cmd (EGO-swarm v2).
    Computes the same metrics/logic as SUPER:
      - travel_time: from first motion (> tol from start) until goal within tol
      - path_length: sum of consecutive segment lengths
      - smoothness: ∫ ||jerk|| dt via _trapz_safe
      - per-message violation counts with 1% slack
      - total_pos_cmds to compute percentages
    Returns None if a valid travel time cannot be computed.
    """
    bag = rosbag.Bag(bag_file)
    goal_time = None
    start_time = None
    goal_position = None
    travel_end_time = None

    pos_cmd_times = []
    velocities = []
    accelerations = []
    jerks = []
    positions = []

    vel_violations = 0
    acc_violations = 0
    jerk_violations = 0

    # 1% slack on constraints (match SUPER behavior)
    perct = 0.01
    v_thresh = v_constraint * (1.0 + perct)
    a_thresh = a_constraint * (1.0 + perct)
    j_thresh = j_constraint * (1.0 + perct)

    total_pos_cmds = 0
    prev_time = None
    prev_acc = None

    print(f"Processing bag: {bag_file}")
    for topic, msg, t in bag.read_messages(topics=["/move_base_simple/goal", "/drone_0_planning/pos_cmd"]):
        if topic == "/move_base_simple/goal" and goal_time is None:
            goal_time = t.to_sec()
            goal_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            print(f"  Found /move_base_simple/goal at time {goal_time:.3f}, goal_position = {goal_position}")

        elif topic == "/drone_0_planning/pos_cmd" and goal_time is not None:
            pos_time = t.to_sec()
            if pos_time < goal_time:
                continue

            pos_cmd_times.append(pos_time)

            # Position (for path length)
            pos = (msg.position.x, msg.position.y, msg.position.z)
            positions.append(pos)

            # Norms
            vel = np.linalg.norm([msg.velocity.x, msg.velocity.y, msg.velocity.z])
            acc_vec = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], dtype=float)
            acc = float(np.linalg.norm(acc_vec))

            dt = (pos_time - prev_time) if prev_time is not None else None
            jrk, acc_vec = _get_msg_jerk_norm(msg, fallback_acc=prev_acc, dt=dt)

            velocities.append(vel)
            accelerations.append(acc)
            jerks.append(jrk)

            if vel > v_thresh: vel_violations += 1
            if acc > a_thresh: acc_violations += 1
            if jrk > j_thresh: jerk_violations += 1

            total_pos_cmds += 1
            prev_time = pos_time
            prev_acc = acc_vec

            # Detect start of motion (> tol from initial position)
            if start_time is None:
                if compute_distance(pos, positions[0]) > tol:
                    start_time = pos_time
                    print(f"  Start of travel detected at time {start_time:.3f}")
                else:
                    continue

            # Goal reached?
            if compute_distance(pos, goal_position) <= tol:
                travel_end_time = pos_time
                print(f"  Travel time: {travel_end_time - start_time:.3f} s")
                break

    bag.close()

    if goal_time is None or travel_end_time is None or start_time is None:
        print("  Could not compute travel time for this bag.")
        return None

    travel_time = travel_end_time - start_time

    # Path length
    path_length = 0.0
    if len(positions) >= 2:
        for i in range(len(positions) - 1):
            path_length += compute_distance(positions[i], positions[i + 1])

    # Smoothness: ∫ ||jerk|| dt
    smoothness = _trapz_safe(jerks, pos_cmd_times)

    result = {
        "travel_time": float(travel_time),
        "path_length": float(path_length),
        "pos_cmd_times": np.array(pos_cmd_times, dtype=float),
        "velocities": np.array(velocities, dtype=float),
        "accelerations": np.array(accelerations, dtype=float),
        "jerks": np.array(jerks, dtype=float),
        "smoothness": float(smoothness),
        "vel_violations": int(vel_violations),
        "acc_violations": int(acc_violations),
        "jerk_violations": int(jerk_violations),
        "total_pos_cmds": int(total_pos_cmds),
    }
    return result


def save_plots(bag_file, results, v_constraint=10.0, a_constraint=20.0, j_constraint=30.0):
    """
    Save:
      1) Velocity histogram  -> <bag>_velocity_profile.pdf
      2) Time histories (v, a, j) -> <bag>_vel_accel_jerk.pdf
    (Same style as SUPER.)
    """
    base_name = os.path.splitext(os.path.basename(bag_file))[0]
    folder = os.path.dirname(bag_file)

    # Plot 1: Velocity histogram
    plt.figure()
    plt.hist(results["velocities"], bins=20, edgecolor="black")
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Frequency")
    plt.title("Velocity Profile Histogram")
    plt.axvline(x=v_constraint, color="red", linestyle="--", label=f"v constraint = {v_constraint} m/s")
    plt.legend()
    plt.grid(True)
    hist_path = os.path.join(folder, f"{base_name}_velocity_profile.pdf")
    plt.savefig(hist_path)
    plt.close()
    print("  Saved velocity histogram to:", hist_path)

    # Plot 2: Time histories
    t = results["pos_cmd_times"]
    v = results["velocities"]
    a = results["accelerations"]
    j = results["jerks"]

    plt.figure(figsize=(12, 10))
    plt.subplot(3, 1, 1)
    plt.plot(t, v, label="Velocity (m/s)")
    plt.axhline(y=v_constraint, color="red", linestyle="--", label=f"v constraint = {v_constraint} m/s")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend(); plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t, a, label="Acceleration (m/s²)")
    plt.axhline(y=a_constraint, color="red", linestyle="--", label=f"a constraint = {a_constraint} m/s²")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend(); plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t, j, label="Jerk (m/s³)")
    plt.axhline(y=j_constraint, color="red", linestyle="--", label=f"j constraint = {j_constraint} m/s³")
    plt.xlabel("Time (s)")
    plt.ylabel("Jerk (m/s³)")
    plt.legend(); plt.grid(True)

    plt.tight_layout()
    time_history_path = os.path.join(folder, f"{base_name}_vel_accel_jerk.pdf")
    plt.savefig(time_history_path)
    plt.close()
    print("  Saved vel/accel/jerk time history plot to:", time_history_path)


def main():
    if len(sys.argv) < 5:
        print(f"Usage: {sys.argv[0]} <bag_folder> <v_max> <a_max> <j_max>")
        print("Example: python3 analyze_bag.py /home/kota/data 10.0 20.0 30.0")
        sys.exit(1)

    bag_folder = sys.argv[1]
    v_constraint = float(sys.argv[2])
    a_constraint = float(sys.argv[3])
    j_constraint = float(sys.argv[4])

    tol = 0.5  # meters

    bag_files = glob.glob(os.path.join(bag_folder, "*.bag"))
    if not bag_files:
        print("No bag files found in folder:", bag_folder)
        sys.exit(1)

    # Sort by ego_swarm_v2_num_<N>
    bag_files = sorted(bag_files, key=lambda f: extract_number(os.path.basename(f)))

    overall_travel_times = []
    overall_path_lengths = []
    overall_smoothness = []
    overall_vel_violations = 0
    overall_acc_violations = 0
    overall_jerk_violations = 0
    overall_cmds = 0
    processed_count = 0

    stats_lines = []
    stats_lines.append("Bag File Statistics:\n\n")

    for bag_file in bag_files:
        result = process_bag(bag_file, tol, v_constraint, a_constraint, j_constraint)
        if result is None:
            stats_lines.append(f"{os.path.basename(bag_file)}: Could not compute travel time (missing /goal or not reached)\n\n")
            continue

        processed_count += 1
        overall_travel_times.append(result["travel_time"])
        overall_path_lengths.append(result["path_length"])
        overall_smoothness.append(result["smoothness"])
        overall_vel_violations += result["vel_violations"]
        overall_acc_violations += result["acc_violations"]
        overall_jerk_violations += result["jerk_violations"]
        overall_cmds += result["total_pos_cmds"]

        # Per-bag percentages
        v_pct = (result["vel_violations"] / result["total_pos_cmds"] * 100.0) if result["total_pos_cmds"] else 0.0
        a_pct = (result["acc_violations"] / result["total_pos_cmds"] * 100.0) if result["total_pos_cmds"] else 0.0
        j_pct = (result["jerk_violations"] / result["total_pos_cmds"] * 100.0) if result["total_pos_cmds"] else 0.0

        stats_lines.append(f"{os.path.basename(bag_file)}:\n")
        stats_lines.append(f"  Travel time: {result['travel_time']:.3f} s\n")
        stats_lines.append(f"  Path length: {result['path_length']:.3f} m\n")
        stats_lines.append(f"  Smoothness (∫||jerk|| dt): {result['smoothness']:.6f} m/s^2\n")
        stats_lines.append(f"  Velocity violations (>{v_constraint} m/s): {result['vel_violations']} ({v_pct:.2f}%)\n")
        stats_lines.append(f"  Acceleration violations (>{a_constraint} m/s²): {result['acc_violations']} ({a_pct:.2f}%)\n")
        stats_lines.append(f"  Jerk violations (>{j_constraint} m/s³): {result['jerk_violations']} ({j_pct:.2f}%)\n\n")

        # Plots per bag
        save_plots(bag_file, result, v_constraint, a_constraint, j_constraint)

    # Overall stats
    if processed_count > 0:
        avg_travel_time = float(np.mean(overall_travel_times))
        avg_path_length = float(np.mean(overall_path_lengths))
        avg_smoothness = float(np.mean(overall_smoothness)) if overall_smoothness else 0.0

        stats_lines.append("Overall Statistics:\n")
        stats_lines.append(f"  Processed bag files: {processed_count}\n")
        stats_lines.append(f"  Average travel time: {avg_travel_time:.3f} s\n")
        stats_lines.append(f"  Average path length: {avg_path_length:.3f} m\n")
        stats_lines.append(f"  Average smoothness (∫||jerk|| dt): {avg_smoothness:.6f} m/s^2\n")
        if overall_cmds > 0:
            stats_lines.append(f"  Velocity violations: {overall_vel_violations / overall_cmds * 100.0:.2f} %\n")
            stats_lines.append(f"  Acceleration violations: {overall_acc_violations / overall_cmds * 100.0:.2f} %\n")
            stats_lines.append(f"  Jerk violations: {overall_jerk_violations / overall_cmds * 100.0:.2f} %\n")
    else:
        stats_lines.append("No valid travel times computed from the bag files.\n")

    stats_file = os.path.join(bag_folder, "ego_swarm_v2_statistics.txt")
    with open(stats_file, "w") as f:
        f.writelines(stats_lines)
    print("Saved overall statistics to:", stats_file)


if __name__ == "__main__":
    main()
