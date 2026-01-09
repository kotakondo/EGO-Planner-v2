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


# ----------------------------- Helpers -----------------------------
def compute_distance(p1, p2):
    """Compute Euclidean distance between two 3D points."""
    return np.linalg.norm(np.array(p1) - np.array(p2))


def extract_number(filename):
    """
    Extract numeric part from filename using regex.
    Assumes filename contains 'ego_swarm_v2_num_<number>'. Returns the integer number.
    """
    match = re.search(r'ego_swarm_v2_num_(\d+)', filename)
    return int(match.group(1)) if match else float('inf')


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


def _ensure_strictly_increasing(t):
    """Nudge non-increasing timestamps forward by a tiny epsilon (copy)."""
    t = np.asarray(t, dtype=float).copy()
    if t.size == 0:
        return t
    eps = 1e-9
    for k in range(1, t.size):
        if t[k] <= t[k - 1]:
            t[k] = t[k - 1] + eps
    return t


def _get_msg_jerk_vec(msg, fallback_acc=None, dt=None):
    """
    Return (jerk_vec, acc_vec). Prefer msg.jerk; fall back to finite-differenced acceleration.
    """
    acc_vec = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], float)

    # Try message jerk first
    try:
        j_vec = np.array([msg.jerk.x, msg.jerk.y, msg.jerk.z], float)
        if np.all(np.isfinite(j_vec)):
            return j_vec, acc_vec
    except Exception:
        pass

    # Fallback: finite difference on acceleration
    if fallback_acc is not None and dt is not None and dt > 0.0:
        j_vec = (acc_vec - fallback_acc) / dt
    else:
        j_vec = np.zeros(3, float)
    return j_vec, acc_vec


def compute_Jsmooth_and_Seff(times,
                             acc_x, acc_y, acc_z,
                             jerk_x=None, jerk_y=None, jerk_z=None):
    """
    Compute:
      - J_smooth = sqrt((1/T) * ∫ ||jerk||^2 dt)   [m/s^3]
      - S_eff    = sqrt((1/T) * ∫ ||snap||^2 dt)   [m/s^4],  snap = d(jerk)/dt
    Also returns 'snaps' = ||snap|| time series aligned with 'times'.

    Args:
      times:   timestamps [s], strictly increasing preferred
      acc_*:   acceleration components (x,y,z) [m/s^2]
      jerk_*:  optional jerk components; if None, jerk is FD from acc

    Returns:
      dict with {'J_smooth': float, 'S_eff': float, 'snaps': np.ndarray}
    """
    t  = np.asarray(times, float)
    ax = np.asarray(acc_x, float)
    ay = np.asarray(acc_y, float)
    az = np.asarray(acc_z, float)
    n = min(t.size, ax.size, ay.size, az.size)
    if n < 2:
        return {"J_smooth": 0.0, "S_eff": 0.0, "snaps": np.array([])}
    t, ax, ay, az = t[:n], ax[:n], ay[:n], az[:n]

    # Ensure strictly increasing time (avoid zero/negative dt)
    t_inc = _ensure_strictly_increasing(t)

    # Jerk components: provided or FD from acceleration
    have_j = (jerk_x is not None and jerk_y is not None and jerk_z is not None)
    if have_j:
        jx = np.asarray(jerk_x, float)[:t_inc.size]
        jy = np.asarray(jerk_y, float)[:t_inc.size]
        jz = np.asarray(jerk_z, float)[:t_inc.size]
    else:
        edge = 2 if t_inc.size >= 3 else 1
        jx = np.gradient(ax, t_inc, edge_order=edge)
        jy = np.gradient(ay, t_inc, edge_order=edge)
        jz = np.gradient(az, t_inc, edge_order=edge)

    # J_smooth (RMS jerk)
    j2 = jx*jx + jy*jy + jz*jz                  # ||j||^2
    J2 = _trapz_safe(j2, t_inc)                 # ∫ ||j||^2 dt
    T  = max(float(t_inc[-1] - t_inc[0]), 1e-12)
    J_smooth = float(np.sqrt(J2 / T))           # [m/s^3]

    # S_eff via snap = d/dt jerk (RMS snap)
    edge = 2 if t_inc.size >= 3 else 1
    sx = np.gradient(jx, t_inc, edge_order=edge)
    sy = np.gradient(jy, t_inc, edge_order=edge)
    sz = np.gradient(jz, t_inc, edge_order=edge)
    s2 = sx*sx + sy*sy + sz*sz                  # ||s||^2
    S2 = _trapz_safe(s2, t_inc)                 # ∫ ||s||^2 dt
    S_eff = float(np.sqrt(S2 / T))              # [m/s^4]
    snaps = np.sqrt(s2)                         # ||snap|| time series
    return {"J_smooth": J_smooth, "S_eff": S_eff, "snaps": snaps}


# ----------------------------- Core analysis -----------------------------
def process_bag(bag_file, tol=0.5, v_constraint=10.0, a_constraint=20.0, j_constraint=30.0):
    """
    Process a single bag file (EGO-swarm v2).

    Reads /drone_0_planning/pos_cmd.
    Computes:
      - travel_time: from first motion (> tol from start) until goal within tol
                     (goal fixed to (305, 0, 3) by default)
      - path_length: sum of segment lengths over the travel segment
      - smoothness:  ∫ ||jerk|| dt   [m/s^2] (legacy L1 jerk)
      - J_smooth:    RMS jerk         [m/s^3]
      - S_eff:       RMS snap         [m/s^4]
      - per-message violation counts with 1% slack
      - total_pos_cmds for percentage calculations
    Returns None if a valid travel time cannot be computed.
    """
    bag = rosbag.Bag(bag_file)
    start_time = None
    goal_position = (105.0, 0.0, 3.0)  # Default goal if not found in bag
    travel_end_time = None

    # Raw logs (full stream after bag start; we’ll window later)
    pos_cmd_times = []
    positions = []
    velocities = []
    accelerations = []
    jerks = []

    # For vector metrics
    acc_x, acc_y, acc_z = [], [], []
    jerk_x, jerk_y, jerk_z = [], [], []

    vel_violations = 0
    acc_violations = 0
    jerk_violations = 0

    # 1% slack on constraints
    perct = 0.01
    v_thresh = v_constraint * (1.0 + perct)
    a_thresh = a_constraint * (1.0 + perct)
    j_thresh = j_constraint * (1.0 + perct)

    total_pos_cmds = 0
    prev_time = None
    prev_acc = None

    print(f"Processing bag: {bag_file}")
    for topic, msg, t in bag.read_messages(topics=["/drone_0_planning/pos_cmd"]):
        pos_time = t.to_sec()
        pos = (msg.position.x, msg.position.y, msg.position.z)

        # Detect start of motion (> tol from initial position)
        start_tol = 0.05
        if start_time is None and len(positions) > 0:
            if compute_distance(pos, positions[0]) > start_tol:
                start_time = pos_time
                print(f"  Start of travel detected at time {start_time:.3f}")
            else:
                # still stationary near the first position; skip logging this one
                continue

        # Log time and position
        pos_cmd_times.append(pos_time)
        positions.append(pos)

        # Velocity & acceleration
        v_vec = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        vel = float(np.linalg.norm(v_vec))
        acc_vec = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], float)
        acc = float(np.linalg.norm(acc_vec))

        # Jerk: prefer message; fallback FD on acceleration
        dt = (pos_time - prev_time) if prev_time is not None else None
        j_vec, acc_vec = _get_msg_jerk_vec(msg, fallback_acc=prev_acc, dt=dt)
        jrk = float(np.linalg.norm(j_vec))

        velocities.append(vel)
        accelerations.append(acc)
        jerks.append(jrk)
        acc_x.append(acc_vec[0]); acc_y.append(acc_vec[1]); acc_z.append(acc_vec[2])
        jerk_x.append(j_vec[0]);  jerk_y.append(j_vec[1]);  jerk_z.append(j_vec[2])

        if vel > v_thresh: vel_violations += 1
        if acc > a_thresh: acc_violations += 1
        if jrk > j_thresh: jerk_violations += 1

        total_pos_cmds += 1
        prev_time = pos_time
        prev_acc  = acc_vec

        # Goal reached?
        if start_time is not None and compute_distance(pos, goal_position) <= tol:
            travel_end_time = pos_time
            print(f"  Travel time: {travel_end_time - start_time:.3f} s")
            break

    bag.close()

    if travel_end_time is None or start_time is None:
        print("  Could not compute travel time for this bag.")
        return None

    # ---------- Window to the travel segment ----------
    t_arr = np.asarray(pos_cmd_times, float)
    mask = (t_arr >= start_time) & (t_arr <= travel_end_time)
    idxs = np.nonzero(mask)[0]
    if idxs.size < 2:
        print("  Not enough samples in the travel window.")
        return None

    t_seg   = t_arr[idxs]
    v_seg   = np.asarray(velocities, float)[idxs]
    a_seg   = np.asarray(accelerations, float)[idxs]
    j_seg   = np.asarray(jerks, float)[idxs]
    ax_seg  = np.asarray(acc_x, float)[idxs]
    ay_seg  = np.asarray(acc_y, float)[idxs]
    az_seg  = np.asarray(acc_z, float)[idxs]
    jx_seg  = np.asarray(jerk_x, float)[idxs]
    jy_seg  = np.asarray(jerk_y, float)[idxs]
    jz_seg  = np.asarray(jerk_z, float)[idxs]
    pos_seg = [positions[i] for i in idxs]

    travel_time = float(t_seg[-1] - t_seg[0])

    # Path length on travel segment
    path_length = 0.0
    for i in range(len(pos_seg) - 1):
        path_length += compute_distance(pos_seg[i], pos_seg[i + 1])

    # Legacy L1 jerk (∫||j|| dt) on travel window
    smoothness = _trapz_safe(j_seg, t_seg)  # [m/s^2]

    # New metrics (RMS jerk & RMS snap) on travel window
    metrics = compute_Jsmooth_and_Seff(
        times=t_seg,
        acc_x=ax_seg, acc_y=ay_seg, acc_z=az_seg,
        jerk_x=jx_seg, jerk_y=jy_seg, jerk_z=jz_seg
    )
    J_smooth = metrics["J_smooth"]  # [m/s^3]
    S_eff    = metrics["S_eff"]     # [m/s^4]
    snaps    = metrics["snaps"]     # ||snap|| aligned with t_seg

    result = {
        "travel_time": travel_time,
        "path_length": float(path_length),
        "pos_cmd_times": t_seg,
        "velocities": v_seg,
        "accelerations": a_seg,
        "jerks": j_seg,
        "snaps": snaps,
        "smoothness": float(smoothness),  # ∫||j|| dt
        "J_smooth": float(J_smooth),      # RMS jerk
        "S_eff": float(S_eff),            # RMS snap (attitude effort)
        "vel_violations": int(vel_violations),
        "acc_violations": int(acc_violations),
        "jerk_violations": int(jerk_violations),
        "total_pos_cmds": int(total_pos_cmds),
        "start_time": float(start_time),
        "end_time": float(travel_end_time),
    }
    return result


def save_plots(bag_file, results, v_constraint=10.0, a_constraint=20.0, j_constraint=30.0, show_snap=True):
    """
    Save:
      1) Velocity histogram  -> <bag>_velocity_profile.pdf
      2) Time histories (v, a, j[, s]) -> <bag>_vel_accel_jerk[_snap].pdf
    """
    base_name = os.path.splitext(os.path.basename(bag_file))[0]
    folder = os.path.dirname(bag_file)

    # Plot 1: Velocity histogram
    plt.figure()
    plt.hist(results["velocities"], bins=20, edgecolor="black")
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Frequency")
    plt.title("Velocity Profile Histogram")
    plt.axvline(x=v_constraint, linestyle="--", label=f"v constraint = {v_constraint} m/s")
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
    s = results.get("snaps", None)

    if show_snap and s is not None and s.size == t.size:
        plt.figure(figsize=(12, 12))
        rows = 4
    else:
        plt.figure(figsize=(12, 10))
        rows = 3

    plt.subplot(rows, 1, 1)
    plt.plot(t, v, label="Velocity (m/s)")
    plt.axhline(y=v_constraint, linestyle="--", label=f"v constraint = {v_constraint} m/s")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend(); plt.grid(True)

    plt.subplot(rows, 1, 2)
    plt.plot(t, a, label="Acceleration (m/s²)")
    plt.axhline(y=a_constraint, linestyle="--", label=f"a constraint = {a_constraint} m/s²")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend(); plt.grid(True)

    plt.subplot(rows, 1, 3)
    plt.plot(t, j, label="Jerk (m/s³)")
    plt.axhline(y=j_constraint, linestyle="--", label=f"j constraint = {j_constraint} m/s³")
    plt.xlabel("Time (s)")
    plt.ylabel("Jerk (m/s³)")
    plt.legend(); plt.grid(True)

    if rows == 4:
        plt.subplot(rows, 1, 4)
        plt.plot(t, s, label="Snap (m/s⁴)")
        plt.xlabel("Time (s)")
        plt.ylabel("Snap (m/s⁴)")
        plt.legend(); plt.grid(True)

    plt.tight_layout()
    suffix = "_vel_accel_jerk_snap.pdf" if rows == 4 else "_vel_accel_jerk.pdf"
    time_history_path = os.path.join(folder, f"{base_name}{suffix}")
    plt.savefig(time_history_path)
    plt.close()
    print("  Saved time history plot to:", time_history_path)


# ----------------------------- Entry point -----------------------------
def main():
    if len(sys.argv) < 5:
        print(f"Usage: {sys.argv[0]} <bag_folder> <v_max> <a_max> <j_max>")
        print("Example: python3 analyze_bag_ego_v2.py /home/kota/data 10.0 20.0 30.0")
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
    overall_smoothness = []   # ∫||j|| dt (legacy L1 jerk)
    overall_J_smooth = []     # RMS jerk
    overall_S_eff = []        # RMS snap

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
            stats_lines.append(f"{os.path.basename(bag_file)}: Could not compute travel time (no start/finish)\n\n")
            continue

        processed_count += 1
        overall_travel_times.append(result["travel_time"])
        overall_path_lengths.append(result["path_length"])
        overall_smoothness.append(result["smoothness"])
        overall_J_smooth.append(result["J_smooth"])
        overall_S_eff.append(result["S_eff"])
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
        stats_lines.append(f"  J_smooth (RMS jerk): {result['J_smooth']:.6f} m/s^3\n")
        stats_lines.append(f"  S_eff (RMS snap): {result['S_eff']:.6f} m/s^4\n")
        stats_lines.append(f"  Velocity violations (>{v_constraint} m/s): {result['vel_violations']} ({v_pct:.2f}%)\n")
        stats_lines.append(f"  Acceleration violations (>{a_constraint} m/s²): {result['acc_violations']} ({a_pct:.2f}%)\n")
        stats_lines.append(f"  Jerk violations (>{j_constraint} m/s³): {result['jerk_violations']} ({j_pct:.2f}%)\n\n")

        # Plots per bag (set show_snap=True to include snap panel)
        save_plots(bag_file, result, v_constraint, a_constraint, j_constraint, show_snap=True)

    # Overall stats
    if processed_count > 0:
        avg_travel_time = float(np.mean(overall_travel_times))
        avg_path_length = float(np.mean(overall_path_lengths))
        avg_smoothness = float(np.mean(overall_smoothness)) if overall_smoothness else 0.0
        avg_J_smooth = float(np.mean(overall_J_smooth)) if overall_J_smooth else 0.0
        avg_S_eff = float(np.mean(overall_S_eff)) if overall_S_eff else 0.0

        stats_lines.append("Overall Statistics:\n")
        stats_lines.append(f"  Processed bag files: {processed_count}\n")
        stats_lines.append(f"  Average travel time: {avg_travel_time:.3f} s\n")
        stats_lines.append(f"  Average path length: {avg_path_length:.3f} m\n")
        stats_lines.append(f"  Average smoothness (∫||jerk|| dt): {avg_smoothness:.6f} m/s^2\n")
        stats_lines.append(f"  Average J_smooth (RMS jerk): {avg_J_smooth:.6f} m/s^3\n")
        stats_lines.append(f"  Average S_eff (RMS snap): {avg_S_eff:.6f} m/s^4\n")
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
