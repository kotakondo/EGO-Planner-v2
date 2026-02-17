#!/usr/bin/env python3
"""
Benchmark runner for EGO-Swarm v2 in the dynamic obstacle environment.

Launches dynamic_forest.py (subprocess) + roslaunch single_drone_waypoints_dynamic.launch
+ rosbag record.  No Gazebo, no start_world.launch, no perfect_tracker.

Usage:
  python3 benchmarking_dynamic.py <total_sim_runs> <max_duration> <goal_x,goal_y,goal_z> \
      <goal_threshold> [--num-obstacles N] [--dynamic-ratio R] [--seed S]
"""
import argparse
import subprocess
import sys
import time
import math
import os

import rospy
import rosgraph
from quadrotor_msgs.msg import PositionCommand

# CSV file to log simulation outcomes.
CSV_PATH = "/home/kota/data_dynamic/goal_reached_status_ego_swarm_v2_dynamic.csv"

# Scripts directory inside the container.
SCRIPTS = "/home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker"


def run_command(cmd):
    """Launch a command via bash -c and return the Popen handle."""
    return subprocess.Popen(["bash", "-c", cmd])


class SimulationMonitor:
    """Monitor drone pose via /drone_0_planning/pos_cmd to detect goal arrival."""

    def __init__(self, goal, threshold):
        self.goal = goal
        self.threshold = threshold
        self.current_pose = None
        rospy.Subscriber("/drone_0_planning/pos_cmd", PositionCommand, self.pose_callback)

    def pose_callback(self, msg):
        self.current_pose = (msg.position.x, msg.position.y, msg.position.z)

    def reached_goal(self):
        if self.current_pose is None:
            return False
        dx = self.current_pose[0] - self.goal[0]
        dy = self.current_pose[1] - self.goal[1]
        dz = self.current_pose[2] - self.goal[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist <= self.threshold


def wait_for_ros_master(timeout=30):
    """Wait for the ROS master to become available, or exit after timeout."""
    start_time = time.time()
    master = rosgraph.Master("/benchmark_dynamic")
    while True:
        try:
            master.getUri()
            rospy.loginfo("ROS master is available!")
            return True
        except Exception:
            if time.time() - start_time > timeout:
                rospy.logerr("Timeout waiting for ROS master!")
                return False
            rospy.loginfo("Waiting for ROS master...")
            time.sleep(1)


def launch_simulation(sim_num, env_source, num_obstacles, dynamic_ratio, seed):
    """
    Launch simulation processes (excluding roscore) for the dynamic environment.
    No Gazebo, no start_world.launch, no perfect_tracker.
    """
    # Dynamic forest obstacle generator
    forest_cmd = (
        f"{env_source} && python3 {SCRIPTS}/dynamic_forest.py"
        f" --num-obstacles {num_obstacles}"
        f" --dynamic-ratio {dynamic_ratio}"
        f" --seed {seed}"
    )

    # EGO-Planner with dynamic launch file
    planner_cmd = (
        f"{env_source} && roslaunch --wait ego_planner"
        f" single_drone_waypoints_dynamic.launch simulation_number:={sim_num}"
    )

    # RViz
    rviz_cmd = f"{env_source} && roslaunch --wait ego_planner rviz.launch"

    # Rosbag record
    bag_file = f"/home/kota/data_dynamic/ego_swarm_v2_dynamic_num_{sim_num}.bag"
    rosbag_cmd = (
        f"{env_source} && rosbag record"
        " /tf /tf_static /rosout"
        " /drone_0_ego_planner_node/optimal_list"
        " /drone_0_odom_visualization/path"
        " /drone_0_ego_planner_node/grid_map/occupancy_inflate"
        " /drone_0_odom_visualization/robot"
        " /drone_0_planning/pos_cmd"
        " /move_base_simple/goal"
        " /dynamic_forest/markers"
        f" -O {bag_file}"
    )

    processes = []

    rospy.loginfo("Launching dynamic_forest.py ...")
    processes.append(run_command(forest_cmd))
    time.sleep(2)

    rospy.loginfo("Launching ego_planner rviz.launch ...")
    processes.append(run_command(rviz_cmd))
    time.sleep(2)

    rospy.loginfo("Launching single_drone_waypoints_dynamic.launch ...")
    processes.append(run_command(planner_cmd))
    time.sleep(2)

    rospy.loginfo(f"Launching rosbag recording to {bag_file} ...")
    processes.append(run_command(rosbag_cmd))
    time.sleep(2)

    return processes


def kill_processes(processes):
    for p in processes:
        p.terminate()
    for p in processes:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


def main():
    parser = argparse.ArgumentParser(description="EGO-Swarm v2 dynamic environment benchmark")
    parser.add_argument("total_sim_runs", type=int)
    parser.add_argument("max_duration", type=float)
    parser.add_argument("goal", type=str, help="goal_x,goal_y,goal_z")
    parser.add_argument("goal_threshold", type=float)
    parser.add_argument("--num-obstacles", type=int, default=200)
    parser.add_argument("--dynamic-ratio", type=float, default=0.65)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(rospy.myargv()[1:])

    goal_coords = tuple(map(float, args.goal.split(",")))

    env_source = (
        "source /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/"
        "swarm-playground/main_ws/devel/setup.bash"
        " && source /home/kota/mid360_ws/devel/setup.bash"
    )

    if not wait_for_ros_master():
        sys.exit(1)

    os.makedirs("/home/kota/data_dynamic", exist_ok=True)

    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w") as f:
            f.write("sim_num,status\n")

    rospy.init_node("benchmark_dynamic", anonymous=True)

    sim_num = 0
    while not rospy.is_shutdown() and sim_num < args.total_sim_runs:
        rospy.loginfo(f"\n==== Starting dynamic simulation {sim_num} ====")
        monitor = SimulationMonitor(goal=goal_coords, threshold=args.goal_threshold)
        sim_start_time = time.time()
        processes = launch_simulation(
            sim_num, env_source,
            args.num_obstacles, args.dynamic_ratio, args.seed,
        )
        goal_reached = False

        while time.time() - sim_start_time < args.max_duration and not rospy.is_shutdown():
            if monitor.reached_goal():
                rospy.loginfo("Goal reached!")
                goal_reached = True
                break
            time.sleep(0.1)

        travel_time = time.time() - sim_start_time
        status = "reached" if goal_reached else "timeout"
        rospy.loginfo(f"Simulation {sim_num} {status} in {travel_time:.1f}s")

        with open(CSV_PATH, "a") as csv_file:
            csv_file.write(f"{sim_num},{status}\n")

        kill_processes(processes)
        rospy.loginfo("Simulation processes terminated.")

        sim_num += 1
        rospy.loginfo("Restarting simulation in 5 seconds...")
        time.sleep(5)

    rospy.loginfo("All dynamic simulation runs complete.")


if __name__ == "__main__":
    main()
