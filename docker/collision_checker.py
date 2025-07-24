#!/usr/bin/env python
import rosbag
import csv
import glob
import os
import sys
import math
from geometry_msgs.msg import PoseStamped
from snapstack_msgs.msg import State

def load_forest_parameters(csv_path):
    """Load forest parameters from CSV and return a list of cylinder dictionaries."""
    cylinders = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cyl = {
                "id": row["id"],
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "radius": float(row["radius"]),
                "height": float(row["height"])
            }
            cylinders.append(cyl)
    return cylinders

def check_collision(agent_pos, cylinder, drone_radius=0.1):
    """
    Check if agent_pos (tuple of x,y,z) collides with the given cylinder.
    Cylinder is defined by (x, y, z, radius, height), where z is the cylinder's base (here spawned with z = height/2, so it spans z=0 to z=height).
    If a collision occurs, return a tuple (True, cylinder_id, penetration_distance), 
    where penetration_distance = (drone_radius + cylinder_radius) - horizontal_distance.
    Otherwise, return (False, None, None).
    """
    dx = agent_pos[0] - cylinder["x"]
    dy = agent_pos[1] - cylinder["y"]
    horizontal_dist = math.sqrt(dx*dx + dy*dy)
    allowed_dist = cylinder["radius"] + drone_radius
    if horizontal_dist <= allowed_dist:
        # Check vertical collision: agent's z must be within [0, cylinder["height"]]
        if 0 <= agent_pos[2] <= cylinder["height"]:
            penetration = allowed_dist - horizontal_dist
            return (True, cylinder["id"], penetration)
    return (False, None, None)

def process_bag(bag_path, cylinders, drone_radius, topic="/drone_0_planning/pos_cmd"):
    """
    Process a bag file: iterate over messages on the specified topic.
    For each message, check for collision with any cylinder.
    If a collision is found, return a tuple: (True, collided_cylinder, penetration).
    If no collision is detected in the bag, return (False, None, None).
    """
    try:
        bag = rosbag.Bag(bag_path)
    except Exception as e:
        print("Error opening bag {}: {}".format(bag_path, e))
        return (False, None, None)

    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        # Assume msg is of type State.
        agent_pos = (msg.position.x, msg.position.y, msg.position.z)
        for cyl in cylinders:
            collision, collided_id, penetration = check_collision(agent_pos, cyl, drone_radius)
            if collision:
                bag.close()
                return (True, collided_id, penetration)
    bag.close()
    return (False, None, None)

def main():
    if len(sys.argv) < 5:
        print("Usage: {} <forest_csv_path> <bag_folder> <drone_radius> <pos_topic_name>".format(sys.argv[0]))
        print("Example: python3 collision_checker.py /home/kota/data/easy_forest_obstacle_parameters.csv /home/kota/data 0.1 /drone_0_planning/pos_cmd")
        sys.exit(1)
    forest_csv = sys.argv[1]
    bag_folder = sys.argv[2]
    drone_radius = float(sys.argv[3])
    topic = sys.argv[4]  

    cylinders = load_forest_parameters(forest_csv)
    bag_files = glob.glob(os.path.join(bag_folder, "*.bag"))
    bag_files = sorted(bag_files)

    results = []
    for bag_file in bag_files:
        print("Processing bag: {}".format(bag_file))
        collision, collided_obstacle, penetration = process_bag(bag_file, cylinders, drone_radius, topic)
        if collision:
            status = "collision"
        else:
            status = "no collision"
            collided_obstacle = ""
            penetration = ""
        results.append((os.path.basename(bag_file), status, collided_obstacle, penetration))

    # Write results to collision_check.csv.
    output_csv = "collision_check.csv"
    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["bag_file", "collision_status", "collided_obstacle", "penetration"])
        for bag_file, status, collided, pen in results:
            writer.writerow([bag_file, status, collided, pen])

    print("Collision check complete. Results written to {}".format(output_csv))

if __name__ == "__main__":
    main()
