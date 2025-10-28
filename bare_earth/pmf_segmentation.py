#!/usr/bin/env python
import rospy
import rosbag
import numpy as np
import os
import subprocess
import tempfile
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def extract_point_cloud_from_bag(bag_path, topic_name, output_pcd):
    """Extract the first point cloud from a bag file and save as PCD"""
    try:
        bag = rosbag.Bag(bag_path)
        for _, msg, _ in bag.read_messages(topics=[topic_name]):
            # Convert PointCloud2 to points
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = list(points)
            
            # Write PCD file
            with open(output_pcd, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points_list)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points_list)}\n")
                f.write("DATA ascii\n")
                
                for point in points_list:
                    f.write(f"{point[0]} {point[1]} {point[2]}\n")
            
            bag.close()
            print(f"Extracted {len(points_list)} points from {bag_path}")
            return True
            
    except Exception as e:
        print(f"Error extracting point cloud: {e}")
        return False

def main():
    # Parameters
    bag_file = "auto.bag"  # Path to your bag file
    topic_name = "/velodyne_points"  # Point cloud topic
    
    # Output files
    input_pcd = "input_cloud.pcd"
    ground_pcd = "ground.pcd"
    objects_pcd = "objects.pcd"
    
    print(f"Processing bag file: {bag_file}")
    
    # Extract point cloud from bag
    if not extract_point_cloud_from_bag(bag_file, topic_name, input_pcd):
        print("Failed to extract point cloud from bag file")
        return
    
    print(f"Point cloud extracted to {input_pcd}")
    
    # Run the bare_earth executable
    try:
        cmd = ["./bare_earth", input_pcd, ground_pcd, objects_pcd]
        print(f"Running: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
        
        print(f"Ground points saved to {ground_pcd}")
        print(f"Object points saved to {objects_pcd}")
        
    except subprocess.CalledProcessError as e:
        print(f"Error running bare_earth: {e}")
    except FileNotFoundError:
        print("bare_earth executable not found. Make sure it's compiled and in the current directory.")

if __name__ == "__main__":
    main()