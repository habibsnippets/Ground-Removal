#!/usr/bin/env python3
"""
ROS Bag to PCD Converter and Visualizer
Converts point cloud data from .bag files to .pcd format and visualizes them
"""

import rosbag
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import os
import argparse
from pathlib import Path


def extract_pointcloud_from_bag(bag_file, topic_name, output_dir="pcd_output"):
    """
    Extract point cloud messages from a ROS bag file and save as PCD files
    
    Args:
        bag_file: Path to the .bag file
        topic_name: Name of the point cloud topic (e.g., '/camera/depth/points')
        output_dir: Directory to save PCD files
    
    Returns:
        List of paths to saved PCD files
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    pcd_files = []
    
    try:
        bag = rosbag.Bag(bag_file, 'r')
        
        # Get topic info
        info = bag.get_type_and_topic_info()
        topics = info.topics
        
        if topic_name not in topics:
            print(f"Topic '{topic_name}' not found in bag file.")
            print(f"Available topics: {list(topics.keys())}")
            bag.close()
            return pcd_files
        
        print(f"Processing topic: {topic_name}")
        print(f"Message count: {topics[topic_name].message_count}")
        
        # Read messages from the specified topic
        msg_count = 0
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # Convert ROS PointCloud2 message to numpy array
            points = []
            for p in pc2.read_points(msg, skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if len(points) == 0:
                print(f"Warning: Message {msg_count} has no valid points")
                continue
            
            points_array = np.array(points, dtype=np.float32)
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_array)
            
            # Save as PCD file
            output_file = os.path.join(output_dir, f"pointcloud_{msg_count:06d}.pcd")
            o3d.io.write_point_cloud(output_file, pcd)
            pcd_files.append(output_file)
            
            msg_count += 1
            if msg_count % 10 == 0:
                print(f"Processed {msg_count} messages...")
        
        bag.close()
        print(f"\nSuccessfully converted {msg_count} point clouds to PCD format")
        print(f"Files saved in: {output_dir}")
        
    except Exception as e:
        print(f"Error processing bag file: {e}")
    
    return pcd_files


def visualize_pcd(pcd_file):
    """
    Visualize a single PCD file using Open3D
    
    Args:
        pcd_file: Path to the PCD file
    """
    print(f"Loading point cloud: {pcd_file}")
    pcd = o3d.io.read_point_cloud(pcd_file)
    
    print(f"Point cloud has {len(pcd.points)} points")
    
    # Visualize
    print("Displaying point cloud (press Q to close)...")
    o3d.visualization.draw_geometries([pcd],
                                      window_name="Point Cloud Viewer",
                                      width=1024,
                                      height=768,
                                      point_show_normal=False)


def visualize_multiple_pcd(pcd_files):
    """
    Visualize multiple PCD files sequentially
    
    Args:
        pcd_files: List of paths to PCD files
    """
    if not pcd_files:
        print("No PCD files to visualize")
        return
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud Viewer", width=1024, height=768)
    
    for i, pcd_file in enumerate(pcd_files):
        print(f"\nShowing point cloud {i+1}/{len(pcd_files)}: {pcd_file}")
        pcd = o3d.io.read_point_cloud(pcd_file)
        print(f"Points: {len(pcd.points)}")
        
        vis.clear_geometries()
        vis.add_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        
        input("Press Enter for next point cloud (or Ctrl+C to exit)...")
    
    vis.destroy_window()


def list_topics(bag_file):
    """
    List all topics in a bag file
    
    Args:
        bag_file: Path to the .bag file
    """
    try:
        bag = rosbag.Bag(bag_file, 'r')
        info = bag.get_type_and_topic_info()
        
        print(f"\nTopics in {bag_file}:")
        print("-" * 80)
        for topic, topic_info in info.topics.items():
            print(f"Topic: {topic}")
            print(f"  Type: {topic_info.msg_type}")
            print(f"  Message Count: {topic_info.message_count}")
            print()
        
        bag.close()
    except Exception as e:
        print(f"Error reading bag file: {e}")


def main():
    parser = argparse.ArgumentParser(description="Convert ROS bag to PCD and visualize")
    parser.add_argument("bag_file", help="Path to the .bag file")
    parser.add_argument("-t", "--topic", help="Point cloud topic name", default=None)
    parser.add_argument("-o", "--output", help="Output directory for PCD files", default="pcd_output")
    parser.add_argument("-l", "--list", action="store_true", help="List topics in bag file")
    parser.add_argument("-v", "--visualize", action="store_true", help="Visualize converted point clouds")
    parser.add_argument("--single", help="Visualize a single PCD file", default=None)
    
    args = parser.parse_args()
    
    # List topics mode
    if args.list:
        list_topics(args.bag_file)
        return
    
    # Visualize single PCD file mode
    if args.single:
        visualize_pcd(args.single)
        return
    
    # Conversion mode
    if args.topic is None:
        print("Error: Please specify a topic name with -t or use -l to list topics")
        list_topics(args.bag_file)
        return
    
    # Convert bag to PCD
    pcd_files = extract_pointcloud_from_bag(args.bag_file, args.topic, args.output)
    
    # Visualize if requested
    if args.visualize and pcd_files:
        visualize_multiple_pcd(pcd_files)


if __name__ == "__main__":
    main()