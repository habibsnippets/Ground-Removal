import rospy
import rosbag
import pcl
import pcl.pcl_visualization
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

bag = rosbag.Bag("auto_main.orig.bag", "r")
pcd_filename = "output.pcd"

cloud_points = []  # define outside the loop

for topic, msg, t in bag.read_messages(topics=["/velodyne_points"]):
    print(f"Reading frame at time {t}")
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        cloud_points.append([p[0], p[1], p[2]])

bag.close()

if not cloud_points:
    print("❌ No points found! `.")
    exit(1)

cloud = pcl.PointCloud()
cloud.from_list(cloud_points)
pcl.save(cloud, pcd_filename)
print(f"✅ Saved {len(cloud_points)} points to {pcd_filename}")
