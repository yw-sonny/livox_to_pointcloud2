#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np

accumulated_points = []
cnt = 0
pcd_file_dst = "/home/xavier/Downloads/GlobalMap.pcd"  # 저장 경로

def save_map():
    if accumulated_points:
        points_np = np.array(accumulated_points, dtype=np.float32)
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points_np)
        o3d.io.write_point_cloud(pcd_file_dst, cloud)
        rospy.loginfo(f"Map saved on shutdown: {pcd_file_dst}")
    else:
        rospy.logwarn("No points accumulated, nothing to save.")

def pointcloud_callback(msg):
    global accumulated_points, cnt

    try:
        transform = tf_buffer.lookup_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
        cloud_transformed_msg = tf2_sensor_msgs.do_transform_cloud(msg, transform)
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
        rospy.logwarn(f"TF transform failed: {e}")
        return

    cnt += 1
    # if cnt % 5 == 0:
    points = [p[:3] for p in pc2.read_points(cloud_transformed_msg, skip_nans=True)]
    accumulated_points.extend(points)
    rospy.loginfo(f"Accumulated points: {len(accumulated_points)}")

def main():
    global tf_buffer

    rospy.init_node('pointcloud_accumulator', anonymous=True)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.Subscriber("/cloud_registered_body", PointCloud2, pointcloud_callback)
    
    rospy.on_shutdown(save_map)
    
    rospy.spin()

if __name__ == "__main__":
    main()

