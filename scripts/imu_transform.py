#!/usr/bin/env python3
import rospy
import tf2_ros
import tf.transformations as tft
import numpy as np
from sensor_msgs.msg import Imu

def imu_callback(msg):
    # try:
    # trans = tf_buffer.lookup_transform("body", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    # q_tf = trans.transform.rotation
    # R_tf = tft.quaternion_matrix([q_tf.x, q_tf.y, q_tf.z, q_tf.w])[:3, :3]  # 회전행렬
    
    roll = 0.0       # rad
    pitch = 56*3.14/180.0      # rad
    yaw = 0.0       # rad  (예시)

    # euler_matrix 순서는 (roll, pitch, yaw)
    T = tft.euler_matrix(roll, pitch, yaw)   # 4x4 homogeneous matrix
    R_tf = T[:3, :3]                          # 회전 행렬(3x3)

    # # --- orientation ---
    # q_imu = msg.orientation
    # R_imu = tft.quaternion_matrix([q_imu.x, q_imu.y, q_imu.z, q_imu.w])[:3, :3]
    # R_body = R_tf @ R_imu
    # q_body = tft.quaternion_from_matrix(np.vstack((np.hstack((R_body, np.zeros((3,1)))), [0,0,0,1])))

    # --- angular velocity ---
    av = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    av_b = R_tf @ av

    # --- linear acceleration ---
    la = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    la_b = R_tf @ la

    imu_out = Imu()
    imu_out.header = msg.header
    imu_out.header.frame_id = "livox_frame"
    # imu_out.orientation.x, imu_out.orientation.y, imu_out.orientation.z, imu_out.orientation.w = q_imu
    imu_out.orientation = msg.orientation
    imu_out.angular_velocity.x, imu_out.angular_velocity.y, imu_out.angular_velocity.z = av_b
    imu_out.linear_acceleration.x, imu_out.linear_acceleration.y, imu_out.linear_acceleration.z = la_b

    pub.publish(imu_out)

    # except Exception as e:
    #     rospy.logwarn("TF 변환 실패: %s", e)

if __name__ == "__main__":
    rospy.init_node("imu_livox_to_body")

    # tf_buffer = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/livox/imu", Imu, imu_callback)
    pub = rospy.Publisher("/livox/imu/aligned", Imu, queue_size=10)

    rospy.spin()
