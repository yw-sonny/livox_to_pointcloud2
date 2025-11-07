#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import tf.transformations as tft

from livox_ros_driver.msg import CustomMsg, CustomPoint

def callback(msg):
    try:
        trans = tf_buffer.lookup_transform("body", "livox_frame", rospy.Time(0), rospy.Duration(1.0))

        t = trans.transform.translation
        q = trans.transform.rotation
        T = tft.concatenate_matrices(
            tft.translation_matrix((t.x, t.y, t.z)),
            tft.quaternion_matrix((q.x, q.y, q.z, q.w))
        )

        transformed_msg = CustomMsg()
        transformed_msg.header = msg.header
        transformed_msg.header.frame_id = "body"
        transformed_msg.timebase = msg.timebase
        transformed_msg.lidar_id = msg.lidar_id
        transformed_msg.point_num = msg.point_num

        for p in msg.points:
            vec = np.array([p.x, p.y, p.z, 1.0])
            new_xyz = T.dot(vec)[:3]

            new_p = CustomPoint()
            new_p.x, new_p.y, new_p.z = new_xyz
            new_p.reflectivity = p.reflectivity
            new_p.offset_time = p.offset_time
            new_p.tag = p.tag
            new_p.line = p.line

            transformed_msg.points.append(new_p)

        pub.publish(transformed_msg)

    except Exception as e:
        rospy.logwarn("Fail tf transform: %s", e)

if __name__ == "__main__":
    rospy.init_node("pointcloud_transform_node")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/livox/lidar", CustomMsg, callback)
    pub = rospy.Publisher("/livox/lidar/body", CustomMsg, queue_size=5)

    rospy.spin()
