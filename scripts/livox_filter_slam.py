#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint

def filterAxis(source, axis, min_val, max_val):
    inrange = []
    outrange = []
    
    for p in source:
        value = getattr(p, axis); # value of corresponding axis of points
        if(min_val < value < max_val):
            inrange.append(p)
        else:
            outrange.append(p) 
    
    return inrange, outrange

def callback(msg):
    filtered = []
    source = []
    for p in msg.points:
        source.append(p)
    
    x_inrange, x_outrange = filterAxis(source, "x", -0.5, 0.5)
    for p in x_outrange:
        filtered.append(p)
  
    y_inrange, y_outrange = filterAxis(x_inrange, "y", -0.5, 0.5)
    for p in y_outrange:
        filtered.append(p)
        
    filtered_msg = CustomMsg()
    filtered_msg.header = msg.header
    filtered_msg.timebase = msg.timebase
    filtered_msg.lidar_id = msg.lidar_id
    filtered_msg.point_num = len(filtered)
    filtered_msg.points = filtered

    pub.publish(filtered_msg)
    
if __name__ == '__main__':
    rospy.init_node('livox_filter_node', anonymous=True)
    rospy.loginfo("✅ Livox Filter Node Started")

    pub = rospy.Publisher('/livox/lidar/filtered', CustomMsg, queue_size=10)
    sub = rospy.Subscriber('/livox/lidar', CustomMsg, callback, queue_size=10)

    rospy.spin()

