#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint

def filterAxis(source, axis, min_val, max_val):
    inrange = []
    outrange = []
    
    for p in source:
        value = getattr(p, axis) # value of corresponding axis of points
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
    
    z_inrange, z_outrange = filterAxis(source, "z", -2.0, 2.0)
    
    x_inrange, x_outrange = filterAxis(z_inrange, "x", -0.5, 0.5)

    x_inrange2, x_outrange2 = filterAxis(x_outrange, "x", -6.12, 6.12)
    y_inrange, y_outrange = filterAxis(x_inrange2, "y", -6.12, 6.12)
    for p in y_inrange:
        filtered.append(p)
  
    y_inrange2, y_outrange2 = filterAxis(x_inrange, "y", -0.2, 0.2)
    y_inrange2, y_outrange2 = filterAxis(y_outrange2, "y", -6.12, 6.12)
    for p in y_inrange2:
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

    sub = rospy.Subscriber('/input', CustomMsg, callback, queue_size=10)
    pub = rospy.Publisher('/output', CustomMsg, queue_size=10)

    rospy.spin()

