#!/usr/bin/env python3  

import rospy
import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import LinkStates

def posecallback(msg):
    global position,orientation
    # My camera link pose was published at 9th index in the msg. To check yours, try rostopic echo /gazebo/link_states
    position = msg.pose[8].position
    orientation = msg.pose[8].orientation

if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster')
    camera_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, posecallback)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    position = None
    t.header.frame_id = "world"
    t.child_frame_id = "depth_camera"

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        if position != None:
            t.header.stamp = rospy.Time.now()
            t.transform.translation = position
            t.transform.rotation = orientation
            br.sendTransform(t)
            print("tf published")
            rate.sleep()