#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist,Pose,Point,Quaternion
from gazebo_msgs.srv import SpawnModel
import math
import os


def turtlebot_circle():
    rospy.init_node('square',anonymous=True)
    # rospy.wait_for_service("/gazebo/spawn_urdf_model")
    # spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    # spawn_model_client(
    #     model_name = 'turtlebot3_burger',
    #     model_xml = open('turtlebot3_burger.urdf', 'r').read(),
    #     robot_namespace = '/turtlebot',
    #     initial_pose = Pose(position= Point(3,0,0),orientation=Quaternion(0,0,0,0)),
    #     reference_frame = 'world'
    # )
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(50)    
    vel = Twist()
    lin_vel = 0.5
    ang_vel = 0.5
    count = 0
    while count<100:
            vel.linear.x = lin_vel
            vel.linear.y = 0.0
            vel.linear.z = 0.0

            vel.angular.x = 0.0
            vel.angular.y = 0.0 
            vel.angular.z = ang_vel
            rospy.loginfo("Radius = {}".format(lin_vel/ang_vel))
            pub.publish(vel)
            rate.sleep()
            count += 1
            
if __name__ == "__main__":
    try:
        turtlebot_circle()
        os.system("python tf_world_to_drone.py")
    except rospy.ROSInterruptException:
        pass

