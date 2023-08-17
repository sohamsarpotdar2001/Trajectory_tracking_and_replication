#!/usr/bin/env python3

import cv2 as cv
import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image,CameraInfo
from image_geometry import PinholeCameraModel
import geometry_msgs.msg
from ros_basics_tutorials.msg import samplemsg
from cv_bridge import CvBridge, CvBridgeError
import csv
import numpy as np
from math import *
# from nav_msgs.msg import Odometry

class image_capture:

    def __init__(self):
        rospy.init_node('centroid_tracker', anonymous=True)
        self.rate = rospy.Rate(10)
        self.command_pub = rospy.Publisher("command_topic",samplemsg,queue_size=10)
        self.bridge = CvBridge()
        self.cam_sub = rospy.Subscriber('/iris_downward_camera/camera/rgb/camera_info', CameraInfo, self.cam_callback)
        self.image_sub = rospy.Subscriber("/iris_downward_camera/camera/rgb/image_raw",Image,self.callback)
        self.object_detector = cv.createBackgroundSubtractorMOG2()
        self.point_on_image = None
        self.point3d = None

    def cam_callback(self,point_msg):
        cam_model = PinholeCameraModel()
        cam_model.fromCameraInfo(point_msg)
        if self.point_on_image:
            cam_model_point = cam_model.projectPixelTo3dRay(self.point_on_image)
            self.point3d = geometry_msgs.msg.PointStamped()
            self.point3d.point.x = cam_model_point[0]
            self.point3d.point.y = cam_model_point[1]
            self.point3d.point.z = cam_model_point[2]

    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
        self.zones()
        
        mask = self.object_detector.apply(self.cv_image)
        __, mask = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)
        _ , contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 100:
                # cv.drawContours(image, [cnt], -1, (0, 255, 0), 2)
                x_orig, y_orig, w, h = cv.boundingRect(cnt)
                cv.rectangle(self.cv_image, (x_orig, y_orig), (x_orig + w, y_orig + h), (0, 255, 0), 3)
                self.point_on_image = [x_orig+w/2,y_orig+h/2]
                self.point_on_image_rounded = [int(x_orig+w/2),int(y_orig+h/2)]
                if self.point3d:
                    self.convert_coordinates(self.point3d)
                    self.point = [self.world_point.point.x,self.world_point.point.y,self.world_point.point.z,sqrt(self.world_point.point.x**2 + self.world_point.point.y**2)]
                    if self.point_on_image_rounded[0] in self.centroid_track_area[0][:] and self.point_on_image_rounded[1] in self.centroid_track_area[1][:]:
                        print(self.point)
                        csvwriter.writerow(self.point)
                    self.repositioning()

        cv.imshow("image",self.cv_image)
        # cv.imshow("Mask",mask)
        
        key = cv.waitKey(1)

    def convert_coordinates(self,camera_point):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        tf_buffer.can_transform('world','depth_camera', rospy.Time(0), rospy.Duration(1.0))

        try:
            transform = tf_buffer.lookup_transform('world', 'depth_camera', rospy.Time(0), rospy.Duration(1.0))
            self.world_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
    
    def zones(self):
        self.red_up = [[*range(50,798+1)],[*range(0,50+1)]]
        self.red_down = [[*range(50,798+1)],[*range(430,480+1)]]
        self.red_left = [[*range(0,50+1)],[*range(0,480+1)]]
        self.red_right = [[*range(798,848+1)],[*range(0,480+1)]]
        self.centroid_track_area = [[*range(50,798+1)],[*range(50,430+1)]]

    def repositioning(self):
        commands = ["Forward","Left","Right","Backward"]
        if self.point_on_image[0] in self.red_up[0][:] and self.point_on_image[1] in self.red_up[1][:] :
            self.command_pub.publish(commands[0])
        if self.point_on_image[0] in self.red_down[0][:] and self.point_on_image[1] in self.red_down[1][:] :
            self.command_pub.publish(commands[3])
        if self.point_on_image[0] in self.red_left[0][:] and self.point_on_image[1] in self.red_left[1][:] :
            self.command_pub.publish(commands[1])
        if self.point_on_image[0] in self.red_right[0][:] and self.point_on_image[1] in self.red_right[1][:] :
            self.command_pub.publish(commands[2])
                

if __name__ == '__main__':
    with open("points.csv",mode="w",newline="") as csvfile:
        csvwriter = csv.writer(csvfile)
        header = ["x","y","z","dist"]
        csvwriter.writerow(header)
        ic = image_capture()
        rospy.spin()
        cv.destroyAllWindows()