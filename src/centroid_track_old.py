#!/usr/bin/env python3

import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from ros_basics_tutorials.msg import samplemsg
from cv_bridge import CvBridge, CvBridgeError
import csv
import numpy as np
from math import *

class image_capture:

    def __init__(self):
        self.command_pub = rospy.Publisher("command_topic",samplemsg,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris_downward_camera/camera/rgb/image_raw",Image,self.callback)
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.posecallback)
        self.object_detector = cv.createBackgroundSubtractorMOG2()

    def posecallback(self,msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

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
                x_c , y_c = [x_orig+w/2,y_orig+h/2]
                r = sqrt((848-x_c)**2+(480-y_c)**2)
                theta = atan((480-y_c)/(848-x_c))
                x = sqrt(((r/150)**2) * (cos(theta)**2)) - 1.55
                y = (x+1.55)*tan(theta) + 0.469
                dist = sqrt(x**2+y**2)
                self.point = [x,y,dist]
                self.point_on_image = [x_c,y_c]
                if self.point_on_image[0] in self.centroid_track_area[0][:] and self.point_on_image[1] in self.centroid_track_area[1][:]:
                    print(self.point)
                    csvwriter.writerow(self.point)
                self.repositioning()

        cv.imshow("image",self.cv_image)
        # cv.imshow("Mask",mask)
        
        key = cv.waitKey(1)

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
                

with open("points.csv",mode="w",newline="") as csvfile:
    csvwriter = csv.writer(csvfile)
    header = ["x","y","dist"]
    csvwriter.writerow(header)
    ic = image_capture()
    rospy.init_node('centroid_tracker', anonymous=True)
    rospy.Rate(10)
    rospy.spin()
    cv.destroyAllWindows()





