#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2021 wechange tech.
# Developer: FuZhi Liu 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from dynamic_reconfigure.server import Server
from robot_vision.cfg import line_hsvConfig
# from robot_vision.cfg

from geometry_msgs.msg import Twist

class line_follow:
    def __init__(self):    
        #define topic publisher and subscriber
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        self.mask_pub = rospy.Publisher("/mask_image", Image, queue_size=1)
        self.result_pub = rospy.Publisher("/result_image", Image, queue_size=1)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.srv = Server(line_hsvConfig,self.dynamic_reconfigure_callback)
        # get param from launch file 
        self.test_mode = bool(rospy.get_param('~test_mode',False))
        self.h_lower = int(rospy.get_param('~h_lower',110))
        self.s_lower = int(rospy.get_param('~s_lower',50))
        self.v_lower = int(rospy.get_param('~v_lower',50))

        self.h_upper = int(rospy.get_param('~h_upper',130))
        self.s_upper = int(rospy.get_param('~s_upper',255))
        self.v_upper = int(rospy.get_param('~v_upper',255))
        self.scan_offsets = [144, 114, 84, 54, 24, -6]
        #line center point X Axis coordinate
        self.center_point = 0

    def dynamic_reconfigure_callback(self,config,level):
        # update config param
        self.h_lower = config.h_lower
        self.s_lower = config.s_lower
        self.v_lower = config.v_lower
        self.h_upper = config.h_upper
        self.s_upper = config.s_upper
        self.v_upper = config.v_upper
        return config

    def callback(self,data):
        # convert ROS topic to CV image formart
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
        # cv_bridge with bgr8 returns BGR image, so convert from BGR to HSV.
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        #set color mask min amd max value
        line_lower = np.array([self.h_lower,self.s_lower,self.v_lower])
        line_upper = np.array([self.h_upper,self.s_upper,self.v_upper])
        # get mask from color
        mask = cv2.inRange(hsv_image,line_lower,line_upper)
        # close operation to fit some little hole without over-widening the seam
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
        # if test mode,output the center point HSV value
        res = cv_image
        if self.test_mode:
            cv2.circle(res, (hsv_image.shape[1]/2,hsv_image.shape[0]/2), 5, (0,0,255), 1)
            cv2.line(res,(hsv_image.shape[1]/2-10, hsv_image.shape[0]/2), (hsv_image.shape[1]/2+10,hsv_image.shape[0]/2), (0,0,255), 1)
            cv2.line(res,(hsv_image.shape[1]/2, hsv_image.shape[0]/2-10), (hsv_image.shape[1]/2, hsv_image.shape[0]/2+10), (0,0,255), 1)
            rospy.loginfo("Point HSV Value is %s"%hsv_image[hsv_image.shape[0]/2,hsv_image.shape[1]/2])            
        else:
            image_center_y = hsv_image.shape[0] / 2
            for offset in self.scan_offsets:
                row_index = max(0, min(mask.shape[0] - 1, image_center_y + offset))
                row_pixels = np.nonzero(mask[row_index])[0]
                if len(row_pixels) > 6:
                    self.center_point = int(np.mean(row_pixels))
                    cv2.circle(res, (self.center_point, row_index), 5, (0,0,255), 5)
                    break
        if self.center_point:
            self.twist_calculate(hsv_image.shape[1]/2,self.center_point)
        else:
            self.stop_robot()
        self.center_point = 0


        # show CV image in debug mode(need display device)
        # cv2.imshow("Image window", res)
        # cv2.imshow("Mask window", mask)
        # cv2.waitKey(3)

        # convert CV image to ROS topic and pub 
        try:
            img_msg = self.bridge.cv2_to_imgmsg(res, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.result_pub.publish(img_msg)
            img_msg = self.bridge.cv2_to_imgmsg(mask, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.mask_pub.publish(img_msg)
            
        except CvBridgeError as e:
            print e

    def twist_calculate(self,width,center):
        center = float(center)
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        error = (width - center) / width
        steer_error = error
        if abs(error) > 0.03:
            steer_error += 0.010 * np.sign(error)
        self.twist.angular.z = max(min(steer_error * 0.93, 0.45), -0.45)
        if abs(error) < 0.08:
            self.twist.linear.x = 0.12
        elif abs(error) < 0.18:
            self.twist.linear.x = 0.08
        else:
            self.twist.linear.x = 0.042
        self.pub_cmd.publish(self.twist)

    def stop_robot(self):
        self.pub_cmd.publish(Twist())



if __name__ == '__main__':
    try:
        # init ROS node 
        rospy.init_node("line_follow")
        rospy.loginfo("Starting Line Follow node")
        line_follow()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
