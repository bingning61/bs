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
        self.scan_row_ratios = [0.82, 0.74, 0.66, 0.58, 0.50, 0.42]
        self.scan_row_weights = [0.30, 0.24, 0.18, 0.14, 0.09, 0.05]
        self.min_track_pixels = int(rospy.get_param('~min_track_pixels', 8))
        self.min_track_area = float(rospy.get_param('~min_track_area', 180.0))
        self.turn_gain = float(rospy.get_param('~turn_gain', 0.90))
        self.lookahead_gain = float(rospy.get_param('~lookahead_gain', 0.55))
        self.max_angular_speed = float(rospy.get_param('~max_angular_speed', 0.58))
        self.straight_speed = float(rospy.get_param('~straight_speed', 0.115))
        self.curve_speed = float(rospy.get_param('~curve_speed', 0.080))
        self.sharp_curve_speed = float(rospy.get_param('~sharp_curve_speed', 0.060))
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
        # remove isolated noise first, then reconnect small gaps in the seam.
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        # if test mode,output the center point HSV value
        res = cv_image.copy()
        display_mask = mask
        line_found = False
        if self.test_mode:
            cv2.circle(res, (hsv_image.shape[1]/2,hsv_image.shape[0]/2), 5, (0,0,255), 1)
            cv2.line(res,(hsv_image.shape[1]/2-10, hsv_image.shape[0]/2), (hsv_image.shape[1]/2+10,hsv_image.shape[0]/2), (0,0,255), 1)
            cv2.line(res,(hsv_image.shape[1]/2, hsv_image.shape[0]/2-10), (hsv_image.shape[1]/2, hsv_image.shape[0]/2+10), (0,0,255), 1)
            rospy.loginfo("Point HSV Value is %s"%hsv_image[hsv_image.shape[0]/2,hsv_image.shape[1]/2])            
        else:
            display_mask, tracking_state = self.extract_track_state(mask, res)
            if tracking_state is not None:
                line_found = True
                self.center_point = tracking_state["weighted_center"]
                self.twist_calculate(
                    hsv_image.shape[1] / 2,
                    tracking_state["weighted_center"],
                    tracking_state["near_center"],
                    tracking_state["far_center"])
        if line_found:
            pass
        elif not self.test_mode:
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
            img_msg = self.bridge.cv2_to_imgmsg(display_mask, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.mask_pub.publish(img_msg)
            
        except CvBridgeError as e:
            print e

    def extract_track_state(self, mask, res):
        contours_info = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_info) == 3:
            _, contours, _ = contours_info
        else:
            contours, _ = contours_info
        if not contours:
            return mask, None

        main_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(main_contour) < self.min_track_area:
            return mask, None

        track_mask = np.zeros_like(mask)
        cv2.drawContours(track_mask, [main_contour], -1, 255, -1)
        cv2.drawContours(res, [main_contour], -1, (255, 180, 0), 2)

        sampled_centers = []
        for index, row_ratio in enumerate(self.scan_row_ratios):
            row_index = max(0, min(track_mask.shape[0] - 1, int(track_mask.shape[0] * row_ratio)))
            row_pixels = np.nonzero(track_mask[row_index])[0]
            if len(row_pixels) < self.min_track_pixels:
                continue

            left_edge = int(row_pixels[0])
            right_edge = int(row_pixels[-1])
            center = int((left_edge + right_edge) / 2.0)
            sampled_centers.append({
                "center": center,
                "row": row_index,
                "weight": self.scan_row_weights[index]
            })

            cv2.circle(res, (center, row_index), 4, (0, 0, 255), -1)
            cv2.line(res, (left_edge, row_index), (right_edge, row_index), (0, 255, 255), 1)

        if not sampled_centers:
            return track_mask, None

        total_weight = sum([point["weight"] for point in sampled_centers])
        if total_weight == 0:
            return track_mask, None

        weighted_center = int(sum([point["center"] * point["weight"] for point in sampled_centers]) / total_weight)
        near_center = sampled_centers[0]["center"]
        far_center = sampled_centers[-1]["center"]
        cv2.line(res, (weighted_center, sampled_centers[0]["row"]), (weighted_center, sampled_centers[-1]["row"]), (0, 255, 0), 2)
        return track_mask, {
            "weighted_center": weighted_center,
            "near_center": near_center,
            "far_center": far_center
        }

    def twist_calculate(self,image_center,center,near_center,far_center):
        center = float(center)
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        image_center = float(image_center)
        lateral_error = (image_center - center) / image_center
        lookahead_error = (float(near_center) - float(far_center)) / image_center
        turn_error = lateral_error * self.turn_gain + lookahead_error * self.lookahead_gain
        self.twist.angular.z = max(min(turn_error, self.max_angular_speed), -self.max_angular_speed)

        if abs(turn_error) < 0.10:
            self.twist.linear.x = self.straight_speed
        elif abs(turn_error) < 0.24:
            self.twist.linear.x = self.curve_speed
        else:
            self.twist.linear.x = self.sharp_curve_speed
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
