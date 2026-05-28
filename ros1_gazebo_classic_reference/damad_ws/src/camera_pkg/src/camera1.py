#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import message_filters
import numpy as np

def callback(msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7):
    image0 = CvBridge().compressed_imgmsg_to_cv2(msg0, "bgr8")
    image1 = CvBridge().compressed_imgmsg_to_cv2(msg1, "bgr8")
    image2 = CvBridge().compressed_imgmsg_to_cv2(msg2, "bgr8")
    image3 = CvBridge().compressed_imgmsg_to_cv2(msg3, "bgr8")
    image4 = CvBridge().compressed_imgmsg_to_cv2(msg4, "bgr8")
    image5 = CvBridge().compressed_imgmsg_to_cv2(msg5, "bgr8")
    image6 = CvBridge().compressed_imgmsg_to_cv2(msg6, "bgr8")
    image7 = CvBridge().compressed_imgmsg_to_cv2(msg7, "bgr8")
    iris = np.concatenate((image0, image1), axis=1)
    iris = cv2.resize(iris,(960,240))
    rover0 = np.concatenate((image2, image3, image4), axis=1)
    rover1 = np.concatenate((image5, image6, image7), axis=1)
    #print(iris.shape, rover0.shape, rover1.shape)
    result = np.concatenate((iris, rover0, rover1), axis=0)
    cv2.imshow("image", result)
    cv2.waitKey(1)
    
def main():
    rospy.init_node('camera', anonymous=True)
    """iris0_sub = rospy.Subscriber("/iris0/fpv_cam/image_raw", Image, iris0_callback)
    iris1_sub = rospy.Subscriber("/iris1/fpv_cam/image_raw", Image, iris1_callback)
    rover0_sub0 = rospy.Subscriber("/rover0/fpv_cam0/image_raw", Image, rover0_callback0)
    rover0_sub1 = rospy.Subscriber("/rover0/fpv_cam1/image_raw", Image, rover0_callback1)
    rover0_sub2 = rospy.Subscriber("/rover0/fpv_cam2/image_raw", Image, rover0_callback2)
    rover1_sub0 = rospy.Subscriber("/rover1/fpv_cam0/image_raw", Image, rover1_callback0)
    rover1_sub1 = rospy.Subscriber("/rover1/fpv_cam1/image_raw", Image, rover1_callback1)
    rover1_sub2 = rospy.Subscriber("/rover1/fpv_cam2/image_raw", Image, rover1_callback2)"""
    iris0_sub = message_filters.Subscriber("/iris0/fpv_cam/image_raw/compressed", CompressedImage)
    iris1_sub = message_filters.Subscriber("/iris1/fpv_cam/image_raw/compressed", CompressedImage)
    rover0_sub0 = message_filters.Subscriber("/rover0/fpv_cam0/image_raw/compressed", CompressedImage)
    rover0_sub1 = message_filters.Subscriber("/rover0/fpv_cam1/image_raw/compressed", CompressedImage)
    rover0_sub2 = message_filters.Subscriber("/rover0/fpv_cam2/image_raw/compressed", CompressedImage)
    rover1_sub0 = message_filters.Subscriber("/rover1/fpv_cam0/image_raw/compressed", CompressedImage)
    rover1_sub1 = message_filters.Subscriber("/rover1/fpv_cam1/image_raw/compressed", CompressedImage)
    rover1_sub2 = message_filters.Subscriber("/rover1/fpv_cam2/image_raw/compressed", CompressedImage)
    ts = message_filters.ApproximateTimeSynchronizer([iris0_sub, iris1_sub, rover0_sub0, rover0_sub1, rover0_sub2, rover1_sub0, rover1_sub1, rover1_sub2], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == "__main__":
    main()
