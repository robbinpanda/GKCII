#!/usr/bin/env python
# wangxiang
import cv2,cv_bridge,rospy
from sensor_msgs.msg import Image

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('window', frame)
    cv2.waitKey(1)
    pass

def get_camera():
    rospy.Subscriber("/image_raw_7", Image, image_callback)
    pass

if __name__=='__main__':
    rospy.init_node('get_car_camera')
    get_camera()
    rospy.spin()
    pass