#!/usr/bin/env python
# wangxiang
import cv2,cv_bridge,rospy
from sensor_msgs.msg import Image
from msgs.msg._MsgRadCam import MsgRadCam

def radar_camera_callback(msg):
    print(msg.num_overpass)
    pass

def get_radar_camera_msg():
    rospy.Subscriber("/radar_camera_fused", MsgRadCam, radar_camera_callback)
    pass

if __name__=='__main__':
    rospy.init_node('radar_camera_show')
    get_radar_camera_msg()
    rospy.spin()
    pass