#! /usr/bin/env python
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from math import *

index = 0

def set_roi_forward(h, w, mask):
    search_top = 400
    search_bot = search_top + 80
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    return mask
    pass

def follow_line(image):
    global index
    cmd_vel_pub = rospy.Publisher("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30,  255])
    mask = cv2.inRange(hsv, lower_white, upper_white)

    h, w = mask.shape
    mask = set_roi_forward(h, w, mask)
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])-0
        cy = int(M['m01'] / M['m00'])
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        err = cx - w / 2 - 20
        x = 0.1
        z = -float(err / 2.0) / 50

        akm = AckermannDriveStamped()
        akm.drive.speed = x*1.80
        if x==0.0:
            akm.drive.steering_angle = 0
        else:
            akm.drive.steering_angle = atan(z/x*0.133)
        cmd_vel_pub.publish(akm)

    # cv2.imwrite('/home/xiang/23_dachuang/RMMDet-master/src/site_model/src/scripts_wx/pictures/test_pictures_1/'+str(index)+'.png', image)
    # index += 1
    cv2.imshow('image', image)
    cv2.waitKey(1)
    pass

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    follow_line(frame)
    pass

if __name__ == '__main__':
    rospy.init_node("follow")
    rospy.Subscriber("/car1/car1/camera/zed_left/image_rect_color_left", Image, image_callback)
    rospy.spin()
    pass