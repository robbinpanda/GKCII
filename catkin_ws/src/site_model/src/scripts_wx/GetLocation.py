#! /usr/bin/env python
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from math import *
from gazebo_msgs.msg import ModelStates
import message_filters

'''
将摄像头画面和小车位置匹配起来
'''

model_name = 'car1'
model_index = None

def get_model_index(msg):
    global model_name
    for i,name in enumerate(msg.name):
        if model_name==name:
            return i

def callback(image_msg, model_states_msg):
    global model_index

    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(image_msg, 'bgr8')

    if not model_index:
        model_index = get_model_index(model_states_msg)
    model_pos = model_states_msg.pose[model_index].position

    x = round(model_pos.x,2)
    y = round(model_pos.y,2)

    cv2.putText(frame, 'x: '+str(x)+' y: '+str(y), (20,20), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,0,255))
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
    pass

if __name__ == '__main__':
    rospy.init_node("follow")
    m1 = message_filters.Subscriber("/car1/car1/camera/zed_left/image_rect_color_left", Image)
    m2 = message_filters.Subscriber("/gazebo/model_states", ModelStates)
    ts = message_filters.ApproximateTimeSynchronizer([m1,m2], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
    pass