#!/usr/bin/env python3

#############################################################
#   This py file subscribe the topic of cameras and publish #
#   the integrated information.                             #
#############################################################

# 导入ROS Python客户端库
import rospy
# 导入ROS图像消息类型
from sensor_msgs.msg import Image
# 导入自定义的相机消息类型
from msgs.msg._MsgCamera import *  # camera msgs class
# 导入标准ROS消息头类型
import std_msgs.msg
# 导入NumPy库，用于数值计算
import numpy as np
# 导入消息过滤器库，用于时间同步
import message_filters

# 定义相机监听回调函数，接收8个图像消息
def camera_listener(image11, image12, image13, image14, image41, image42, image43, image44):
    """
    回调函数，用于处理接收到的摄像头图像，并将其集成到一个消息中。
    :param image11: 第一组相机图像
    :param image12: 第二组相机图像
    :param image13: 第三组相机图像
    :param image14: 第四组相机图像
    :param image41: 第五组相机图像
    :param image42: 第六组相机图像
    :param image43: 第七组相机图像
    :param image44: 第八组相机图像
    """
    # 创建消息对象
    msgcamera = MsgCamera()
    mark = [image11, image12, image13, image14, image41, image42, image43, image44]
    for i in mark:
        msgcamera.camera.append(i)  # 将每个图像添加到消息中

    # 添加时间戳
    msgcamera.header = std_msgs.msg.Header()  # 创建消息头
    msgcamera.header.stamp = rospy.Time.now()  # 设置当前时间为时间戳
    # 发布消息
    pub = rospy.Publisher("/camera_msgs_combined", MsgCamera, queue_size=1)  # 创建发布者
    pub.publish(msgcamera)  # 发布集成后的相机消息

if __name__ == '__main__':
    rospy.init_node('camera_listener', anonymous=True)  # 初始化ROS节点

    # 订阅各个摄像头的原始图像消息
    sub_image_11 = message_filters.Subscriber('/image_raw_11', Image)
    sub_image_12 = message_filters.Subscriber('/image_raw_12', Image)
    sub_image_13 = message_filters.Subscriber('/image_raw_13', Image)
    sub_image_14 = message_filters.Subscriber('/image_raw_14', Image)
    sub_image_41 = message_filters.Subscriber('/image_raw_41', Image)
    sub_image_42 = message_filters.Subscriber('/image_raw_42', Image)
    sub_image_43 = message_filters.Subscriber('/image_raw_43', Image)
    sub_image_44 = message_filters.Subscriber('/image_raw_44', Image)

    # 创建时间同步器，确保多个订阅者的消息时间戳同步
    sync = message_filters.ApproximateTimeSynchronizer(
        [sub_image_11, sub_image_12, sub_image_13, sub_image_14, 
         sub_image_41, sub_image_42, sub_image_43, sub_image_44], 
        10, 1)  # 10个输入，1个输出
    sync.registerCallback(camera_listener)  # 注册回调函数
    print("摄像头监听器已启动。")  # 输出启动信息
    rospy.spin()  # 保持节点运行