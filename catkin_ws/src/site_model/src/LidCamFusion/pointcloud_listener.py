#!/usr/bin/env python3

#############################################################
#   This py file subscribe the topic of lidars and output a #
#   list of them.                                           #
#############################################################

import rospy #  导入ROS Python客户端库
import std_msgs.msg #  导入标准消息库中的消息类型
import message_filters #  导入消息过滤器库，用于同步多主题的消息
from sensor_msgs.msg import PointCloud2 #  导入传感器消息库中的点云2消息类型
# self-defined msg
from msgs.msg._ListPointCloud import * #  从msgs模块的msg子模块中导入_ListPointCloud类

def pointcloud_listener(cloud11, cloud12, cloud2):
    """
    回调函数，用于处理接收到的点云数据，并将其集成到一个消息中。
    :param cloud11: 第一组点云数据
    :param cloud12: 第二组点云数据
    :param cloud2: 第三组点云数据
    """
    msgcloud = ListPointCloud()  # 创建点云列表消息对象
    msgcloud.pointcloud.append(cloud11)  # 将第一组点云添加到消息中
    msgcloud.pointcloud.append(cloud12)  # 将第二组点云添加到消息中
    msgcloud.pointcloud.append(cloud2)  # 将第三组点云添加到消息中

    # 添加时间戳
    msgcloud.header = std_msgs.msg.Header()  # 创建消息头
    msgcloud.header.stamp = rospy.Time.now()  # 设置当前时间为时间戳
    # 发布消息
    pub = rospy.Publisher("/pointcloud_list", ListPointCloud, queue_size=1)  # 创建发布者
    pub.publish(msgcloud)  # 发布集成后的点云列表消息

if __name__ == '__main__':
    rospy.init_node('lidar_listener', anonymous=True)  # 初始化ROS节点

    # 订阅各个激光雷达的点云数据
    sub_cloudpts_11 = message_filters.Subscriber('/velodyne1_points', PointCloud2)  # 第一台激光雷达
    sub_cloudpts_12 = message_filters.Subscriber('/velodyne2_points', PointCloud2)  # 第二台激光雷达
    sub_cloudpts_2 = message_filters.Subscriber('/velodyne3_points', PointCloud2)  # 第三台激光雷达

    # 创建时间同步器，确保多个订阅者的消息时间戳同步
    sync = message_filters.ApproximateTimeSynchronizer(
        [sub_cloudpts_11, sub_cloudpts_12, sub_cloudpts_2], 
        10, 1)  # 10个输入，1个输出
    sync.registerCallback(pointcloud_listener)  # 注册回调函数
    print("点云列表已启动。")  # 输出启动信息
    rospy.spin()  # 保持节点运行