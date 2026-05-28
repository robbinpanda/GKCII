#!/usr/bin/env python3
"""
订阅原始雷达主题并发布过滤后的消息。
"""

import rospy  # 导入ROS Python客户端库
import message_filters  # 导入消息过滤器库，用于时间同步订阅
from per_msgs.msg._SensorMsgsRadar import SensorMsgsRadar  # 导入雷达消息类型
from per_msgs.msg._GeometryMsgsRadarObject import GeometryMsgsRadarObject  # 导入雷达对象几何消息类型
from msgs.msg._MsgRadar import MsgRadar  # 导入自定义雷达消息类型
from msgs.msg._MsgRadarObject import MsgRadarObject  # 导入自定义雷达对象消息类型


def append_obj(t: GeometryMsgsRadarObject, obj_list: list):
    """
    将雷达对象从GeometryMsgsRadarObject类型转换为MsgRadarObject类型并添加到列表中。
    :param t: 输入的雷达对象，类型为GeometryMsgsRadarObject。
    :param obj_list: 存储转换后的MsgRadarObject对象的列表。
    """
    obj = MsgRadarObject()  # 创建一个新的雷达对象
    obj.distance = t.range  # 设置距离
    obj.velocity = t.range_rate  # 设置速度
    obj.angle_centroid = t.angle_centroid  # 设置质心角度
    obj.pos_x = t.obj_vcs_posex  # 设置X坐标
    obj.pos_y = t.obj_vcs_posey  # 设置Y坐标
    obj.track_id = t.track_id  # 设置跟踪ID
    obj_list.append(obj)  # 将对象添加到列表中


def radar_listener(radar2: SensorMsgsRadar, radar3: SensorMsgsRadar):
    print("radar_listener is working")
    """
    处理来自两个雷达的消息，并将其过滤后发布。
    :param radar2: 来自第一个雷达的消息，类型为SensorMsgsRadar。
    :param radar3: 来自第二个雷达的消息，类型为SensorMsgsRadar。
    """
    global pub

    msg_radar = MsgRadar()  # 创建一个新的自定义雷达消息
    # 处理左侧雷达数据
    msg_radar.num_left = int(radar2.total_front_left_esr_tracks)  # 左侧雷达检测到的对象数量
    for t in radar2.front_left_esr_tracklist:  # 遍历左侧雷达对象列表
        append_obj(t, msg_radar.objects_left)  # 将对象添加到msg_radar对象列表中

    # 处理右侧雷达数据
    msg_radar.num_right = int(radar3.total_front_right_esr_tracks)  # 右侧雷达检测到的对象数量
    for t in radar3.front_right_esr_tracklist:  # 遍历右侧雷达对象列表
        append_obj(t, msg_radar.objects_right)  # 将对象添加到msg_radar对象列表中

    # 添加时间戳
    msg_radar.header.stamp = rospy.Time.now()  # 设置当前时间为消息的时间戳
    pub.publish(msg_radar)  # 发布合成的雷达消息


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node("radar_listener", anonymous=True)
    # 初始化发布者
    pub = rospy.Publisher("/radar_msgs_combined", MsgRadar, queue_size=10)  # 创建消息发布者
    # 订阅雷达消息
    sub_radar_2 = message_filters.Subscriber("/ARS_408_21_2_Topic", SensorMsgsRadar)  # 订阅第一个雷达
    sub_radar_3 = message_filters.Subscriber("/ARS_408_21_3_Topic", SensorMsgsRadar)  # 订阅第二个雷达
    # 同步时间戳
    sync = message_filters.ApproximateTimeSynchronizer([sub_radar_2, sub_radar_3], 1, 1)  # 创建时间同步器

    sync.registerCallback(radar_listener)  # 注册回调函数

    print("\033[0;32m雷达监听器成功初始化。\033[0m")  # 输出初始化成功信息

    rospy.spin()  # 保持节点运行
