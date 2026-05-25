#! /usr/bin/env python
import rospy, cv2, cv_bridge#  导入ROS Python客户端库rospy，用于与ROS系统进行交互 导入OpenCV库cv2，用于图像处理 导入cv_bridge库，用于ROS图像消息与OpenCV图像之间的转换
import numpy as np #  导入NumPy库，用于进行数值计算
from sensor_msgs.msg import Image #  从sensor_msgs.msg模块导入Image消息类型，用于处理图像数据
from ackermann_msgs.msg import AckermannDriveStamped #  从ackermann_msgs.msg模块导入AckermannDriveStamped消息类型，用于控制车辆
from math import * #  导入math模块，提供数学函数
from gazebo_msgs.msg import ModelStates #  从gazebo_msgs.msg模块导入ModelStates消息类型，用于获取仿真环境中模型的状态
import message_filters #  导入message_filters库，用于同步多主题的消息
import time 
'''
区域1下降0.2
区域2下降0.3
区域3下降0.1
'''

def roll(p):
    # 确定是否选择该区域，根据概率p生成随机数
    if np.random.rand() <= p:
        return 1  # 选择该区域
    else:
        return 0  # 不选择该区域

def count(indexes):
    # 统计每个区域的选择次数
    density = [0, 0, 0, 0]  # 初始化四个区域的计数
    for i in indexes:
        density[int(i)] += 1  # 根据索引增加对应区域的计数
    return np.array(density)  # 返回计数结果

if __name__ == '__main__':
    rospy.init_node('Main')  # 初始化ROS节点
    beta_a = np.ones(3)  # Beta分布的参数a初始化为1
    beta_b = np.ones(3)  # Beta分布的参数b初始化为1
    probility = np.array([0.2, 0.3, 0.1])  # 各区域选择的真实概率
    regret = 0  # 懊悔值初始化为0
    regret_file = open('/home/computenest/catkin_ws/src/site_model/src/scripts_wx/RMMDet_MAB/Regret_Record_04.txt', 'w')  # 打开文件记录懊悔值

    try:
        while True:
            time.sleep(1)  # 每次循环等待1秒
            # MAB算法部分
            indexes = []
            for i in range(3):
                # 获取每个区域的选择索引
                indexes.append(int(rospy.get_param('car' + str(i + 1) + '/strict_index')))
            indexes = np.array(indexes)  # 转换为NumPy数组
            density = np.delete(count(indexes) * 1000, 0)  # 统计选择密度并删除第一个区域的计数
            award_predicted = density * np.random.beta(beta_a, beta_b)  # 预测奖励
            strict_selected = np.argmax(award_predicted) + 1  # 选择奖励最大的区域索引（从1开始）
            # 将参数设置为选中的区域
            rospy.set_param('strict_selected', float(strict_selected))  # 更新选中的区域参数
            award = roll(probility[strict_selected - 1])  # 根据真实概率决定是否获得奖励
            beta_a[strict_selected - 1] += award  # 更新Beta分布的参数a
            beta_b[strict_selected - 1] += 1 - award  # 更新Beta分布的参数b

            # 计算懊悔
            real_award = probility * density  # 计算真实奖励
            regret += np.max(real_award) - real_award[strict_selected - 1]  # 更新懊悔值
            regret = round(regret, 2)  # 四舍五入懊悔值
            print('Regret now: ', regret)  # 打印当前的懊悔值
            regret_file.write(str(regret) + '\n')  # 将懊悔值写入文件
    except:
        regret_file.close()  # 关闭懊悔值文件
        pass