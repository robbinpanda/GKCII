#! /usr/bin/env python
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from math import *
from gazebo_msgs.msg import ModelStates
import message_filters
import time
from msgs.msg import StrictIndex

class Car:
    def __init__(self, model_name, start_index, data_path):
        # 初始化车辆类
        self.cmd_vel_pub = rospy.Publisher("/"+model_name+"/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)
        self.strict_index_pub = rospy.Publisher('/'+model_name+'/strict_index', StrictIndex, queue_size=10)
        self.model_index = None  # 车辆索引
        self.model_name = model_name  # 车辆名称
        self.strict_index = start_index  # 开始时的严格索引
        self.data_file = open(data_path, 'a+')  # 打开数据文件以追加写入
        self.circle_start = 0  # 圆形区域开始时间
        self.circle_end = 0  # 圆形区域结束时间
        self.time_in_circle = 0  # 在圆形区域的时间
        self.time_start = 0  # 启动时间
        self.print_flag = 1  # 打印标志

    def run(self):
        # 启动ROS节点并订阅相机和模型状态
        self.time_start = round(time.time(), 2)
        rospy.init_node("RunningCar_"+self.model_name)
        m1 = message_filters.Subscriber("/"+self.model_name+"/"+self.model_name+"/camera/zed_left/image_rect_color_left", Image)
        m2 = message_filters.Subscriber("/gazebo/model_states", ModelStates)
        ts = message_filters.ApproximateTimeSynchronizer([m1, m2], 1, 1, allow_headerless=True)
        ts.registerCallback(self.callback)
        rospy.spin()  # 保持节点运行

    def callback(self, image_msg, model_states_msg):
        # 处理回调函数，获取模型状态和图像信息
        now = round(time.time(), 2)
        time_used = now - self.time_start
        self.data_file.write('Time used: '+str(time_used)+', Time in circle: '+str(self.time_in_circle)+'\n')
        
        # 从参数服务器获取参数
        self.p_strict_1 = rospy.get_param('p_strict_1', 0.7)  # 区域1的概率
        self.p_strict_2 = rospy.get_param('p_strict_2', 0.8)  # 区域2的概率
        self.p_strict_3 = rospy.get_param('p_strict_3', 0.6)  # 区域3的概率

        # 将自己的位置写入参数服务器
        rospy.set_param(self.model_name+'/strict_index', self.strict_index)

        # 判断是否需要调整概率
        use_mab = rospy.get_param('use_mab')
        if use_mab:
            tensive_strict = rospy.get_param('strict_selected', 0)
            if tensive_strict == self.strict_index:
                if self.print_flag:
                    print(self.model_name + ': Parameters changed!')
                    self.print_flag = 0
                self.adjust_param()  # 调整参数

        bridge = cv_bridge.CvBridge()
        frame = bridge.imgmsg_to_cv2(image_msg, 'bgr8')  # 将ROS图像消息转换为OpenCV格式

        if not self.model_index:
            self.model_index = self.get_model_index(model_states_msg)  # 获取模型索引
        model_pos = model_states_msg.pose[self.model_index].position  # 获取模型位置
        self.running(frame, model_pos)  # 运行车辆逻辑

    def reset_param(self):
        # 重置参数
        self.print_flag = 1
        rospy.set_param('p_strict_1', 0.7)
        rospy.set_param('p_strict_2', 0.8)
        rospy.set_param('p_strict_3', 0.6)
    
    def adjust_param(self):
        # 调整概率参数
        rospy.set_param('p_strict_1', 0.5)
        rospy.set_param('p_strict_2', 0.5)
        rospy.set_param('p_strict_3', 0.5)
        
    def running(self, image, pos):
        # 根据严格索引运行不同的逻辑
        pos_x = round(pos.x, 3)
        pos_y = round(pos.y, 3)
        if self.strict_index == 1:
            self.runningcar_1(image, pos_x, pos_y)  # 执行区域1的逻辑
        elif self.strict_index == 2:
            self.runningcar_2(image, pos_x, pos_y)  # 执行区域2的逻辑
        elif self.strict_index == 3:
            self.runningcar_3(image, pos_x, pos_y)  # 执行区域3的逻辑
        elif self.strict_index == 0:
            self.runningcar_0(image, pos_x, pos_y)  # 执行默认逻辑

    def record(self):
        # 记录时间
        self.circle_end = round(time.time(), 2)
        self.time_in_circle += self.circle_end - self.circle_start
        self.circle_end = 0
        self.circle_start = 0

    def set_roi_forward(self, h, w, mask):
        # 设置前方感兴趣区域（ROI）
        search_top = 400
        search_bot = search_top + 80
        mask[0:search_top, 0:w] = 0  # 上部区域置为0
        mask[search_bot:h, 0:w] = 0  # 下部区域置为0
        return mask
    
    def get_model_index(self, msg):
        # 获取模型的索引
        for i, name in enumerate(msg.name):
            if self.model_name == name:
                return i
    
    def roll(self, p):
        # 根据概率p决定是否返回True
        if np.random.rand() <= p:
            return True
        else:
            return False
        
    def follow_line(self, image):
        # 跟踪白线
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # 转换颜色空间
        lower_white = np.array([0, 0, 200])  # 白色的下界
        upper_white = np.array([180, 30,  255])  # 白色的上界
        mask = cv2.inRange(hsv, lower_white, upper_white)  # 创建白色掩膜

        h, w = mask.shape
        mask = self.set_roi_forward(h, w, mask)  # 设置ROI
        M = cv2.moments(mask)  # 计算图像矩
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00']) - 0
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)  # 在图像上绘制中心点
            err = cx - w / 2 - 50  # 计算偏差
            x = 0.1  # 前进速度
            z = -float(err / 2.0) / 50  # 转向角度
        else:
            # 如果未检测到白线，根据严格索引调整
            if self.strict_index == 2:
                print('Adjust ' + self.model_name + ' in strict 2...')
                x = 0.1
                z = 0.7
            elif self.strict_index == 0:
                print('Adjust ' + self.model_name + ' in strict 0...')
                x = 0.0
                z = -0.0
            else:
                x = 0.
                z = 0.    
        
        akm = AckermannDriveStamped()  # 创建AckermannDriveStamped消息
        akm.drive.speed = x * 1.80  # 设置速度
        if x == 0.0:
            akm.drive.steering_angle = 0  # 停止时转向角度为0
        else:
            akm.drive.steering_angle = atan(z / x * 0.133)  # 计算转向角度
        self.cmd_vel_pub.publish(akm)  # 发布控制命令

    def keep_running(self, x, a, duration):
        # 持续运行指定时间
        akm = AckermannDriveStamped()
        akm.drive.speed = x
        akm.drive.steering_angle = a

        self.cmd_vel_pub.publish(akm)
        time.sleep(duration)  # 等待指定时间

    # 环内的逻辑
    def turn_0_1(self):
        self.keep_running(0.2, 0.6, 5)  # 向左转
        self.keep_running(0.1, -0.7, 1)  # 向后转
        self.keep_running(0.1, 0.7, 2)  # 向右转

    def turn_0_2(self):
        self.keep_running(0.3, -0.25, 1)  # 向左轻微转

    def turn_0_2_(self):
        self.keep_running(0.3, 0.25, 3)  # 向右轻微转
        self.keep_running(0.2, -0.06, 3)  # 向后微调

    def turn_0_3(self):
        self.keep_running(0.3, 0.5, 4)  # 向左转

    def runningcar_0(self, image, pos_x, pos_y):
        # 区域0的运行逻辑
        err_x_1 = pos_x - 0.69
        err_y_1 = pos_y + 1.27

        err_x_2 = pos_x + 0.32
        err_y_2 = pos_y + 0.33

        err_x_3 = pos_x + 0.12
        err_y_3 = pos_y + 1.77

        # 根据位置误差判断是否进入新区域
        if abs(err_x_2) < 0.03 and abs(err_y_2) < 0.03:
            if self.roll(0.5):
                self.turn_0_2()
            else:
                self.turn_0_2_()
                self.record()
                self.strict_index = 3
                print(self.model_name + " enters strict 3!")
        elif abs(err_x_1) < 0.03 and abs(err_y_1) < 0.03:
            if self.roll(0.5):
                self.turn_0_1()
                self.record()
                self.strict_index = 2
                print(self.model_name + " enters strict 2!")
            else:
                self.follow_line(image)
        elif abs(err_x_3) < 0.03 and abs(err_y_3) < 0.03:
            if self.roll(0.5):
                self.turn_0_3()
                self.record()
                self.strict_index = 1
                print(self.model_name + " enters strict 1!")
            else:
                pass
        else:
            self.follow_line(image)  # 继续跟踪线

    # 第一个路口的逻辑
    def turn_1(self):
        self.keep_running(0.3, 0.7, 1.2)  # 向左转
        self.keep_running(0.3, 0.0, 1)  # 直行
        self.keep_running(0.3, 0.8, 1.6)  # 向右转
    
    def keep_1(self):
        self.keep_running(0.3, 0.7, 1.1)  # 继续向左转
        self.keep_running(0.3, 0.0, 2.7)  # 直行
    
    def runningcar_1(self, image, pos_x, pos_y):
        # 区域1的运行逻辑
        err_x = pos_x - 1.46
        err_y = pos_y + 2.68

        if abs(err_x) < 0.05 and abs(err_y) < 0.03:
            if self.roll(self.p_strict_1):
                self.turn_1()
                self.circle_start = round(time.time(), 2)
                print(self.model_name + " enters strict 0!")
                self.strict_index = 0
            else:
                self.keep_1()
                print(self.model_name + " enters strict 2!")
                self.strict_index = 2
            self.reset_param()
        else:
            self.follow_line(image)  # 继续跟踪线

    # 第二个路口的逻辑
    def turn_2(self):
        self.keep_running(0.3, -0.3, 2.5)  # 向左转
        self.keep_running(0.3, 0.7, 1.5)  # 向右转
        self.keep_running(0.3, 0., 1)  # 直行
        self.keep_running(0.3, 0.7, 1.0)  # 向右转

    def keep_2(self):
        self.keep_running(0.3, -0.5, 4.5)  # 向左转

    def runningcar_2(self, image, pos_x, pos_y):
        # 区域2的运行逻辑
        err_x_2 = pos_x - 0.8
        err_y_2 = pos_y - 1.1

        if abs(err_x_2) < 0.03 and abs(err_y_2) < 0.03:
            if self.roll(self.p_strict_2):
                self.turn_2()
                self.circle_start = round(time.time(), 2)
                print(self.model_name + " enters strict 0!")
                self.strict_index = 0
            else:
                self.keep_2()
                print(self.model_name + " enters strict 3!")
                self.strict_index = 3
            self.reset_param()
        else:
            self.follow_line(image)  # 继续跟踪线

    # 第三个路口的逻辑
    def keep_3(self):
        self.keep_running(0.2, -0.02, 6)  # 向左转

    def turn_3(self):
        self.keep_running(0.3, 0.6, 3.5)  # 向左转
        self.keep_running(0.3, -0.5, 1)  # 向右转
        self.keep_running(0.3, 0.7, 1.5)  # 向左转

    def runningcar_3(self, image, pos_x, pos_y):
        # 区域3的运行逻辑
        err_x_1 = pos_x + 1.74
        err_y_1 = pos_y - 1.65
        err_x_2 = pos_x + 1.77
        err_y_2 = pos_y + 1.83

        if abs(err_x_1) < 0.03 and abs(err_y_1) < 0.03:
            self.keep_3()
        elif abs(err_x_2) < 0.03 and abs(err_y_2) < 0.03:
            if self.roll(self.p_strict_3):
                self.turn_3()
                self.circle_start = round(time.time(), 2)
                print(self.model_name + " enters strict 0!")
                self.strict_index = 0
            else:
                print(self.model_name + " enters strict 1!")
                self.strict_index = 1
            self.reset_param()
        else:
            self.follow_line(image)  # 继续跟踪线