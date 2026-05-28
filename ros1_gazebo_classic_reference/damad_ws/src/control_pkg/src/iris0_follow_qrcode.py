#!/usr/bin/env python3

#二维码设置路径 uavros_gazebo/models/landing_pad_rover
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import *
from mavros_msgs.msg import State
import math
import sys, select, termios, tty
import pyapriltags
#import apriltag
import time

#获取键值函数
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
def iris0_callback(msg):
    global x, y, z, th, target_x_speed, target_y_speed, target_z_speed, target_turn, control_x_speed, control_y_speed, control_z_speed, control_turn, speed, turn, current_hight, target_hight, start_follow, init_pos, curr_pos
    #print("init_pos:", init_pos)
    #print("curr_pos:", curr_pos)
    
    image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 识别需要跟随的二维码
    if detector.detect(gray):
        tags = detector.detect(gray)
        for tag in tags:
            if tag.tag_id == 18 and start_follow:
                # 绘制识别出的二维码的四个点
                cv2.circle(image, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
                cv2.circle(image, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
                cv2.circle(image, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
                cv2.circle(image, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
                #print("tag id: ", tag.tag_id)
                #print(tag.corners[0].astype(int),tag.corners[1].astype(int),tag.corners[2].astype(int),tag.corners[3].astype(int))
            
                
                k = (tag.corners[0].astype(int)[1] - tag.corners[3].astype(int)[1]) / (tag.corners[0].astype(int)[0] - tag.corners[3].astype(int)[0])
                """if k < 0:
                    th = -0.1
                    
                elif k > 0:
                    th = 0.1
                else:
                    th = 0"""
                    
                # 计算二维码中心点
                tag_x = int((tag.corners[1].astype(int)[0] + tag.corners[0].astype(int)[0]) / 2)
                tag_y = int((tag.corners[1].astype(int)[1] + tag.corners[2].astype(int)[1]) / 2)
                # 计算图像中心点
                image_x = int(image.shape[0] / 2)
                image_y = int(image.shape[1] / 2)
                #print("二维码中心点坐标为:", tag_x, tag_y)
                #print("图像中心点坐标为:", image_x, image_y)
                
                # 根据二维码中心点和图像中心点的xy坐标差距设置相应的飞行速度
                x = (image_y - tag_y) * 0.019
                y = (image_x - tag_x) * 0.019
                """if tag_y > image_y:
                    #print("往后")
                    x = -(tag_y - image_y) * 0.4 / 25
                elif image_y > tag_y:
                    #print("往前")
                    x = (image_y - tag_y) * 0.4 / 25
                else:
                    x = 0
            
                if tag_x > image_x:
                    #print("往右")
                    y = -(tag_x - image_x) * 0.4 / 25
                elif image_x > tag_x:
                    #print("往左")
                    y = (image_x - tag_x) * 0.4 / 25
                else:
                    y = 0"""
    
    # 升高到一定高度时开始识别二维码                
    if (target_hight - current_hight) < 2:
        start_follow = True   
    # 无人机保持指定高度飞行
    if (current_hight - target_hight) > 0.2:
        #print("下降")
        z = -0.5
    elif (current_hight - target_hight) < -0.2:
        #print("升高")
        z = 0.5
    else:
        z = 0
        
    if not start_follow:
        x = -init_pos.x + curr_pos.x
        y = -init_pos.y + curr_pos.y
        
    th = 0
    
    #print("target_hight:", target_hight, " current_hight:", current_hight)
    #print(x, y, z, th)
    
    #根据速度与方向计算目标速度
    target_x_speed = speed * x
    target_y_speed = speed * y
    target_z_speed = speed * z
    target_turn = turn * th


    #x方向平滑控制，计算前进后退实际控制速度
    if target_x_speed > control_x_speed:
        control_x_speed = min( target_x_speed, control_x_speed + 0.1 )
    elif target_x_speed < control_x_speed:
        control_x_speed = max( target_x_speed, control_x_speed - 0.1 )
    else:
        control_x_speed = target_x_speed
                
                
    #y方向平滑控制，计算前进后退实际控制速度
    if target_y_speed > control_y_speed:
        control_y_speed = min( target_y_speed, control_y_speed + 0.1 )
    elif target_y_speed < control_y_speed:
        control_y_speed = max( target_y_speed, control_y_speed - 0.1 )
    else:
        control_y_speed = target_y_speed
            
    #z方向平滑控制，实际控制速度
    if target_z_speed > control_z_speed:
        control_z_speed = min( target_z_speed, control_z_speed + 0.1 )
    elif target_z_speed < control_z_speed:
        control_z_speed = max( target_z_speed, control_z_speed - 0.1 )
    else:
        control_z_speed = target_z_speed

    #平滑控制，计算转向实际控制速度
    if target_turn > control_turn:
        control_turn = min( target_turn, control_turn + 0.5 )
    elif target_turn < control_turn:
        control_turn = max( target_turn, control_turn - 0.5 )
    else:
        control_turn = target_turn
         
    # 计算出y方向的sin值
    y_sin = math.sin(sita/180*math.pi)
    # 如果小于0，则改为正数
    if y_sin < 0:
        y_sin = -y_sin
    # 乘以y分量的正负（通过四元数z*w获得，z*w>0,y分量在x轴上方）
    y_sin = y_sin * zf

    twist = Twist()  #创建ROS速度话题变量
    
    #正反朝向前进时
    if x != 0 and y == 0:
        twist.linear.x = control_x_speed * math.cos(sita/180*math.pi)
        twist.linear.y = control_x_speed * y_sin  # 朝向速度乘以y轴sin值
    #左右平移时
    if y != 0 and x == 0:
        twist.linear.x = control_y_speed * -y_sin
        twist.linear.y = control_y_speed * math.cos(sita/180*math.pi)
    #斜向移动？
    if x != 0 and y != 0:
        twist.linear.x = control_x_speed * math.cos(sita/180*math.pi)
        twist.linear.y = control_y_speed * math.cos(sita/180*math.pi)
                
    twist.linear.z = control_z_speed
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = control_turn

    pub.publish(twist) #ROS发布速度话题
            
    cv2.imshow("iris0_image", image)
    cv2.waitKey(1)


# 回调函数:订阅无人机位姿
def pose_cb(m):
    global sita
    global z
    global w
    global zf
    global current_hight
    
    global init_pos, curr_pos
    if init_pos is None:
        init_pos = m.pose.position
    curr_pos = m.pose.position
        
    current_hight = m.pose.position.z
    z = m.pose.orientation.z
    w = m.pose.orientation.w
    # 计算朝向在x轴的上方还是下方
    if z*w > 0:
        zf = 1
    else:
        zf = -1
    sita = 2*math.acos(w)*180/math.pi
    # rospy.loginfo('%.2f\r',sita)

# 回调函数：订阅mavros状态
def state_cb(state):
    global current_state
    current_state = state
    
def takeoff():
    global current_state
    x = -1
    y = 0
    while(1):
        key = getKey() #获取键值,去除后无法起飞？
        if x == 0:
            # 开启offboard模式
            setModeServer(custom_mode='OFFBOARD')
            print("UAV0 Offboard enabled")
            x += 1
        elif x == 1:
            # 解锁，准备起飞
            armServer(True) 
            print("UAV0 Vehicle armed")
            x += 1
            time.sleep(2)
        elif x == 2:
            # 起飞
            print("UAV0 Vehicle Takeoff")
            setModeServer(custom_mode='AUTO.TAKEOFF')
            x += 1
            time.sleep(5)
        elif x == 3:
            # 获得控制权
            setModeServer(custom_mode='OFFBOARD')
            x += 1
            print("UAV0 Offboard enabled")
        elif x == 4:
            break
                

        twist = Twist()  #创建ROS速度话题变量
        pub.publish(twist) #ROS发布速度话题,去除后无法起飞？
        y += 1
        if y == 10:
            x = 0
#主函数
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin) #获取键值初始化，读取终端相关属性
    detector = pyapriltags.Detector() #二维码识别器
    rospy.init_node('uav0_follow') #创建ROS节点
    
    pub = rospy.Publisher('uav0/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5) #创建速度话题发布者
    
    init_pos = None
    curr_pos = None
    
    # 订阅无人机位姿
    current_hight  = 0   # 当前高度
    target_hight   = 6.5   #目标高度
    start_follow = False #第一次达到目标高度后开始识别二维码跟踪
    x      = 0   #前进后退方向
    y      = 0   #左右移动方向
    z      = 0   #上下移动方向
    th     = 0   #转向/横向移动方向
    target_x_speed = 0 #前进后退目标速度
    target_y_speed = 0 #左右平移目标速度
    target_z_speed = 0 #上下运动目标速度
    target_turn  = 0 #转向目标速度
    control_x_speed = 0 #前进后退实际控制速度
    control_y_speed = 0 #左右平移实际控制速度
    control_z_speed = 0 #上下运动实际控制速度
    control_turn  = 0 #转向实际控制速度
    
    speed = 1 #默认移动速度 m/s
    turn  = 1   #默认转向速度 rad/s
    
    sita = 0.0  # 朝向
    z = 0
    w = 0
    zf = 1
    # 订阅uav0位姿信息
    rospy.Subscriber('uav0/mavros/local_position/pose',PoseStamped, pose_cb)

    # 订阅mavros状态
    current_state = State()
    rospy.Subscriber('uav0/mavros/state',State,state_cb)

    # 定义起飞降落服务客户端（起飞，降落）
    setModeServer = rospy.ServiceProxy('uav0/mavros/set_mode',SetMode)
    armServer = rospy.ServiceProxy('uav0/mavros/cmd/arming', CommandBool)

    # uav0起飞
    takeoff()
    # 订阅uav0上的摄像头图像信息
    iris0_sub = rospy.Subscriber("/iris0/fpv_cam/image_raw", Image, iris0_callback)
    rospy.spin()
    
    #程序结束前设置终端相关属性
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

