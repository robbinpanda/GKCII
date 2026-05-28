#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import *
from mavros_msgs.msg import State
import math
import sys, select, termios, tty
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
    
def iris1_callback():
    global x, y, z, th, target_x_speed, target_y_speed, target_z_speed, target_turn, control_x_speed, control_y_speed, control_z_speed, control_turn, speed, turn, current_position, init_position, target_hight, start_follow, target_position, target_count, refresh
    #获取到新的无人机位置数据时才会进行下一步
    if not refresh:
        return
        
    
    print("target_position:", target_position[target_count][0], target_position[target_count][1])
    print("current_position:", current_position.x, current_position.y)
    #开始跟随目标点
    if start_follow:
        x = (target_position[target_count][0] - current_position.x) * 0.1
        y = (target_position[target_count][1] - current_position.y) * 0.1

    
    # 升高到一定高度时              
    if (target_hight - current_position.z) < 0.2:
        start_follow = True   
    # 无人机保持指定高度飞行
    if (current_position.z - target_hight) > 0.1:
        #print("下降")
        z = -0.1
    elif (current_position.z - target_hight) < -0.1:
        #print("升高")
        z = 0.1
    else:
        z = 0
    
    #print("x:", init_position.x, current_position.x)
    #print("y:", init_position.y, current_position.y)
    #未开始跟随目标点时保持无人机位置在当前位置
    if not start_follow:
        x = (init_position.x - current_position.x) * 0.1
        y = (init_position.y - current_position.y) * 0.1
    refresh = False
    
    th = 0
    #判断是否在目标点附近
    if abs(target_position[target_count][0] - current_position.x) <= 0.25 and abs(target_position[target_count][1] - current_position.y) <= 0.25:
        target_count += 1 #切换到下一个目标点
    #当所有目标点巡航完毕时
    if target_count >= len(target_position):
        for i in range(100):
            setModeServer(custom_mode='AUTO.LAND') #降落
        time.sleep(25)
        for i in range(100):
            armServer(False)
        time.sleep(5)
        rospy.signal_shutdown("~owari~")
        exit(0)
        
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

# 回调函数:订阅无人机位姿
def pose_cb(m):
    global sita, z, w, zf, current_position, init_position, refresh
    #起飞时初始位置
    if init_position is None:
        init_position = m.pose.position
        #print("init_position:", init_position)
    current_position = m.pose.position #当前位置
    #print("current_position:", current_position)
    refresh = True
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
            print("UAV1 Offboard enabled")
            x += 1
        elif x == 1:
            # 解锁，准备起飞
            armServer(True) 
            print("UAV1 Vehicle armed")
            x += 1
            time.sleep(2)
        elif x == 2:
            # 起飞
            print("UAV1 Vehicle Takeoff")
            setModeServer(custom_mode='AUTO.TAKEOFF')
            x += 1
            time.sleep(10)
        elif x == 3:
            # 获得控制权
            setModeServer(custom_mode='OFFBOARD')
            x += 1
            print("UAV1 Offboard enabled")
        elif x == 4:
            break
                

        twist = Twist()  #创建ROS速度话题变量
        pub.publish(twist) #ROS发布速度话题,去除后无法起飞？
        y += 1
        if y == 10:
            x = 0
#主函数
if __name__=="__main__":
    global target_position, target_count, refresh
    target_position = [[4.8,2.25],[0,4.5],[4.8,6.75],[0,9]] #目标点路径
    target_count = 0
    refresh = False
    settings = termios.tcgetattr(sys.stdin) #获取键值初始化，读取终端相关属性
    rospy.init_node('uav1_cruise') #创建ROS节点
    
    pub = rospy.Publisher('uav1/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5) #创建速度话题发布者
    
    current_position = None
    init_position = None
    target_hight = 2 #目标高度
    start_follow = False #第一次达到目标高度后开始
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
    rospy.Subscriber('uav1/mavros/local_position/pose',PoseStamped, pose_cb)

    # 订阅mavros状态
    current_state = State()
    rospy.Subscriber('uav1/mavros/state',State,state_cb)

    # 定义起飞降落服务客户端（起飞，降落）
    setModeServer = rospy.ServiceProxy('uav1/mavros/set_mode',SetMode)
    armServer = rospy.ServiceProxy('uav1/mavros/cmd/arming', CommandBool)

    # uav1起飞
    takeoff()
    while True:
        iris1_callback()
    rospy.spin()
    
    #程序结束前设置终端相关属性
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

