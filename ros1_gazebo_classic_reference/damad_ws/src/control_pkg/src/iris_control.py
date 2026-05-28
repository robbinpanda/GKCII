#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import *
from mavros_msgs.msg import State
import math
import sys, select, termios, tty


msg = """
Control Your UAV!

UAV0
1  ：开启offboard模式
2  ：解锁，准备起飞
3  ：起飞
4  ：降落
---------------------------
Moving around:
        w    
   a    s    d
   z    x    c
q/e : up/down
space key, s : force stop

UAV1
5  ：开启offboard模式
6  ：解锁，准备起飞
7  ：起飞
8  ：降落
---------------------------
Moving around:
        t    
   f    g    h
   v    b    n
r/y : up/down
space key, g : force stop
CTRL-C to quit
"""

#键值对应移动/转向方向 (x, y, z, th)
moveBindings0 = {
        'q':(0, 0, 1, 0),
        'w':( 1, 0, 0, 0),
        'e':(0, 0, -1, 0),
        'a':( 0, 1, 0, 0),
        'd':( 0,-1, 0, 0),
        'z':(-1, 0, 0, -1),
        'x':(-1, 0, 0, 0),
        'c':(-1, 0, 0, 1),
           }
moveBindings1 = {
        'r':(0, 0, 1, 0),
        't':( 1, 0, 0, 0),
        'y':(0, 0, -1, 0),
        'f':( 0, 1, 0, 0),
        'h':( 0,-1, 0, 0),
        'v':(-1, 0, 0, -1),
        'b':(-1, 0, 0, 0),
        'n':(-1, 0, 0, 1),
           }
           
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


speed0 = 0.8 #默认移动速度 m/s
speed1 = 0.8 #默认移动速度 m/s
turn0  = 1   #默认转向速度 rad/s
turn1  = 1   #默认转向速度 rad/s
#以字符串格式返回当前速度
def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

sita0 = 0.0  # 朝向
sita1 = 0.0  # 朝向
z0 = 0
z1 = 0
w0 = 0
w1 = 0
zf0 = 1
zf1 = 1
# 回调函数:订阅无人机位姿
def pose_cb0(m):
    global sita0
    global z0
    global w0
    global zf0
    z0 = m.pose.orientation.z
    w0 = m.pose.orientation.w
    # 计算朝向在x轴的上方还是下方
    if z0*w0 > 0:
        zf0 = 1
    else:
        zf0 = -1
    sita0 = 2*math.acos(w0)*180/math.pi
    # rospy.loginfo('%.2f\r',sita)
def pose_cb1(m):
    global sita1
    global z1
    global w1
    global zf1
    z1 = m.pose.orientation.z
    w1 = m.pose.orientation.w
    # 计算朝向在x轴的上方还是下方
    if z1*w1 > 0:
        zf1 = 1
    else:
        zf1 = -1
    sita1 = 2*math.acos(w1)*180/math.pi
    # rospy.loginfo('%.2f\r',sita)

current_state0 = State()
current_state1 = State()
# 回调函数：订阅mavros状态
def state_cb0(state):
    global current_state0
    current_state0 = state
def state_cb1(state):
    global current_state1
    current_state1 = state
    
#主函数
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin) #获取键值初始化，读取终端相关属性
    
    rospy.init_node('uav_teleop') #创建ROS节点
    pub0 = rospy.Publisher('uav0/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5) #创建uav0速度话题发布者
    pub1 = rospy.Publisher('uav1/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5) #创建uav1速度话题发布者
    # 订阅无人机位姿
    rospy.Subscriber('uav0/mavros/local_position/pose',PoseStamped, pose_cb0)
    rospy.Subscriber('uav1/mavros/local_position/pose',PoseStamped, pose_cb1)

    # 订阅mavros状态
    rospy.Subscriber('uav0/mavros/state',State,state_cb0)
    rospy.Subscriber('uav1/mavros/state',State,state_cb1)

    # 定义起飞降落服务客户端（起飞，降落）
    setModeServer0 = rospy.ServiceProxy('uav0/mavros/set_mode',SetMode)
    setModeServer1 = rospy.ServiceProxy('uav1/mavros/set_mode',SetMode)
    
    armServer0 = rospy.ServiceProxy('uav0/mavros/cmd/arming', CommandBool)
    armServer1 = rospy.ServiceProxy('uav1/mavros/cmd/arming', CommandBool)

    x0      = 0   #前进后退方向
    y0      = 0   #左右移动方向
    z0      = 0   #上下移动方向
    th0     = 0   #转向/横向移动方向
    count0  = 0   #键值不再范围计数
    target_x_speed0 = 0 #前进后退目标速度
    target_y_speed0 = 0 #左右平移目标速度
    target_z_speed0 = 0 #上下运动目标速度
    target_turn0  = 0 #转向目标速度
    control_x_speed0 = 0 #前进后退实际控制速度
    control_y_speed0 = 0 #左右平移实际控制速度
    control_z_speed0 = 0 #上下运动实际控制速度
    control_turn0  = 0 #转向实际控制速度
    
    x1      = 0   #前进后退方向
    y1      = 0   #左右移动方向
    z1      = 0   #上下移动方向
    th1     = 0   #转向/横向移动方向
    count1  = 0   #键值不再范围计数
    target_x_speed1 = 0 #前进后退目标速度
    target_y_speed1 = 0 #左右平移目标速度
    target_z_speed1 = 0 #上下运动目标速度
    target_turn1  = 0 #转向目标速度
    control_x_speed1 = 0 #前进后退实际控制速度
    control_y_speed1 = 0 #左右平移实际控制速度
    control_z_speed1 = 0 #上下运动实际控制速度
    control_turn1  = 0 #转向实际控制速度
    
    try:
        print(msg) #打印控制说明
        #print(vels(speed,turn)) #打印当前速度
        while(1):
            key = getKey() #获取键值

            #if key:
            #    print('key = ',key)
            
            #判断键值是否在移动/转向方向键值内
            if key in moveBindings0.keys():
                x0  = moveBindings0[key][0]
                y0 = moveBindings0[key][1]
                z0 = moveBindings0[key][2]
                th0 = moveBindings0[key][3]
                count0 = 0
            elif key in moveBindings1.keys():
                x1  = moveBindings1[key][0]
                y1 = moveBindings1[key][1]
                z1 = moveBindings1[key][2]
                th1 = moveBindings1[key][3]
                count1 = 0


            #空键值/'k',相关变量置0
            elif key == 's' :
                x0  = 0
                y0  = 0
                z0  = 0
                th0 = 0
                control_x_speed0 = 0
                control_y_speed0 = 0
                control_z_speed0 = 0
                control_turn0  = 0
            elif key == 'g' :
                x1  = 0
                y1  = 0
                z1  = 0
                th1 = 0
                control_x_speed1 = 0
                control_y_speed1 = 0
                control_z_speed1 = 0
                control_turn1  = 0

            # 降落
            elif key == '4':
                print("UAV0 Vehicle Land")
                setModeServer0(custom_mode='AUTO.LAND')
            elif key == '8':
                print("UAV1 Vehicle Land")
                setModeServer1(custom_mode='AUTO.LAND')
            # 开启offboard模式
            elif key == '1':
                if current_state0.mode != "OFFBOARD" :
                    setModeServer0(custom_mode='OFFBOARD')
                    print("UAV0 Offboard enabled")
            elif key == '5':
                if current_state1.mode != "OFFBOARD" :
                    setModeServer1(custom_mode='OFFBOARD')
                    print("UAV1 Offboard enabled")
            # 解锁，准备起飞
            elif key == '2':
                armServer0(True) 
                print("UAV0 Vehicle armed")
            elif key == '6':
                armServer1(True) 
                print("UAV1 Vehicle armed")
            # 起飞
            elif key == '3':
                print("UAV0 Vehicle Takeoff")
                setModeServer0(custom_mode='AUTO.TAKEOFF')
            elif key == '7':
                print("UAV1 Vehicle Takeoff")
                setModeServer1(custom_mode='AUTO.TAKEOFF')

            #长期识别到不明键值，相关变量置0
            else:
                count0 = count0 + 1
                count1 = count1 + 1
                if count0 > 4:
                    x0  = 0
                    y0  = 0
                    z0  = 0
                    th0 = 0
                if count1 > 4:
                    x1  = 0
                    y1  = 0
                    z1  = 0
                    th1 = 0
                if (key == '\x03'):
                    break

            #根据速度与方向计算目标速度
            target_x_speed0 = speed0 * x0
            target_y_speed0 = speed0 * y0
            target_z_speed0 = speed0 * z0
            target_turn0  = turn0 * th0
            
            target_x_speed1 = speed1 * x1
            target_y_speed1 = speed1 * y1
            target_z_speed1 = speed1 * z1
            target_turn1  = turn1 * th1

            #x方向平滑控制，计算前进后退实际控制速度
            if target_x_speed0 > control_x_speed0:
                control_x_speed0 = min( target_x_speed0, control_x_speed0 + 0.1 )
            elif target_x_speed0 < control_x_speed0:
                control_x_speed0 = max( target_x_speed0, control_x_speed0 - 0.1 )
            else:
                control_x_speed0 = target_x_speed0
                
            if target_x_speed1 > control_x_speed1:
                control_x_speed1 = min( target_x_speed1, control_x_speed1 + 0.1 )
            elif target_x_speed1 < control_x_speed1:
                control_x_speed1 = max( target_x_speed1, control_x_speed1 - 0.1 )
            else:
                control_x_speed1 = target_x_speed1
                
            #y方向平滑控制，计算前进后退实际控制速度
            if target_y_speed0 > control_y_speed0:
                control_y_speed0 = min( target_y_speed0, control_y_speed0 + 0.1 )
            elif target_y_speed0 < control_y_speed0:
                control_y_speed0 = max( target_y_speed0, control_y_speed0 - 0.1 )
            else:
                control_y_speed0 = target_y_speed0
                
            if target_y_speed1 > control_y_speed1:
                control_y_speed1 = min( target_y_speed1, control_y_speed1 + 0.1 )
            elif target_y_speed1 < control_y_speed1:
                control_y_speed1 = max( target_y_speed1, control_y_speed1 - 0.1 )
            else:
                control_y_speed1 = target_y_speed1
            
            #z方向平滑控制，实际控制速度
            if target_z_speed0 > control_z_speed0:
                control_z_speed0 = min( target_z_speed0, control_z_speed0 + 0.1 )
            elif target_z_speed0 < control_z_speed0:
                control_z_speed0 = max( target_z_speed0, control_z_speed0 - 0.1 )
            else:
                control_z_speed0 = target_z_speed0
                
            if target_z_speed1 > control_z_speed1:
                control_z_speed1 = min( target_z_speed1, control_z_speed1 + 0.1 )
            elif target_z_speed1 < control_z_speed1:
                control_z_speed1 = max( target_z_speed1, control_z_speed1 - 0.1 )
            else:
                control_z_speed1 = target_z_speed1

            #平滑控制，计算转向实际控制速度
            if target_turn0 > control_turn0:
                control_turn0 = min( target_turn0, control_turn0 + 0.5 )
            elif target_turn0 < control_turn0:
                control_turn0 = max( target_turn0, control_turn0 - 0.5 )
            else:
                control_turn0 = target_turn0
                
            if target_turn1 > control_turn1:
                control_turn1 = min( target_turn1, control_turn1 + 0.5 )
            elif target_turn1 < control_turn1:
                control_turn1 = max( target_turn1, control_turn1 - 0.5 )
            else:
                control_turn1 = target_turn1
         
            # 计算出y方向的sin值
            y_sin0 = math.sin(sita0/180*math.pi)
            y_sin1 = math.sin(sita1/180*math.pi)
            # 如果小于0，则改为正数
            if y_sin0 < 0:
                y_sin0 = -y_sin0
            if y_sin1 < 0:
                y_sin1 = -y_sin1
            # 乘以y分量的正负（通过四元数z*w获得，z*w>0,y分量在x轴上方）
            y_sin0 = y_sin0 * zf0
            y_sin1 = y_sin1 * zf1

            twist0 = Twist()  #创建ROS速度话题变量
            twist1 = Twist()  #创建ROS速度话题变量
            #正反朝向前进时
            if key == 'w' or key == 'x':
                twist0.linear.x = control_x_speed0 * math.cos(sita0/180*math.pi)
                twist0.linear.y = control_x_speed0 * y_sin0  # 朝向速度乘以y轴sin值
            if key == 't' or key == 'b':
                twist1.linear.x = control_x_speed1 * math.cos(sita1/180*math.pi)
                twist1.linear.y = control_x_speed1 * y_sin1  # 朝向速度乘以y轴sin值
            #左右平移时
            if key == 'a' or key == 'd':
                twist0.linear.x = control_y_speed0 * -y_sin0
                twist0.linear.y = control_y_speed0 * math.cos(sita0/180*math.pi)
            if key == 'f' or key == 'h':
                twist1.linear.x = control_y_speed1 * -y_sin1
                twist1.linear.y = control_y_speed1 * math.cos(sita1/180*math.pi)
                
            twist0.linear.z = control_z_speed0
            twist0.angular.x = 0
            twist0.angular.y = 0
            twist0.angular.z = control_turn0

            pub0.publish(twist0) #ROS发布速度话题
            
            twist1.linear.z = control_z_speed1
            twist1.angular.x = 0
            twist1.angular.y = 0
            twist1.angular.z = control_turn1

            pub1.publish(twist1) #ROS发布速度话题

    #运行出现问题则程序终止并打印相关错误信息
    except Exception as e:
        print(e)

    #程序结束前发布速度为0的速度话题
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn0
        pub0.publish(twist)
        pub1.publish(twist)

    #程序结束前设置终端相关属性
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


