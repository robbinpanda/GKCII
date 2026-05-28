#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import *
from mavros_msgs.msg import State
import math
import sys, select, termios, tty

# 空格：降落
# 1  ：开启offboard模式
# 2  ：解锁，准备起飞
# 3  ：起飞

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

r/t : increase/decrease max speeds by 10%
f/g : increase/decrease only linear speed by 10%
v/b : increase/decrease only angular speed by 10%
space key, s : force stop
anything else : stop smoothly
b : switch to OmniMode/CommonMode
CTRL-C to quit
"""
Omni = 0 #全向移动模式

#键值对应移动/转向方向 (x, y, z, th)
moveBindings = {
        'q':( 1, 0, 0, 1),
        'w':( 1, 0, 0, 0),
        'e':( 1, 0, 0, -1),
        'a':( 0, 1, 0, 0),
        'd':( 0,-1, 0, 0),
        'z':(-1, 0, 0, -1),
        'x':(-1, 0, 0, 0),
        'c':(-1, 0, 0, 1),
        '4':(0, 0, 1, 0),
        '5':(0, 0, -1, 0),
           }

#键位不够
"""#键值对应速度增量
speedBindings={
        'r':(1.1,1.1),
        't':(0.9,0.9),
        'f':(1.1,1),
        'g':(0.9,1),
        'v':(1,  1.1),
        'b':(1,  0.9),
          }"""

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


speed = 0.8 #默认移动速度 m/s
turn  = 1   #默认转向速度 rad/s
#以字符串格式返回当前速度
def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

sita = 0.0  # 朝向
z = 0
w = 0
zf = 1
# 回调函数:订阅无人机位姿
def pose_cb(m):
    global sita
    global z
    global w
    global zf
    z = m.pose.orientation.z
    w = m.pose.orientation.w
    # 计算朝向在x轴的上方还是下方
    if z*w > 0:
        zf = 1
    else:
        zf = -1
    sita = 2*math.acos(w)*180/math.pi
    # rospy.loginfo('%.2f\r',sita)

current_state = State()
# 回调函数：订阅mavros状态
def state_cb(state):
    global current_state
    current_state = state

#主函数
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin) #获取键值初始化，读取终端相关属性
    
    rospy.init_node('turtlebot_teleop') #创建ROS节点
    pub = rospy.Publisher('uav0/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5) #创建速度话题发布者
    # 订阅无人机位姿
    rospy.Subscriber('uav0/mavros/local_position/pose',PoseStamped, pose_cb)

    # 订阅mavros状态
    rospy.Subscriber('uav0/mavros/state',State,state_cb)

    # 定义起飞降落服务客户端（起飞，降落）
    setModeServer = rospy.ServiceProxy('uav0/mavros/set_mode',SetMode)

    armServer = rospy.ServiceProxy('uav0/mavros/cmd/arming', CommandBool)

    x      = 0   #前进后退方向
    y      = 0   #左右移动方向
    z      = 0   #上下移动方向
    th     = 0   #转向/横向移动方向
    count  = 0   #键值不再范围计数
    target_x_speed = 0 #前进后退目标速度
    target_y_speed = 0 #左右平移目标速度
    target_z_speed = 0 #上下运动目标速度
    target_turn  = 0 #转向目标速度
    control_x_speed = 0 #前进后退实际控制速度
    control_y_speed = 0 #左右平移实际控制速度
    control_z_speed = 0 #上下运动实际控制速度
    control_turn  = 0 #转向实际控制速度
    try:
        print(msg) #打印控制说明
        print(vels(speed,turn)) #打印当前速度
        while(1):
            key = getKey() #获取键值

            #if key:
            #    print('key = ',key)
            
            #判断键值是否在移动/转向方向键值内
            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                count = 0


            """#判断键值是否在速度增量键值内
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn  = turn  * speedBindings[key][1]
                count = 0
                print(vels(speed,turn)) #速度发生变化，打印出来"""

            #空键值/'k',相关变量置0
            elif key == 's' :
                x  = 0
                y  = 0
                z  = 0
                th = 0
                control_x_speed = 0
                control_y_speed = 0
                control_z_speed = 0
                control_turn  = 0

            # 降落
            elif key == ' ':
                print("Vehicle Land")
                setModeServer(custom_mode='AUTO.LAND')
            # 开启offboard模式
            elif key == '1':
                if current_state.mode != "OFFBOARD" :
                    setModeServer(custom_mode='OFFBOARD')
                    print("Offboard enabled")
            # 解锁，准备起飞
            elif key == '2':
                armServer(True) 
                print("Vehicle armed")
            # 起飞
            elif key == '3':
                print("Vehicle Takeoff")
                setModeServer(custom_mode='AUTO.TAKEOFF')

            #长期识别到不明键值，相关变量置0
            else:
                count = count + 1
                if count > 4:
                    x  = 0
                    y  = 0
                    z  = 0
                    th = 0
                if (key == '\x03'):
                    break

            #根据速度与方向计算目标速度
            target_x_speed = speed * x
            target_y_speed = speed * y
            target_z_speed = speed * z
            target_turn  = turn * th

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
            if key == 'w' or key == 'x':
                twist.linear.x = control_x_speed * math.cos(sita/180*math.pi)
                twist.linear.y = control_x_speed * y_sin  # 朝向速度乘以y轴sin值
            #左右平移时
            if key == 'a' or key == 'd':
                twist.linear.x = control_y_speed * -y_sin
                twist.linear.y = control_y_speed * math.cos(sita/180*math.pi)
                
            twist.linear.z = control_z_speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = control_turn

            pub.publish(twist) #ROS发布速度话题

    #运行出现问题则程序终止并打印相关错误信息
    except Exception as e:
        print(e)

    #程序结束前发布速度为0的速度话题
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
        pub.publish(twist)

    #程序结束前设置终端相关属性
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


