#! /usr/bin/env python
import rospy  #  导入ROS的Python客户端库
import sys, select, termios, tty #  导入系统库、select库用于多路复用、termios库用于终端I/O控制、tty库用于终端处理
from ackermann_msgs.msg import AckermannDriveStamped 

def getKey():
    # 设置终端为原始模式，以便直接读取键盘输入
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)  # 等待输入
    key = sys.stdin.read(1)  # 读取一个字符
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # 恢复终端设置
    return key #  恢复标准输入的设置

# 获取当前终端设置
settings = termios.tcgetattr(sys.stdin)

def pub_cmd():
    index = 1  # 车辆索引
    rospy.init_node("pub_cmd")  # 初始化ROS节点
    # rospy.set_param("Car Index", 1)  # 设置车辆参数（已注释）
    pub = rospy.Publisher("/car"+str(index)+"/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)  # 发布AckermannDriveStamped消息

    akm = AckermannDriveStamped()  # 创建AckermannDriveStamped消息对象
    
    while True:
        x = 0  # 速度
        a = 0  # 转向角度

        key = getKey()  # 获取用户输入的键
        rospy.loginfo("键盘已录入：%s", key)  # 记录输入的键
        if key == 'w':  # 向前
            x = 0.3  # 设置速度为0.3
            a = 0  # 直行
        elif key == 's':  # 向后
            x = -0.3  # 设置速度为-0.3
            a = 0  # 直行
        elif key == 'a':  # 左转
            x = 0.3  # 设置速度为0.3
            a = 0.7  # 设置转向角度为0.7
        elif key == 'd':  # 右转
            x = 0.3  # 设置速度为0.3
            a = -0.7  # 设置转向角度为-0.7
        elif key == 'x':  # 停止
            x = 0  # 速度为0
            a = 0  # 直行
        elif key == 'o':  # 退出
            break  # 退出循环
        else:
            continue  # 如果输入无效，则继续循环

        # akm.drive.speed = x * 6.97674  # 可选：速度缩放（已注释）
        akm.drive.speed = x  # 设置速度
        akm.drive.steering_angle = a * 0.7  # 设置转向角度
        # akm.drive.jerk = 2  # 可选：设置加速度（已注释）

        pub.publish(akm)  # 发布控制命令

if __name__ == "__main__":
    pub_cmd()  # 调用函数以开始发布命令
    pass  # 结束程序