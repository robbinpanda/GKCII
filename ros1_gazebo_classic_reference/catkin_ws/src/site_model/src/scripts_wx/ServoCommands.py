#!/usr/bin/python3
import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

def set_throttle_steer(data):
    # 发布者，用于控制四个轮子的速度和两个转向铰链的位置
    pub_vel_left_rear_wheel = rospy.Publisher("/car1/left_rear_wheel_velocity_controller/command", Float64, queue_size=10)
    pub_vel_right_rear_wheel = rospy.Publisher("/car1/right_rear_wheel_velocity_controller/command", Float64, queue_size=10)
    pub_vel_left_front_wheel = rospy.Publisher("/car1/left_front_wheel_velocity_controller/command", Float64, queue_size=10)
    pub_vel_right_front_wheel = rospy.Publisher("/car1/right_front_wheel_velocity_controller/command", Float64, queue_size=10)
    pub_pos_left_steering_hinge = rospy.Publisher("/car1/left_steering_hinge_position_controller/command", Float64, queue_size=10)
    pub_pos_right_steering_hinge = rospy.Publisher("/car1/right_steering_hinge_position_controller/command", Float64, queue_size=10)

    # 从接收到的消息中获取速度和转向角度
    throttle = data.drive.speed * 28  # 将速度放大28倍以适应控制器
    steer = data.drive.steering_angle  # 获取转向角度

    # 将速度和转向角度发布到对应的控制器
    pub_vel_left_rear_wheel.publish(throttle) #  发布左后轮的速度指令
    pub_vel_right_rear_wheel.publish(throttle) #  发布右后轮的速度指令
    pub_vel_left_front_wheel.publish(throttle) #  发布左前轮的速度指令
    pub_vel_right_front_wheel.publish(throttle) #  发布右前轮的速度指令
    pub_pos_left_steering_hinge.publish(steer) #  发布左转向节的转向指令
    pub_pos_right_steering_hinge.publish(steer) #  发布右转向节的转向指令

def servo_commands():
    # 初始化ROS节点 ，节点名为'servo_commands1'，anonymous=True表示节点名称会自动添加随机数，确保唯一性
    rospy.init_node('servo_commands1', anonymous=True)
    # 订阅AckermannDriveStamped消息，这里用于获取驱动命令
    rospy.Subscriber("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer) #  订阅主题"/car1/ackermann_cmd_mux/output"，消息类型为AckermannDriveStamped，回调函数为set_throttle_steer
    rospy.spin()  # 保持节点运行，等待回调

if __name__=='__main__':
    try:
        servo_commands()  # 启动服务命令
    except rospy.ROSInternalException:
        pass  # 捕获ROS内部异常