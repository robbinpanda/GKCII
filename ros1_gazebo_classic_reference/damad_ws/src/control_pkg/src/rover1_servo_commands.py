#!/usr/bin/env python3
"""import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import math
import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf.transformations as tft
flag_move = 0
def get_orientation(object_orientation):
    # 注意：从get_model_state获得的orientation信息是四元数，将其转化为欧拉角后其偏航角yaw才是我们想要的orientation
    quaternion = [object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w]

    # 转换为欧拉角
    euler = tft.euler_from_quaternion(quaternion)
    yaw = euler[2]
    return yaw
def set_throttle_steer(data):
    rospy.wait_for_service('/gazebo/get_model_state')

    # 创建服务代理
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # 创建请求对象
    model_rover = GetModelStateRequest()
    model_rover.model_name = 'rover1'
    # 调用服务并传入请求
    objstate_rover = get_state_service(model_rover)

    state_rover = (objstate_rover.pose.position.x, objstate_rover.pose.position.y, objstate_rover.pose.position.z)

    # 提取角度信息
    state_orientation_rover = get_orientation(objstate_rover.pose.orientation)

    wheel_radius = 0.3175
    #wheel_base = 2.196 #distance between front and rear
    wheel_base = 2.054 #distance between front and rear
    wheel_dist = 1.43124 #distance between left and right 

    pub_vel_left_rear_wheel = rospy.Publisher('/ugv1/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/ugv1/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/ugv1/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/ugv1/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/ugv1/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/ugv1/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    #throttle = data.drive.speed*13.95348
    # this geometry analysis is by spx
    v = data.drive.speed
    steer = data.drive.steering_angle
    w_rl = v*(1-wheel_dist*math.tan(steer)/2/wheel_base)/wheel_radius #rear left
    w_rr = v*(1+wheel_dist*math.tan(steer)/2/wheel_base)/wheel_radius #rear right
    w_fl = v*math.sqrt(math.pow((1-wheel_dist*math.tan(steer)/2/wheel_base),2)+math.pow(math.tan(steer),2))/wheel_radius #front left
    w_fr = v*math.sqrt(math.pow((1+wheel_dist*math.tan(steer)/2/wheel_base),2)+math.pow(math.tan(steer),2))/wheel_radius #front right
    steer_fl = math.atan2(math.tan(steer),1-wheel_dist*math.tan(steer)/2/wheel_base)
    steer_fr = math.atan2(math.tan(steer),1+wheel_dist*math.tan(steer)/2/wheel_base)

    pub_vel_left_rear_wheel.publish(w_rl)
    pub_vel_right_rear_wheel.publish(w_rr)
    pub_vel_left_front_wheel.publish(w_fl)
    pub_vel_right_front_wheel.publish(w_fr)


def servo_commands():

    rospy.init_node('servo_commands1', anonymous=True)

    rospy.Subscriber("ugv1/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass"""

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

def set_throttle_steer(data):
    #global flag_move
    # 发布左后轮速度控制
    pub_vel_left_rear_wheel = rospy.Publisher("/ugv1/left_rear_wheel_velocity_controller/command", Float64, queue_size=1)
    # 发布右后轮速度控制
    pub_vel_right_rear_wheel = rospy.Publisher("/ugv1/right_rear_wheel_velocity_controller/command", Float64, queue_size=1)
    # 发布左前轮速度控制
    pub_vel_left_front_wheel = rospy.Publisher("/ugv1/left_front_wheel_velocity_controller/command", Float64, queue_size=1)
    # 发布右前轮速度控制
    pub_vel_right_front_wheel = rospy.Publisher("/ugv1/right_front_wheel_velocity_controller/command", Float64, queue_size=1)
    # 发布左前轮角度控制
    pub_pos_left_steering_hinge = rospy.Publisher("/ugv1/left_steering_hinge_position_controller/command", Float64, queue_size=1)
    # 发布右前轮速度控制
    pub_pos_right_steering_hinge = rospy.Publisher("/ugv1/right_steering_hinge_position_controller/command", Float64, queue_size=1)

    # 将Ackermann消息转换成合适的速度与转角
    throttle = data.drive.speed*28
    steer = data.drive.steering_angle

    # 发布消息
    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

def servo_commands():
    # 新建servo_commands1节点
    rospy.init_node('servo_commands1', anonymous=True)
    # 接收控制ugv1的Ackermann消息
    rospy.Subscriber("ugv1/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
