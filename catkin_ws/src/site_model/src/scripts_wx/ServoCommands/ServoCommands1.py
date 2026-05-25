#!/usr/bin/python3
import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

def set_throttle_steer(data):
    car_index = 1
    pub_vel_left_rear_wheel = rospy.Publisher("/car"+str(car_index)+"/left_rear_wheel_velocity_controller/command", Float64, queue_size=10)
    pub_vel_right_rear_wheel = rospy.Publisher("/car"+str(car_index)+"/right_rear_wheel_velocity_controller/command", Float64, queue_size=10)
    pub_vel_left_front_wheel = rospy.Publisher("/car"+str(car_index)+"/left_front_wheel_velocity_controller/command", Float64, queue_size=10)
    pub_vel_right_front_wheel = rospy.Publisher("/car"+str(car_index)+"/right_front_wheel_velocity_controller/command", Float64, queue_size=10)
    pub_pos_left_steering_hinge = rospy.Publisher("/car"+str(car_index)+"/left_steering_hinge_position_controller/command", Float64, queue_size=10)
    pub_pos_right_steering_hinge = rospy.Publisher("/car"+str(car_index)+"/right_steering_hinge_position_controller/command", Float64, queue_size=10)
    
    throttle = data.drive.speed*28
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

def servo_commands():
    rospy.init_node('servo_commands1', anonymous=True)
    rospy.Subscriber("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)
    rospy.spin()

if __name__=='__main__':
    try:
        servo_commands()
    except rospy.ROSInternalException:
        pass






