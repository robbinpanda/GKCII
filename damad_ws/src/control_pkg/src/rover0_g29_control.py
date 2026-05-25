#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
throttleStart = False
brakeStart = False
class parameters:
    angleRange = 45
    velocityRange = 10
    backRange = 25
    gear1 = 0
    gear2 = 10
    gear3 = 20
    gear4 = 30
    gear5 = 40

def G29_callback(msg):
    akm = AckermannDriveStamped()
    akm.header.stamp = rospy.Time.now()
    akm.header.frame_id = "base_link"
    angle = min(int(abs(msg.axes[0]) * parameters.angleRange / 4),parameters.angleRange)
    angle *= [-1,1][msg.axes[0] >= 0]
    velocity = 0
    v = 0
    _v = 0
    global throttleStart, brakeStart
    #msg.axes[2]和msg.axes[3]启动时初始值为0，使用后初始值为-1
    if msg.axes[2] != 0 and not throttleStart:
        throttleStart = True
        print("油门激活")
    if msg.axes[3] != 0 and not brakeStart:
        brakeStart = True
        print("刹车激活")
    if throttleStart:
        if msg.buttons[12] == 1 or msg.buttons[12:19].count(0) == 7:
            v = 0
        else:
            if msg.buttons[13] == 1:
                v = int((msg.axes[2] + 1) * (parameters.velocityRange + parameters.gear1) / 2)
            elif msg.buttons[14] == 1:
                v = int((msg.axes[2] + 1) * (parameters.velocityRange + parameters.gear2) / 2)
            elif msg.buttons[15] == 1:
                v = int((msg.axes[2] + 1) * (parameters.velocityRange + parameters.gear3) / 2)
            elif msg.buttons[16] == 1:
                v = int((msg.axes[2] + 1) * (parameters.velocityRange + parameters.gear4) / 2)
            elif msg.buttons[17] == 1:
                v = int((msg.axes[2] + 1) * (parameters.velocityRange + parameters.gear5) / 2)
            elif msg.buttons[18] == 1:
                v = int((msg.axes[2] + 1) * parameters.velocityRange / 2) * -1
    if brakeStart:
        _v = int((msg.axes[3] + 1) * parameters.backRange / 2)
    if _v > 0:
        if msg.buttons[18] == 1:
            velocity = min(v + _v,0)
        else:
            velocity = max(v - _v,0)
    else:
        velocity = v
    akm.drive.speed = velocity * 0.01
    akm.drive.steering_angle = angle * 0.5

    pub.publish(akm)
    pass
    
if __name__=="__main__":
    index = 1
    rospy.init_node("g29_control0")
    pub = rospy.Publisher("ugv0/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)
    rospy.Subscriber("/joy", Joy, G29_callback)
    rospy.spin()
    #pub_cmd()
    pass
