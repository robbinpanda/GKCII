#!/usr/bin/env python3
import rospy

from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty

banner = """
1. Reading from the keyboard  
2. Publishing to AckermannDriveStamped!
---------------------------
        上
   左    下    右
anything else : stop
--------------------------
  key:(speed,steer)
CTRL-C to quit
"""

keyBindings = {
  'A':(0.5,0),    #上
  'C':(0.2,-0.6), #右
  'D':(0.2,0.6),  #左
  'B':(-0.5,0),   #下
}

# 获取按键
def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 1
turn = 1

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  # 新建keyop0节点
  rospy.init_node('keyop0')
  # 发布Ackermann消息，由rover0_servo_commands.py中节点接收，控制车辆
  pub = rospy.Publisher("ugv0/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
  print(banner)

  x = 0
  th = 0
  status = 0

  try:
    while(1):
       key = getKey()
       # 根据按键设置速度与转角
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
       # 按下无关按键停止行驶
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break
       msg = AckermannDriveStamped();
       msg.header.stamp = rospy.Time.now();
       msg.header.frame_id = "base_link";

       msg.drive.speed = x*speed;
       msg.drive.acceleration = 1;
       msg.drive.jerk = 1;
       msg.drive.steering_angle = th*turn
       msg.drive.steering_angle_velocity = 1
       # 发布Ackermann消息
       pub.publish(msg)

  except:
    print ('error')

  finally:
    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "base_link";

    msg.drive.speed = 0;
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    msg.drive.steering_angle = 0
    msg.drive.steering_angle_velocity = 1
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
