#!/usr/bin/env python3
import rospy

from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty

banner = """
1. Reading from the keyboard  
2. Publishing to AckermannDriveStamped!

Rover0
---------------------------
        上
   左    下    右
anything else : stop
--------------------------
  key:(speed,steer)
 
Rover1
---------------------------
        i
   j    k    l
anything else : stop
--------------------------
  key:(speed,steer)
  
CTRL-C to quit
"""

keyBindings0 = {
  'A':(0.5,0),
  'C':(0.2,-0.6),
  'D':(0.2,0.6),
  'B':(-0.5,0),
}
keyBindings1 = {
  'i':(0.5,0),
  'l':(0.2,-0.6),
  'j':(0.2,0.6),
  'k':(-0.5,0),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed0 = 1
turn0 = 1

speed1 = 1
turn1 = 1
def vels0(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed0,turn0)
def vels1(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed1,turn1)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub0 = rospy.Publisher("ugv0/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
  pub1 = rospy.Publisher("ugv1/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
  rospy.init_node('keyop')
  print(banner)

  x0 = 0
  th0 = 0
  status0 = 0
  
  x1 = 0
  th1 = 0
  status1 = 0

  try:
    while(1):
       msg0 = AckermannDriveStamped();
       msg0.header.stamp = rospy.Time.now();
       msg0.header.frame_id = "base_link0";
       msg0.drive.acceleration = 1;
       msg0.drive.jerk = 1;
       
       msg1 = AckermannDriveStamped();
       msg1.header.stamp = rospy.Time.now();
       msg1.header.frame_id = "base_link1";
       msg1.drive.acceleration = 1;
       msg1.drive.jerk = 1;
       
       key = getKey()
       if key in keyBindings0.keys():
          x0 = keyBindings0[key][0]
          th0 = keyBindings0[key][1]
          msg0.drive.speed = x0*speed0;
          msg0.drive.steering_angle = th0*turn0
          msg0.drive.steering_angle_velocity = 1
          pub0.publish(msg0)
       elif key in keyBindings1.keys():
          x1 = keyBindings1[key][0]
          th1 = keyBindings1[key][1]
          msg1.drive.speed = x1*speed1;
          msg1.drive.steering_angle = th1*turn1
          msg1.drive.steering_angle_velocity = 1
          pub1.publish(msg1)
       else:
          x0 = 0
          th0 = 0
          x1 = 0
          th1 = 0
          if (key == '\x03'):
             break
          msg0.drive.speed = x0*speed0;
          msg0.drive.steering_angle = th0*turn0
          msg0.drive.steering_angle_velocity = 1
          msg1.drive.speed = x1*speed1;
          msg1.drive.steering_angle = th1*turn1
          msg1.drive.steering_angle_velocity = 1
          pub0.publish(msg0)
          pub1.publish(msg1)

  except:
    print ('error')

  finally:
    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "base_link0";

    msg.drive.speed = 0;
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    msg.drive.steering_angle = 0
    msg.drive.steering_angle_velocity = 1
    pub0.publish(msg)
    msg.header.frame_id = "base_link1";
    pub1.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
