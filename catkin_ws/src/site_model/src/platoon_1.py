#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import tf
import math
import sys, select, termios, tty
import threading

# ¿ØÖÆ×ŽÌ¬£º'leader' ±íÊŸ¿ØÖÆÇ°³µ²¢ÈÃºó³µžúËæ£¬'follower' ±íÊŸ¿ØÖÆºó³µ£¬È¡ÏûžúËæ
control_mode = 'leader'
PI = 3.1415926535

# ×ŽÌ¬±äÁ¿
pose_flag0 = False
pose_flag1 = False
initial_distance = 0
thresh_distance = 0.4
target_point = Odometry()
ugv0_msg = Odometry()
ugv1_msg = Odometry()
leader_msg_queue = []
event = threading.Event()
settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def pose_callback0(msg):
    global pose_flag0, ugv0_msg
    pose_flag0 = True
    ugv0_msg = msg

def pose_callback1(msg):
    global pose_flag1, ugv1_msg
    pose_flag1 = True
    ugv1_msg = msg

def get_pub(index):
    return rospy.Publisher("/car"+str(index)+"/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

def publish_stop(pub, frame_id):
    stop_msg = AckermannDriveStamped()
    stop_msg.header.frame_id = frame_id
    stop_msg.header.stamp = rospy.Time.now()
    stop_msg.drive.speed = 0.0
    stop_msg.drive.steering_angle = 0.0
    pub.publish(stop_msg)

def pub_cmd(frame_id):
    global control_mode
    index = 1
    pub = get_pub(index)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        key = getKey()
        akm = AckermannDriveStamped()
        akm.header.frame_id = frame_id
        akm.header.stamp = rospy.Time.now()
        rospy.loginfo("ŒüÅÌÒÑÂŒÈë£º%s", key)  # ŒÇÂŒÊäÈëµÄŒü

        x = 0
        a = 0

        if key == 'w':
            x = 0.3
        elif key == 's':
            x = -0.3
        elif key == 'a':
            x = 0.3
            a = 0.7
        elif key == 'd':
            x = 0.3
            a = -0.7
        elif key == 'x':
            x = 0
            a = 0
        elif key == 'c':
            # ÇÐ»»Ä£Êœ
            if control_mode == 'leader':
                publish_stop(pub, frame_id)
                control_mode = 'follower'
                index = 2
                pub = get_pub(index)
                rospy.loginfo("ÇÐ»»µœºó³µŒüÅÌ¿ØÖÆ")
                event.clear()
            else:
                publish_stop(pub, frame_id)
                control_mode = 'leader'
                index = 1
                pub = get_pub(index)
                rospy.loginfo("ÇÐ»»µœÇ°³µŒüÅÌ¿ØÖÆ + ºó³µžúËæ")
                event.set()
            continue
        elif key == 'o':
            publish_stop(pub, frame_id)
            event.set()
            break
        else:
            continue

        akm.drive.speed = x * 0.4
        akm.drive.steering_angle = a * 0.7
        pub.publish(akm)

def follow(frame_id, kousha):
    global pose_flag0, pose_flag1, initial_distance, target_point, leader_msg_queue

    pub = rospy.Publisher(f"/{kousha}/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)
    rate = rospy.Rate(100)
    msg = AckermannDriveStamped()
    msg.header.frame_id = frame_id
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle_velocity = 1
    
    initial_distance = 0.2

    while not rospy.is_shutdown():
        if control_mode != 'leader':
            rospy.loginfo("stop following")
            event.wait()
        if not pose_flag0 or not pose_flag1:
            rate.sleep()
            continue

        leader_msg_queue.append(ugv0_msg)

        follower_distance = math.hypot(ugv0_msg.pose.pose.position.x - ugv1_msg.pose.pose.position.x,
                                       ugv0_msg.pose.pose.position.y - ugv1_msg.pose.pose.position.y)
        target_distance = math.hypot(target_point.pose.pose.position.x - ugv1_msg.pose.pose.position.x,
                                     target_point.pose.pose.position.y - ugv1_msg.pose.pose.position.y)

        
        if follower_distance < initial_distance:
            msg.drive.speed = 0
            msg.drive.steering_angle = 0
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rospy.loginfo("too close and stop")
        else:
            #rospy.loginfo("current following distance£º%s", follower_distance)
            if target_point.pose.pose.position.x == 0 and target_point.pose.pose.position.y == 0:
                target_point = ugv0_msg
                initial_distance = follower_distance * 0.4
                rospy.loginfo("shortest following distance£º%s", initial_distance)
                thresh_distance = initial_distance * 2.5

            if target_distance < thresh_distance and leader_msg_queue:
                target_point = leader_msg_queue.pop(0)
            elif target_distance > 5:
                leader_msg_queue.clear()
                target_point = ugv0_msg

            q = ugv1_msg.pose.pose.orientation
            (_, _, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            gamma = yaw
            dx = target_point.pose.pose.position.x - ugv1_msg.pose.pose.position.x
            dy = target_point.pose.pose.position.y - ugv1_msg.pose.pose.position.y
            delta = math.atan2(dy, dx)
            theta = (delta - gamma + PI) % (2 * PI) - PI

            r = follower_distance
            k = 0.6
            if r > initial_distance * 2:
                msg.drive.speed = 0.2 * r 
            else:
                msg.drive.speed = 0
                #rospy.loginfo("too close and stop")
            msg.drive.steering_angle = k * theta
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            #rospy.loginfo("normal following")
        rate.sleep()

if __name__ == '__main__':
    zensha = sys.argv[1]  # Ç°³µÃû
    kousha = sys.argv[2]  # ºó³µÃû
    frame_id = sys.argv[3]
    node_name = sys.argv[4]

    rospy.init_node(node_name)
    rospy.Subscriber(f"/{zensha}/base_pose_ground_truth", Odometry, pose_callback0, queue_size=10)
    rospy.Subscriber(f"/{kousha}/base_pose_ground_truth", Odometry, pose_callback1, queue_size=10)

    event.set()

    t1 = threading.Thread(target=pub_cmd, args=(frame_id,))
    t2 = threading.Thread(target=follow, args=(frame_id, kousha))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
