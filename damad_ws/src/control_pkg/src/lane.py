#! /usr/bin/env python3
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import pyapriltags
#import apriltag
import copy
    
def set_roi_forward(h, w, mask, position):
    if position == "left":
        mask[0:int(h / 3), 0:w] = 0
        mask[0:h, int(w / 4):w] = 0
    if position == "right":
        mask[0:int(h / 3), 0:w] = 0
        mask[0:h, 0:int(w / 4)] = 0
    return mask

def straight(image):
    print("直行通过十字路口")
    global pub, msg
    msg.drive.speed = 0.2
    msg.drive.steering_angle = 0
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)
    
def turn_left(image):
    print("左转通过十字路口")
    global pub, msg
    msg.drive.speed = 0.1
    msg.drive.steering_angle = 0.18
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)
    
def turn_right(image):
    print("右转通过十字路口")
    global pub, msg
    msg.drive.speed = 0.1
    msg.drive.steering_angle = -0.28
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)
    
def lane(image):
    global into_crossroads, signal, msg, pub
    
    """if into_crossroads:
        if signal == -1:
            msg.drive.speed = 0
            msg.drive.steering_angle = 0
            pub.publish(msg)
        elif signal == 0:
            turn_left(image)
        elif signal == 1:
            straight(image)
        elif signal == 2:
            turn_right(image)
        return"""

    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, w, _ = image.shape
    
    
    #cv2.imshow("hsv", hsv)
    lower_white = numpy.array([0, 0, 100])
    upper_white = numpy.array([0, 0, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    #cv2.imshow("mask", mask)
    roi_left = set_roi_forward(h, w, copy.deepcopy(mask), "left")
    #cv2.imshow("roi_left", roi_left)
    roi_right = set_roi_forward(h, w, copy.deepcopy(mask), "right")
    #cv2.imshow("roi_right", roi_right)
    
    #左线质心
    M_left = cv2.moments(roi_left)
    cx_left = 0
    cy_left = h-1
    if M_left['m00'] > 0:
        cx_left = int(M_left['m10'] / M_left['m00']) #质心x坐标
        cy_left = int(M_left['m01'] / M_left['m00']) #质心y坐标
    cv2.circle(image, (cx_left, cy_left), 10, (0, 0, 255), -1)
    #右线质心
    M_right = cv2.moments(roi_right)
    cx_right = w-1
    cy_right = h-1
    if M_right['m00'] > 0:
        cx_right = int(M_right['m10'] / M_right['m00']) #质心x坐标
        cy_right = int(M_right['m01'] / M_right['m00']) #质心y坐标
    cv2.circle(image, (cx_right, cy_right), 10, (0, 0, 255), -1)
    
    
    msg.drive.speed = 0.08
    msg.drive.steering_angle = -float((cx_left + cx_right - w + 50) / 100)
    msg.header.stamp = rospy.Time.now()

    pub.publish(msg)
    print(msg.drive.speed, msg.drive.steering_angle)
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)

def detect_tag(image):
    global into_crossroads, signal, detector
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if detector.detect(gray):
        tags = detector.detect(gray)
        for tag in tags:
            cv2.circle(image, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
            cv2.circle(image, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
            cv2.circle(image, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
            cv2.circle(image, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
            print("tag id: ", tag.tag_id)
            if tag.tag_id == 0:
                into_crossroads = True
                print("检测到进入十字路口二维码")
                return
            if tag.tag_id == 1:
                into_crossroads = False
                print("检测到进入车道二维码")
                return

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    #detect_tag(frame) 
    lane(frame)
    pass
    
def traffic_light_callback(msg):
    global signal
    signal = msg.data
    #print("Current Traffic Light is ", msg.data)

if __name__ == '__main__':
    rospy.init_node("ugv0")

    #新建Ackermann消息
    msg = AckermannDriveStamped()
    msg.header.frame_id = "base_link0"
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle_velocity = 1
    
    detector = pyapriltags.Detector() #二维码识别器

    into_crossroads = False #进入十字路口
    
    signal = "" #交通灯信号
    
    pub = rospy.Publisher("ugv0/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
    rospy.Subscriber("/rover0/fpv_cam3/image_raw", Image, image_callback)
    rospy.Subscriber("traffic_light", String, traffic_light_callback)
    
    rospy.spin()
    pass
