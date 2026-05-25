#! /usr/bin/env python3
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import apriltag
import copy
    
def set_roi_forward(h, w, mask, position):
    if position == "left":
        mask[0:int(h / 3), 0:w] = 0
        mask[0:h, int(w / 4):w] = 0
    if position == "right":
        mask[0:int(h / 3), 0:w] = 0
        mask[0:h, 0:int(w / 4)] = 0
    return mask

blur_ksize = 5  # Gaussian blur kernel size
canny_lthreshold = 0  # Canny edge detection low threshold
canny_hthreshold = 50  # Canny edge detection high threshold

# Hough transform parameters
rho = 1
theta = numpy.pi / 180
threshold = 15
min_line_length = 40
max_line_gap = 20

def roi_mask(img, vertices):
    mask = numpy.zeros_like(img)

    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        mask_color = (255,) * channel_count
    else:
        mask_color = 255
    cv2.fillPoly(mask, vertices, mask_color)
    masked_img = cv2.bitwise_and(img, mask)
    return masked_img
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, numpy.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = numpy.zeros((img.shape[0], img.shape[1], 3), dtype=numpy.uint8)
    
    if lines is not None:
        draw_lanes(line_img, lines)
    return line_img

def draw_lanes(img, lines, color=[0, 0, 255], thickness=8):
    left_lines, right_lines = [], []
    for line in lines:
        for x1, y1, x2, y2 in line:
            k = (y2 - y1) / (x2 - x1)
            if abs(k) < 0.3: #斜率太小默认为干扰物体
                continue
            if k < 0:
                left_lines.append(line)
            else:
                right_lines.append(line)

    #print("left_lines:\n",left_lines)
    #print("right_lines:\n",right_lines)
    if (len(left_lines) <= 0 and len(right_lines) <= 0):
        return img

    clean_lines(left_lines, 0.1)
    clean_lines(right_lines, 0.1)
    left_points = [(x1, y1) for line in left_lines for x1,y1,x2,y2 in line]
    left_points = left_points + [(x2, y2) for line in left_lines for x1,y1,x2,y2 in line]
    right_points = [(x1, y1) for line in right_lines for x1,y1,x2,y2 in line]
    right_points = right_points + [(x2, y2) for line in right_lines for x1,y1,x2,y2 in line]

    #left_vtx = calc_lane_vertices(left_points, 325, img.shape[0])
    #right_vtx = calc_lane_vertices(right_points, 325, img.shape[0])
    if len(left_points) > 0:
        left_vtx = calc_lane_vertices(left_points, int(img.shape[0] * 2 / 3), img.shape[0])
        cv2.line(img, left_vtx[0], left_vtx[1], color, thickness)
    if len(right_points) > 0:
        right_vtx = calc_lane_vertices(right_points, int(img.shape[0] * 2 / 3), img.shape[0])
        cv2.line(img, right_vtx[0], right_vtx[1], color, thickness)

    #print("left_vtx:\n",left_vtx)
    #print("right_vtx:\n",right_vtx)
    #cv2.line(img, left_vtx[0], left_vtx[1], color, thickness)
    #cv2.line(img, right_vtx[0], right_vtx[1], color, thickness)
    #print("egaku")

def clean_lines(lines, threshold):
    slope = [(y2 - y1) / (x2 - x1) for line in lines for x1, y1, x2, y2 in line]
    while len(lines) > 0:
        mean = numpy.mean(slope)
        diff = [abs(s - mean) for s in slope]
        idx = numpy.argmax(diff)
        if diff[idx] > threshold:
            slope.pop(idx)
            lines.pop(idx)
        else:
            break

def calc_lane_vertices(point_list, ymin, ymax):
    x = [p[0] for p in point_list]
    y = [p[1] for p in point_list]
    fit = numpy.polyfit(y, x, 1)
    fit_fn = numpy.poly1d(fit)

    xmin = int(fit_fn(ymin))
    xmax = int(fit_fn(ymax))

    return [(xmin, ymin), (xmax, ymax)]
    
def lane_1(img):
    roi_vtx = numpy.array([[(0, int(img.shape[0]/3)), (img.shape[1], int(img.shape[0]/3)), (img.shape[1], int(img.shape[0]*3/4)), (0, int(img.shape[0]*3/4))]])
    #roi_vtx = numpy.array([[(0, img.shape[0]), (img.shape[1], img.shape[0]), (img.shape[1], 0), (0, 0)]])
    
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    cv2.imshow("gray", gray)
    blur_gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0, 0)
    cv2.imshow("blur_gray", blur_gray)
    edges = cv2.Canny(img, canny_lthreshold, canny_hthreshold)
    cv2.imshow("edges", edges)
    roi_edges = roi_mask(edges, roi_vtx)
    cv2.imshow("roi_edges", roi_edges)
    
    """left = set_roi_forward(roi_edges, "left")
    cv2.imshow("left1", left)
    M_left = cv2.moments(left)
    if M_left['m00'] > 0:
        cx_left = int(M_left['m10'] / M_left['m00']) #质心x坐标
        cy_left = int(M_left['m01'] / M_left['m00']) #质心y坐标
        cv2.circle(img, (cx_left, cy_left), 10, (0, 0, 255), -1)
        
    right = set_roi_forward(roi_edges, "right")
    cv2.imshow("right1", right)
    M_right = cv2.moments(right)
    if M_right['m00'] > 0:
        cx_right = int(M_right['m10'] / M_right['m00']) #质心x坐标
        cy_right = int(M_right['m01'] / M_right['m00']) #质心y坐标
        cv2.circle(img, (cx_right, cy_right), 10, (0, 0, 255), -1)"""
    """#寻找图像右下角跟白线质心在同一y轴下的点
    pixel_number_count = 0
    pixel_x_count = 0
    for i in range(int(img.shape[1] / 2), img.shape[1]):
        if mask_black[cy_left][i] == 255:
            #print(i)
            pixel_x_count += i
            pixel_number_count += 1
    cx_black = int(pixel_x_count / pixel_number_count)
    cv2.circle(image, (cx_black, cy_white), 10, (0, 0, 255), -1)"""
        
    line_img = hough_lines(roi_edges, rho, theta, threshold, min_line_length, max_line_gap)
    cv2.imshow("line_img", line_img)
    res_img = cv2.addWeighted(img, 0.8, line_img, 1, 0)
    cv2.imshow("res_img", res_img)

    #return res_img
    
def lane_0(image):
    global into_crossroads, signal, msg, pub
    
    if into_crossroads:
        if signal == 0:
            msg.drive.speed = 0
            msg.drive.steering_angle = 0
            pub.publish(msg)
        elif signal == 1:
            turn_left(image)
        elif signal == 2:
            straight(image)
        elif signal == 3:
            turn_right(image)
        return

    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, w, _ = image.shape
    
    
    #cv2.imshow("hsv", hsv)
    lower_white = numpy.array([0, 0, 100])
    upper_white = numpy.array([0, 0, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    cv2.imshow("mask", mask)
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
    
    
    msg.drive.speed = 0.2
    msg.drive.steering_angle = -float((cx_left + cx_right - w + 20) / 100)
    msg.header.stamp = rospy.Time.now()

    pub.publish(msg)
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
    detect_tag(frame) 
    lane_0(copy.deepcopy(frame))
    lane_1(copy.deepcopy(frame))
    pass
    
def traffic_light_callback(msg):
    global signal
    signal = int(msg.data)
    print("Current Traffic Light is ", msg.data)

if __name__ == '__main__':
    rospy.init_node("ugv0")

    #新建Ackermann消息
    msg = AckermannDriveStamped()
    msg.header.frame_id = "base_link0"
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle_velocity = 1
    
    detector = apriltag.Detector() #二维码识别器

    into_crossroads = False #进入十字路口
    
    signal = -1 #交通灯信号
    
    pub = rospy.Publisher("ugv0/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
    rospy.Subscriber("/rover0/fpv_cam3/image_raw", Image, image_callback)
    rospy.Subscriber("traffic_light", String, traffic_light_callback)
    
    rospy.spin()
    pass
