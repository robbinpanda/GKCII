#! /usr/bin/env python3
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
#import message_filters
import apriltag

blur_ksize = 5  # Gaussian blur kernel size
canny_lthreshold = 0  # Canny edge detection low threshold
canny_hthreshold = 50  # Canny edge detection high threshold

# Hough transform parameters
rho = 1
theta = numpy.pi / 180
threshold = 15
min_line_length = 40
max_line_gap = 20


def set_roi_forward(img, position):
    if position == "left":
        l = numpy.copy(img)
        l[0:l.shape[0], int(l.shape[1]/2):l.shape[1]] = 0
        return l
    elif position == "right":
        r = numpy.copy(img)
        r[0:r.shape[0], 0:int(r.shape[1]/2)] = 0
        return r

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
    
def process_an_image(img):
    roi_vtx = numpy.array([[(0, int(img.shape[0]/3)), (img.shape[1], int(img.shape[0]/3)), (img.shape[1], int(img.shape[0]*3/4)), (0, int(img.shape[0]*3/4))]])
    #roi_vtx = numpy.array([[(0, img.shape[0]), (img.shape[1], img.shape[0]), (img.shape[1], 0), (0, 0)]])
    
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    #cv2.imshow("gray", gray)
    blur_gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0, 0)
    #cv2.imshow("blur_gray", blur_gray)
    edges = cv2.Canny(img, canny_lthreshold, canny_hthreshold)
    roi_edges = roi_mask(edges, roi_vtx)
    cv2.imshow("roi_edges", roi_edges)
    
    left = set_roi_forward(roi_edges, "left")
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
        cv2.circle(img, (cx_right, cy_right), 10, (0, 0, 255), -1)
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
        
    #line_img = hough_lines(right, rho, theta, threshold, min_line_length, max_line_gap)
    #cv2.imshow("line_img", line_img)
    #res_img = cv2.addWeighted(img, 0.8, line_img, 1, 0)
    #cv2.imshow("res_img", res_img)

    #return res_img

def process_right_side_image(img):
    roi_vtx = numpy.array([[(0, int(img.shape[0]/3)), (img.shape[1], int(img.shape[0]/3)), (img.shape[1], int(img.shape[0]*3/4)), (0, int(img.shape[0]*3/4))]])
    blur_gray = cv2.GaussianBlur(img, (blur_ksize, blur_ksize), 0, 0)
    cv2.imshow("blur_gray", blur_gray)
    edges = cv2.Canny(img, canny_lthreshold, canny_hthreshold)
    edges = roi_mask(edges, roi_vtx)
    cv2.imshow("edges", edges)
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #左侧白线
    lower_white = numpy.array([0, 0, 200])
    upper_white = numpy.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    mask_white = roi_mask(mask_white, roi_vtx)
    cv2.imshow("mask_white", mask_white)
    #右侧黑线
    lower_black = numpy.array([0, 0, 0])
    upper_black = numpy.array([30, 30, 30])
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask_black = roi_mask(mask_black, roi_vtx)
    cv2.imshow("mask_black", mask_black)
    
    left = set_roi_forward(mask_white, "left")
    cv2.imshow("left", left)
    M_left = cv2.moments(left)
    if M_left['m00'] > 0:
        cx_left = int(M_left['m10'] / M_left['m00']) #质心x坐标
        cy_left = int(M_left['m01'] / M_left['m00']) #质心y坐标
        cv2.circle(img, (cx_left, cy_left), 10, (0, 0, 255), -1)
        
    right = set_roi_forward(mask_black, "right")
    cv2.imshow("right", right)
    M_right = cv2.moments(right)
    if M_right['m00'] > 0:
        cx_right = int(M_right['m10'] / M_right['m00']) #质心x坐标
        cy_right = int(M_right['m01'] / M_right['m00']) #质心y坐标
        cv2.circle(img, (cx_right, cy_right), 10, (0, 0, 255), -1)
        
        
        
def right_side(image):
    #process_an_image(image)
    
    process_right_side_image(image)
    
    
    
    
    
    #扫描进入车道时的二维码，检测到后切换为巡线
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("uav0 gray camera", gray)
    if detector.detect(gray):
        tags = detector.detect(gray)
        for tag in tags:
            cv2.circle(image, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
            cv2.circle(image, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
            cv2.circle(image, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
            cv2.circle(image, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
            print("tag id: ", tag.tag_id)
    cv2.imshow("image", image)
    cv2.waitKey(1)

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    right_side(frame)
    pass

if __name__ == '__main__':
    detector = apriltag.Detector() #二维码识别器
    rospy.init_node("ugv0_test")
    rospy.Subscriber("/rover0/fpv_cam3/image_raw", Image, image_callback)
    rospy.spin()
    pass
