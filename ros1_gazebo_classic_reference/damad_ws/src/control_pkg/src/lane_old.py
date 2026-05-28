#! /usr/bin/env python3
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
#import message_filters
import apriltag

"""def match_template(template, image):
    max = 0.0
    #摄像头图像与本地图片进行模板匹配，返回一个矩阵，每个像素值代表匹配程度
    res = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
    #获得矩阵中最大值及位置和最小值及位置
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    if max_val>max:
        max = max_val
    return max"""
    
def set_roi_forward(h, w, mask, position):
    if position == "left":
        search_top = int(h / 2)
        search_right = int(w / 3)
        mask[0:search_top, 0:w] = 0
        mask[0:h, search_right:w] = 0
    elif position == "right":
        search_top = int(h / 2)
        search_left = int(w * 2/ 3)
        mask[0:search_top, 0:w] = 0
        mask[0:h, 0:search_left] = 0
    return mask

def straight(image):
    print("直行通过十字路口")
    global arrive_at_zebra_crossing, signal, pub, detector, msg
    """value = match_template(into_lane_template, image)
    if value > 0.6:
        close_to_zebra_crossing = False
        arrive_at_zebra_crossing = False
    print("straight value: ", value)"""
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
            if tag.tag_id == 1:
                arrive_at_zebra_crossing = False
                print("检测到车道二维码")
                return

    msg.drive.speed = 0.2
    msg.drive.steering_angle = 0
    #print(msg)
    pub.publish(msg)
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)
    
def turn_left(image):
    print("左侧车道左转通过十字路口")
    global arrive_at_zebra_crossing, signal, pub, detector, msg
    """value = match_template(into_lane_template, image)
    if value > 0.6:
        close_to_zebra_crossing = False
        arrive_at_zebra_crossing = False
    print("straight value: ", value)"""
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
            if tag.tag_id == 1:
                arrive_at_zebra_crossing = False
                print("检测到车道二维码")
                return

    msg.drive.speed = 0.1
    msg.drive.steering_angle = 0.08
    #print(msg)
    pub.publish(msg)
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)
    
def turn_right(image):
    print("右侧车道右转通过十字路口")
    global arrive_at_zebra_crossing, signal, pub, detector, msg
    """value = match_template(into_lane_template, image)
    if value > 0.6:
        close_to_zebra_crossing = False
        arrive_at_zebra_crossing = False
    print("straight value: ", value)"""
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
            if tag.tag_id == 1:
                arrive_at_zebra_crossing = False
                print("检测到车道二维码")
                return

    msg.drive.speed = 0.1
    msg.drive.steering_angle = -0.18
    #print(msg)
    pub.publish(msg)
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)
    
#左侧车道
def left_side(image):
    global arrive_at_zebra_crossing, signal, signal_arriving_at_zebra_crossing, msg, pub, detector
    
    if arrive_at_zebra_crossing:
        if signal_arriving_at_zebra_crossing == 2:
            msg.drive.speed = 0
            msg.drive.steering_angle = 0
            pub.publish(msg)
        elif signal_arriving_at_zebra_crossing == 1:
            straight(image)
        elif signal_arriving_at_zebra_crossing == 0:
            turn_left(image)
        return
        
    #扫描靠近斑马线时的二维码，检测到后根据红绿灯信号行动
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
            if tag.tag_id == 0:
                arrive_at_zebra_crossing = True
                signal_arriving_at_zebra_crossing = signal
                print("检测到斑马线二维码")
                return
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, w, _ = image.shape
   
    #右侧白线
    lower_white = numpy.array([0, 0, 200])
    upper_white = numpy.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    cv2.imshow("mask_white0", mask_white)
    histogram_x = numpy.sum(mask_white[int(h / 2):, :], axis=0)
    lane_base = numpy.argmax(histogram_x)
    nwindows = 7
    window_height = int(h / nwindows)
    nonzero = mask_white.nonzero()
    nonzeroy = numpy.array(nonzero[0])
    nonzerox = numpy.array(nonzero[1])
    lane_current = lane_base
    margin = 100
    minpix = 25
    if nonzerox.shape[0] != 0:
        isLeft = numpy.mean(nonzerox) < w / 2
    lane_inds = []
    mask_white = numpy.zeros((h,w), dtype=numpy.uint8) #新建纯黑图
    for window in range(nwindows):
        win_y_low = h - (window + 1) * window_height
        win_y_high = h - window * window_height
        win_x_low = lane_current - margin
        win_x_high = lane_current + margin
        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
        lane_inds.append(good_inds)
        #绘制滑动窗口
        #mask_white = cv2.rectangle(mask_white, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 3)
        #mask_white = cv2.rectangle(mask_white, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 255, 255), 2)
        if len(good_inds) > minpix:
            lane_current = int(numpy.mean(nonzerox[good_inds]))  ####
        elif window >= 3:
            break
    lane_inds = numpy.concatenate(lane_inds)
    pixelX = nonzerox[lane_inds]
    pixelY = nonzeroy[lane_inds]
    try:
        a2, a1, a0 = numpy.polyfit(pixelY, pixelX, 2)
    except:
        print("no image! The lane is " + str(isLeft))
        adjustOrientation()
        return 0
    aveX = numpy.average(pixelX)
    frontDistance = numpy.argsort(pixelY)[int(len(pixelY) / 8)]
    aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]
    ploty = numpy.array(list(set(pixelY)))
    plotx = a2 * ploty ** 2 + a1 * ploty + a0
    num = 0
    for num in range(len(ploty) - 1):
        cv2.line(mask_white, (int(plotx[num]), int(ploty[num])), (int(plotx[num + 1]), int(ploty[num + 1])), (255, 255, 255), 8)
    #cv2.imshow("mask_white0.5", mask_white)
    mask_white = set_roi_forward(h, w, mask_white, "right")
    cv2.imshow("roi_white", mask_white)
    #普通方法找质心
    M_white = cv2.moments(mask_white)
    cx_white = w-1
    cy_white = h-1
    if M_white['m00'] > 0:
        cx_white = int(M_white['m10'] / M_white['m00']) #质心x坐标
        cy_white = int(M_white['m01'] / M_white['m00']) #质心y坐标
        cv2.circle(image, (cx_white, cy_white), 10, (0, 0, 255), -1)
    
    
    #左侧黄线
    lower_yellow = numpy.array([30, 100, 100])
    upper_yellow = numpy.array([60, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    cv2.imshow("mask_yellow0", mask_yellow)
    mask_yellow = set_roi_forward(h, w, mask_yellow, "left")
    cv2.imshow("roi_yellow", mask_yellow)
    #普通方法找质心
    M_yellow = cv2.moments(mask_yellow)
    cx_yellow = 0
    cy_yellow = h-1
    if M_yellow['m00'] > 0:
        cx_yellow = int(M_yellow['m10'] / M_yellow['m00']) #质心x坐标
        cy_yellow = int(M_yellow['m01'] / M_yellow['m00']) #质心y坐标
        cv2.circle(image, (cx_yellow, cy_yellow), 10, (0, 0, 255), -1)
    
    
    msg.drive.speed = 0.2
    msg.drive.steering_angle = -float((cx_yellow + cx_white - w) / 100)
        
    #print(msg)
    pub.publish(msg)
    print("左侧车道行驶")
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)
    
#右侧车道
def right_side(image):
    global arrive_at_zebra_crossing, signal, signal_arriving_at_zebra_crossing, msg, pub, detector
    
    if arrive_at_zebra_crossing:
        if signal_arriving_at_zebra_crossing == 0:
            msg.drive.speed = 0
            msg.drive.steering_angle = 0
            pub.publish(msg)
        elif signal_arriving_at_zebra_crossing == 1:
            straight(image)
        elif signal_arriving_at_zebra_crossing == 2:
            turn_right(image)
        return
        
    #扫描靠近斑马线时的二维码，检测到后根据红绿灯信号行动
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
            if tag.tag_id == 0:
                arrive_at_zebra_crossing = True
                signal_arriving_at_zebra_crossing = signal
                print("检测到斑马线二维码")
                return
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, w, _ = image.shape

    #左侧白线
    lower_white = numpy.array([0, 0, 200])
    upper_white = numpy.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    cv2.imshow("mask_white0", mask_white)

    histogram_x = numpy.sum(mask_white[int(h / 2):, :], axis=0)
    lane_base = numpy.argmax(histogram_x)
    nwindows = 7
    window_height = int(h / nwindows)
    nonzero = mask_white.nonzero()
    nonzeroy = numpy.array(nonzero[0])
    nonzerox = numpy.array(nonzero[1])
    lane_current = lane_base
    margin = 100
    minpix = 25
        
    if nonzerox.shape[0] != 0:
        isLeft = numpy.mean(nonzerox) < w / 2

    lane_inds = []
    mask_white = numpy.zeros((h,w), dtype=numpy.uint8) #新建纯黑图
    for window in range(nwindows):
        win_y_low = h - (window + 1) * window_height
        win_y_high = h - window * window_height
        win_x_low = lane_current - margin
        win_x_high = lane_current + margin
        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]

        lane_inds.append(good_inds)
        
        #绘制滑动窗口
        #mask_white = cv2.rectangle(mask_white, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 3)
        #mask_white = cv2.rectangle(mask_white, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 255, 255), 2)
        if len(good_inds) > minpix:
            lane_current = int(numpy.mean(nonzerox[good_inds]))  ####
        elif window >= 3:
            break

    lane_inds = numpy.concatenate(lane_inds)

    pixelX = nonzerox[lane_inds]
    pixelY = nonzeroy[lane_inds]
    try:
        a2, a1, a0 = numpy.polyfit(pixelY, pixelX, 2)
    except:
        print("no image! The lane is " + str(isLeft))
        adjustOrientation()
        return 0
        
    aveX = numpy.average(pixelX)

    frontDistance = numpy.argsort(pixelY)[int(len(pixelY) / 8)]
    aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]

    ploty = numpy.array(list(set(pixelY)))
    plotx = a2 * ploty ** 2 + a1 * ploty + a0
    num = 0
    for num in range(len(ploty) - 1):
        cv2.line(mask_white, (int(plotx[num]), int(ploty[num])), (int(plotx[num + 1]), int(ploty[num + 1])), (255, 255, 255), 8)
    #cv2.imshow("mask_white0.5", mask_white)
    
    mask_white = set_roi_forward(h, w, mask_white, "left")
    cv2.imshow("roi_white", mask_white)

    """#寻找图像左下角跟黑线质心在同一y轴下的点
    pixel_number_count = 0
    pixel_x_count = 0
    for i in range(int(w / 2), w):
        if mask_white[cy_black][i] == 255:
            print(i)
            pixel_x_count += i
            pixel_number_count += 1
    cx_white = int(pixel_x_count / pixel_number_count)
    cv2.circle(image, (cx_white, cy_black), 10, (0, 0, 255), -1)"""
    #普通方法找质心
    M_white = cv2.moments(mask_white)
    cx_white = 0
    cy_white = h-1
    if M_white['m00'] > 0:
        cx_white = int(M_white['m10'] / M_white['m00']) #质心x坐标
        cy_white = int(M_white['m01'] / M_white['m00']) #质心y坐标
        cv2.circle(image, (cx_white, cy_white), 10, (0, 0, 255), -1)
    
    
    #右侧黑线
    lower_black = numpy.array([0, 0, 0])
    upper_black = numpy.array([30, 30, 30])
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    cv2.imshow("mask_black0", mask_black)
    #h, w = mask_black.shape
    mask_black = set_roi_forward(h, w, mask_black, "right")
    cv2.imshow("roi_black", mask_black)
    """#寻找图像右下角跟白线质心在同一y轴下的点
    pixel_number_count = 0
    pixel_x_count = 0
    for i in range(int(w / 2), w):
        if mask_black[cy_white][i] == 255:
            #print(i)
            pixel_x_count += i
            pixel_number_count += 1
    cx_black = int(pixel_x_count / pixel_number_count)
    cv2.circle(image, (cx_black, cy_white), 10, (0, 0, 255), -1)"""
    #普通方法找质心
    M_black = cv2.moments(mask_black)
    cx_black = w-1
    cy_black = h-1
    if M_black['m00'] > 0:
        cx_black = int(M_black['m10'] / M_black['m00']) #质心x坐标
        cy_black = int(M_black['m01'] / M_black['m00']) #质心y坐标
        cv2.circle(image, (cx_black, cy_black), 10, (0, 0, 255), -1)
    
    
    msg.drive.speed = 0.2
    msg.drive.steering_angle = -float((cx_black + cx_white - w) / 100)
        
    #print(msg)
    pub.publish(msg)
    print("右侧车道行驶")
    cv2.imshow("uav0 front camera", image)
    cv2.waitKey(1)

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    right_side(frame)
    pass
    
def traffic_light_callback(msg):
    global signal
    signal = int(msg.data)
    print("Current Traffic Light is ", msg.data)
 
"""def callback(msg0, msg1):
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg0, 'bgr8')
    right_side(frame)
    print("Current Traffic Light is ", msg1.data)"""
if __name__ == '__main__':
    rospy.init_node("ugv0")

    #新建Ackermann消息
    msg = AckermannDriveStamped();
    #msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "base_link0";
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    msg.drive.steering_angle_velocity = 1
    
    detector = apriltag.Detector() #二维码识别器

    arrive_at_zebra_crossing = False #靠近斑马线
    signal_arriving_at_zebra_crossing = -1 #靠近斑马线时的交通灯信号
    
    
    signal = -1 #交通灯信号
    
    #读取本地图片
    #zebra_crossing_template = cv2.resize(cv2.imread(r"/home/shengzi/damad_ws/src/control_pkg/ZebraCrossing2.png"), (60,80))
    #into_lane_template = cv2.resize(cv2.imread(r"/home/shengzi/damad_ws/src/control_pkg/IntoLane.png"), (60,80))

    pub = rospy.Publisher("ugv0/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
    
    rospy.Subscriber("/rover0/fpv_cam0/image_raw", Image, image_callback)
    rospy.Subscriber("traffic_light", String, traffic_light_callback)
    
    """sub0 = message_filters.Subscriber("/rover0/fpv_cam0/image_raw", Image)
    sub1 = message_filters.Subscriber("traffic_light", String)
    ts = message_filters.ApproximateTimeSynchronizer([sub0, sub1], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)"""
    
    rospy.spin()
    pass
