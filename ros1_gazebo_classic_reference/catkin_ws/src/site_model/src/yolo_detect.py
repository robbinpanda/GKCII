#!/usr/bin/env python3
# 指定解释器为 Python3，确保脚本可直接在终端运行

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

# --------------------------
# 初始化 YOLO 模型
# --------------------------
# 指定模型权重路径（需确认路径与权重文件存在）
weight_path = "/home/ecs-user/catkin_ws/yolo11n.pt"
# 加载 YOLOv11 模型
model = YOLO(weight_path)


# --------------------------
# 图像回调函数
# --------------------------
def image_callback(msg):
    # 创建 CvBridge 对象，用于ROS图像与OpenCV图像之间转换
    bridge = cv_bridge.CvBridge()

    # 将ROS图像消息转换为OpenCV格式（BGR8编码）
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')

    # 将图像送入YOLO模型进行目标检测
    results = model(frame)  # 返回一个包含预测结果的列表（通常只有一个元素）

    # 获取第一帧检测结果
    result = results[0]
    boxes = result.boxes  # 获取所有检测框（bounding boxes）

    # 如果检测到了目标
    if boxes is not None:
        for box in boxes:
            # 提取每个目标框的左上角与右下角坐标（xyxy格式），并转换为整数
            xyxy = box.xyxy[0].cpu().numpy().astype(int)

            # 提取置信度（confidence）和类别编号（class id）
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())

            # 根据类别编号获取类别名称，并拼接置信度，形成标签文本
            label = f"{model.names[cls]} {conf:.2f}"

            # 在图像上绘制矩形框
            cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
            # 在矩形框上方绘制标签文本
            cv2.putText(frame, label, (xyxy[0], xyxy[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 显示结果图像（弹出窗口），支持调整窗口大小
    cv2.namedWindow("camera", 0)
    cv2.imshow("camera", frame)
    cv2.waitKey(1)  # 等待1毫秒，确保图像窗口更新


# --------------------------
# 图像话题订阅函数
# --------------------------
def get_camera():
    # 订阅相机图像话题，注册回调函数 image_callback
    rospy.Subscriber(
        "/car1/car1/camera/zed_left/image_rect_color_left",
        Image,
        image_callback
    )


# --------------------------
# 主程序入口
# --------------------------
if __name__ == '__main__':
    # 初始化ROS节点，名称为 "get_camera_yolo"
    rospy.init_node("get_camera_yolo")

    # 开始图像话题订阅
    get_camera()

    # 保持节点运行，防止退出
    rospy.spin()
