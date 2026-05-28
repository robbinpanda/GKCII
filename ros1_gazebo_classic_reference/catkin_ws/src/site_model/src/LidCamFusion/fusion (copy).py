#!/usr/bin/env python3

#############################################################
#   此Python文件获取激光雷达和相机信息（时间同步）并进行融合。 #
#############################################################

from pathlib import Path
import argparse
import yaml
import rospy
from termcolor import colored
import message_filters
import ros_numpy
import os

from visualization_msgs.msg import MarkerArray  # 可视化消息类型
from PIL import Image
import cv2
from sensor_msgs.msg import PointCloud2  # 点云消息类型
from nav_msgs.msg import Odometry  # 里程计消息类型
from msgs.msg._MsgCamera import *  # 自定义相机消息类
from msgs.msg._MsgLidCam import *  # 融合消息类型
from msgs.msg._MsgLidCamObject import *  # 自定义激光雷达-相机对象消息类
from .OpenPCDet.tools.pred import *  # 3D检测工具
from ..utils.yolo.yolo import YOLO  # 视觉检测工具
from ..utils.poi_and_roi import pointcloud_roi, image_roi  # ROI检测工具
from ..utils.visualization import lidar_camera_match2visual, display_rviz  # 可视化工具
from ..utils.evaluation import eval3d  # 评估工具
from ..utils.common_utils import get_gt_boxes3d, label2camera, label2camera_  # 公共工具
from ..utils.transform import lidar2pixel, box_to_corner_3d, get_dpm, p2w, world2pixel  # 转换工具

def fusion(pointcloud=None, msgcamera=None, odom=None, counter=None):
    """
    融合激光雷达和相机数据。
    :param pointcloud: 点云数据[N, 4]
    :param msgcamera: 相机图像数据[1, 8] -> 八张图像
    :param odom: 里程计数据
    :param counter: 当前计数器，用于评估过程
    """
    global start_time

    if params.eval:
        # 在评估过程中，加载预先存储的数据集
        points = np.loadtxt(os.path.join(str(ROOT_DIR / 'dataset' / 'test_dataset' / 'pcd'), '{}.txt'.format(counter)))
        images = [Image.fromarray(cv2.imread(os.path.join(str(ROOT_DIR / 'dataset' / 'test_dataset' / 'img'), '{}'.format(counter), '{}.jpg'.format(num)))) for num in range(8)]
        odom = odoms[counter]
    else:
        # 确保点云和相机消息的类型正确
        assert isinstance(pointcloud, PointCloud2)
        assert isinstance(msgcamera, MsgCamera)
        pub_lidcam = rospy.Publisher("/lidar_camera_fused", MsgLidCam, queue_size=1)  # 创建发布者
        points = convert_ros_pointcloud_to_numpy(pointcloud)  # 转换点云为NumPy格式
        images = [img for img in msgcamera.camera]  # 获取图像列表
        
    # 点云ROI检测
    pred_boxes3d, pred_labels, pred_scores = pointcloud_detector.get_pred_dicts(points, False)
    cameras, pred_corners3d, pixel_poses = pointcloud_roi(calib, pred_boxes3d)  # 获取摄像头信息
    # 图像ROI检测
    pred_boxes2d = [image_roi(img, yolo) for img in images]
    gt_boxes3d = gt_cameras = gt_corners3d = gt_pixel_poses = None
    if params.savem or params.re:
        # 获取真实框
        assert isinstance(odom, (Odometry, np.ndarray)), f'odom消息应为{Odometry}或{np.ndarray}但得到{type(odom)}'
        gt_boxes3d = get_gt_boxes3d(odom)  # 获取真实的3D边界框
        gt_cameras, gt_corners3d, gt_pixel_poses = pointcloud_roi(calib, gt_boxes3d)  # 获取真实的摄像头信息
        
    # 匹配
    iou_thresh = config['lid_cam_fusion']['iou_thresh']  # IOU阈值
    match, image, lidar, vehicles = get_match(cameras, pixel_poses, pred_boxes2d, iou_thresh)  # 匹配车辆
    # 融合
    updated_boxes3d, updated_corners3d, updated_pixel_poses = get_fusion(match, pred_boxes2d, pred_boxes3d, pred_corners3d, pred_scores, pixel_poses)

    # 生成融合消息并发布结果
    if not params.eval:
        msglidcam = get_msgldcam(match, updated_boxes3d, image, lidar)  # 创建融合消息
        pub_lidcam.publish(msglidcam)  # 发布消息
        fps = get_fps()  # 获取FPS

    # TODO: 支持多评估
    # 评估
    if params.eval or params.re:
        eval.eval(odom, pred_boxes3d, updated_boxes3d, pred_scores)  # 进行评估
    
    if params.disp:  # 在rviz中显示3D框
        marker_array = display_rviz(pred_corners3d, vehicles, gt_corners3d)
        pub_marker = rospy.Publisher('/display_rviz', MarkerArray, queue_size=1)
        pub_marker.publish(marker_array)
    if params.printl:  # 输出激光雷达结果到屏幕
        print2screen_lidar(pred_boxes3d, pred_labels, pred_scores)
    if params.printm:  # 输出匹配结果到屏幕
        print2screen_match(match, image, lidar)
    if params.savem:  # 保存匹配结果
        output_dir = str(ROOT_DIR / config['output']['LidCamFusion_dir'])
        lidar_camera_match2visual(match, image, lidar, pred_boxes2d, pixel_poses, msgcamera, output_dir, gt_cameras, gt_pixel_poses)

def get_fps():
    # FPS评估（不包括结果评估和可视化）
    global start_time
    cur_time = time.time()
    time_span = cur_time - start_time
    start_time = cur_time
    fps = 1 / time_span
    return fps

def get_match(cameras, pixel_poses, boxes2d, iou_thresh):
    """
    匹配激光雷达和相机检测到的物体。
    :param cameras: 摄像头列表
    :param pixel_poses: 像素位置
    :param boxes2d: 2D边界框
    :param iou_thresh: IOU阈值
    :return: 匹配结果和未匹配的对象
    """
    match = []  # 匹配结果
    vehicles = []  # 匹配的车辆
    labels_set = []  # 2D框的标签
    image = []  # 图像
    lidar = []  # 激光雷达数据

    # 为2D框添加标签：0->未匹配，1->已匹配
    for camera in range(len(boxes2d)):
        labels = [0] * len(boxes2d[camera])  # 初始标签
        labels_set.append(labels)

    # 匹配激光雷达和相机
    for vehicle, pixel_pose in enumerate(pixel_poses):  # 遍历每个车辆
        for i, camera in enumerate(cameras[vehicle]):  # 考虑第一个摄像头
            box2d = boxes2d[camera - 1]  # 获取该摄像头的2D框
            labels = labels_set[camera - 1]  # 获取标签
            bbox = get_bbox_from_box3d(pixel_pose[i])  # 将3D框转换为2D框
            if len(box2d) != 0:
                iou2ds = get_iou2d(bbox, box2d, labels, iou_thresh)  # 计算IOU
                if len(np.where(iou2ds != -1)[0]) != 0:  # 存在匹配框
                    idx = np.where(iou2ds == np.max(iou2ds))  # 获取最大IOU的索引
                    labels_set[camera - 1][idx[0][0]] = 1  # 标记为已匹配
                    if vehicle not in vehicles:  # 当前车辆未匹配
                        vehicles.append(vehicle)  # 标记车辆
                        cur_match = [camera, vehicle, i, idx[0][0], box2d[idx[0][0]][4]]  # 记录匹配信息
                        match.append(cur_match)
    
    # 添加未匹配的检测结果
    for camera in range(len(boxes2d)):
        cur_image = [camera + 1]
        for box2d, label in zip(boxes2d[camera], labels_set[camera]):
            if not label:  # 如果未匹配
                cur_image.append(box2d)
        image.append(cur_image)
    
    # 添加激光雷达未匹配的检测结果
    for vehicle in range(len(pixel_poses)):
        if vehicle not in vehicles:
            lidar.append([cameras[vehicle], vehicle])

    return match, image, lidar, vehicles

def get_bbox_from_box3d(pixel_pose):
    W = config['camera']['width']  # 图像宽度
    H = config['camera']['height']  # 图像高度

    xaxis = np.array(pixel_pose)[:, 0]
    yaxis = np.array(pixel_pose)[:, 1]
    x_max = np.max(xaxis) if np.max(xaxis) <= W else W  # X轴最大值
    x_min = np.min(xaxis) if np.min(xaxis) >= 0 else 0  # X轴最小值
    y_max = np.max(yaxis) if np.max(yaxis) <= H else H  # Y轴最大值
    y_min = np.min(yaxis) if np.min(yaxis) >= 0 else 0  # Y轴最小值

    return np.array([x_min, y_min, x_max, y_max])

def get_iou2d(boxa, boxesb, labels, iou_thresh):
    """
    计算2D框的IOU。
    :param boxa: 激光雷达检测的框
    :param boxesb: 相机检测的框
    """
    W = config['camera']['width']
    H = config['camera']['height']

    def get_single_iou2d(boxa, boxb):
        # 计算两个框的IOU
        x1 = max(boxa[0], boxb[0], 0)
        y1 = max(boxa[1], boxb[1], 0)
        x2 = min(boxa[2], boxb[2], W)
        y2 = min(boxa[3], boxb[3], H)
        areaa = (boxa[2] - boxa[0]) * (boxa[3] - boxa[1])  # boxa的面积
        areab = (boxb[2] - boxb[0]) * (boxb[3] - boxb[1])  # boxb的面积
        overlap = (x2 - x1) * (y2 - y1)  # 重叠面积
        iou2d = overlap / (areaa + areab - overlap)  # 计算IOU
        return iou2d

    iou2ds = []
    for boxb, label in zip(boxesb, labels):
        if not label:  # 当前框未匹配
            iou2d = get_single_iou2d(boxa, boxb)
            if iou2d < iou_thresh:  # 如果IOU过小，则不匹配
                iou2d = -1
        else:  # 如果boxb之前已匹配，则不匹配
            iou2d = -1
        iou2ds.append(iou2d)

    return np.array(iou2ds)

def get_fusion(match, boxes2d, boxes3d_origin, corners3d_origin, scores3d, pixels_poses_origin):
    """
    更新3D框、角点和像素位置。
    :param match: 匹配结果
    :param boxes2d: 2D边界框
    :param boxes3d_origin: 原始3D边界框
    :param corners3d_origin: 原始3D角点
    :param scores3d: 3D检测的分数
    :param pixels_poses_origin: 原始像素位置
    """
    import copy
    boxes3d = copy.deepcopy(boxes3d_origin)
    corners3d = copy.deepcopy(corners3d_origin)
    pixels_poses = copy.deepcopy(pixels_poses_origin)
    
    xcameras = np.array([-2, 4, 6, -8])  # 摄像头X坐标
    ycameras = np.array([1, -3, -5, 7])  # 摄像头Y坐标
    
    for obj in match:
        camera_num, vehicle_num, camera_num_vehicle, box2d_num, score2d = obj
        box2d = boxes2d[camera_num - 1][box2d_num]  # 获取2D框
        box3d, corner3d, score3d, pixel_pose = boxes3d[vehicle_num], corners3d[vehicle_num], scores3d[vehicle_num], pixels_poses[vehicle_num][camera_num_vehicle]

        # 检测框被截断
        if is_truncated(box2d, pixel_pose):
            continue

        CAMERA = config['lid_cam_fusion']['camera_weight']  # 摄像头权重
        LIDAR = config['lid_cam_fusion']['lidar_weight']  # 激光雷达权重
        assert (CAMERA + LIDAR == 1), '权重之和应该等于1。'
        
        # 激光雷达修正向量
        lidar_increment = np.array([0] * 7).astype(np.float64)

        # 水平修正
        lidar_center = np.array([np.mean([pixel_pose[0][0], pixel_pose[1][0], pixel_pose[2][0], pixel_pose[3][0]]),
                                 np.mean([pixel_pose[0][1], pixel_pose[3][1], pixel_pose[4][1], pixel_pose[7][1]])])
        camera_center = np.array([np.mean([box2d[0], box2d[2]]), np.mean([box2d[1], box2d[3]])])

        horizontal_diff = (lidar_center[0] - camera_center[0]) * CAMERA
        if camera_num in abs(xcameras):
            dpm = get_dpm(calib, camera_num, box3d[0:2], 0)
            lidar_increment[1] += np.sign(xcameras[np.where(abs(xcameras) == camera_num)[0][0]]) * dpm * horizontal_diff
        elif camera_num in abs(ycameras):
            dpm = get_dpm(calib, camera_num, box3d[0:2], 1)
            lidar_increment[0] += np.sign(ycameras[np.where(abs(ycameras) == camera_num)[0][0]]) * dpm * horizontal_diff
        box3d += lidar_increment  # 更新3D框
        corner3d[:, 0:2] += lidar_increment[0:2]  # 更新角点
        
        # 旋转修正
        pixel_pose = lidar2pixel(calib, label2camera[camera_num], corner3d)  # 将激光雷达点云转换为像素位置
        bbox = get_bbox_from_box3d(pixel_pose)  # 获取像素框
        lidar_ratio = bbox[0] / bbox[1]  # 激光雷达框比例
        camera_ratio = box2d[0] / box2d[1]  # 摄像头框比例

        INCREMENT_RY = 0.01  # 旋转增量
        box3d[6] += INCREMENT_RY  # 旋转增量应用
        corner3d = box_to_corner_3d(np.array([box3d]))[0]
        box3d[6] -= INCREMENT_RY  # 恢复原值
        pixel_pose = lidar2pixel(calib, label2camera[camera_num], corner3d)  # 重新计算像素位置
        bbox = get_bbox_from_box3d(pixel_pose)  # 获取新的像素框
        lidar_ratio_new = bbox[0] / bbox[1]  # 新的激光雷达框比例
        anticlockwise = None
        if abs(lidar_ratio_new - camera_ratio) < abs(lidar_ratio - camera_ratio):
            anticlockwise = 1
        else:
            box3d[6] -= INCREMENT_RY
            corner3d = box_to_corner_3d(np.array([box3d]))[0]
            box3d[6] -= INCREMENT_RY
            pixel_pose = lidar2pixel(calib, label2camera[camera_num], corner3d)
            bbox = get_bbox_from_box3d(pixel_pose)
            lidar_ratio_new = bbox[0] / bbox[1]
            if abs(lidar_ratio_new - camera_ratio) < abs(lidar_ratio - camera_ratio):
                anticlockwise = -1

        if anticlockwise is not None:
            is_decrease = 1
            last_diff = 1e3
            while (abs(lidar_ratio_new - camera_ratio) > abs(lidar_ratio - camera_ratio) * LIDAR and is_decrease):
                lidar_increment[6] += anticlockwise * INCREMENT_RY  # 旋转增量
                box3d[6] += anticlockwise * INCREMENT_RY  # 更新3D框
                corner3d = box_to_corner_3d(np.array([box3d]))[0]  # 更新角点
                pixel_pose = lidar2pixel(calib, label2camera[camera_num], corner3d)
                bbox = get_bbox_from_box3d(pixel_pose)
                lidar_ratio_new = bbox[0] / bbox[1]
                if not abs(lidar_ratio_new - camera_ratio) < last_diff:  # 检查是否减小
                    is_decrease = 0
                last_diff = abs(lidar_ratio_new - camera_ratio)
    
    return boxes3d, corners3d, pixels_poses


def get_msgldcam(match, updated_boxes3d, image, lidar) -> MsgLidCam:
    msglidcam = MsgLidCam()
    msglidcam.header.stamp = rospy.Time.now()

    # 添加匹配的车辆
    for obj in match:
        vehicle_num = obj[1]  # 车辆编号
        box3d = updated_boxes3d[vehicle_num]  # 获取更新后的3D盒子
        
        msglidcamobject = MsgLidCamObject()
        msglidcamobject.pos_x = box3d[0]  # 车辆在X轴的坐标
        msglidcamobject.pos_y = box3d[1]  # 车辆在Y轴的坐标
        msglidcamobject.alpha = box3d[6]   # 车辆的朝向角度
        if msglidcamobject.pos_y >= 0:  # 如果车辆在道路上
            msglidcam.objects_intersection.append(msglidcamobject)
            msglidcam.num_intersection += 1  # 更新交叉口车辆数量
        else:  # 否则在圆形区域内
            msglidcam.objects_circle.append(msglidcamobject)
            msglidcam.num_circle += 1  # 更新圆形区域内车辆数量

    # 添加基于激光雷达的车辆
    for obj in lidar:
        vehicle_num = obj[1]  # 车辆编号
        box3d = updated_boxes3d[vehicle_num]  # 获取更新后的3D盒子

        msglidcamobject = MsgLidCamObject()
        msglidcamobject.pos_x = box3d[0]  # 车辆在X轴的坐标
        msglidcamobject.pos_y = box3d[1]  # 车辆在Y轴的坐标
        msglidcamobject.alpha = box3d[6]   # 车辆的朝向角度
        if msglidcamobject.pos_y >= 0:  # 如果车辆在道路上
            msglidcam.objects_intersection.append(msglidcamobject)
            msglidcam.num_intersection += 1  # 更新交叉口车辆数量
        else:  # 否则在圆形区域内
            msglidcam.objects_circle.append(msglidcamobject)
            msglidcam.num_circle += 1  # 更新圆形区域内车辆数量

    # 添加基于相机的车辆
    # TODO: measurement
    # for obj in image:
    #     if len(obj) != 1 and not is_truncated(obj[1]):
    #         box3d = get_boxes3d_from_boxes2d(calib, np.array(obj))
            
    #         msglidcamobject = MsgLidCamObject()
    #         msglidcamobject.pos_x = box3d[0]  # 车辆在X轴的坐标
    #         msglidcamobject.pos_y = box3d[1]  # 车辆在Y轴的坐标
    #         msglidcamobject.alpha = -1  # 暂未定义
    #         if msglidcamobject.pos_y >= 0:  # 如果车辆在道路上
    #             msglidcam.objects_intersection.append(msglidcamobject)
    #             msglidcam.num_intersection += 1  # 更新交叉口车辆数量
    #         else:  # 否则在圆形区域内
    #             msglidcam.objects_circle.append(msglidcamobject)
    #             msglidcam.num_circle += 1  # 更新圆形区域内车辆数量
    
    return msglidcam  # 返回包含车辆信息的MsgLidCam对象


def get_boxes3d_from_boxes2d(center, camera_num):
    """
    使用p2w函数
    根据相机检测生成的2D框中心获取3D坐标
    """
    w2c = np.array(geometry['cameras'][label2camera_[camera_num]]['w2c']).reshape(4,4)  # 世界坐标到相机坐标的转换矩阵
    c2p = np.array(geometry['cameras'][label2camera_[camera_num]]['c2p']).reshape(3,4)  # 相机坐标到点云坐标的转换矩阵
    pos_wld, _ = p2w(center, 0.1, w2c, c2p)  # 将2D中心转换为3D坐标

    return pos_wld  # 返回3D坐标


def is_truncated(box2d=None, pixel_pose=None):
    W = config['camera']['width']  # 相机图像宽度
    H = config['camera']['height']  # 相机图像高度

    if box2d is not None:
        if box2d[0] <= 0 or box2d[1] <= 0 or box2d[2] >= W or box2d[3] >= H:  # 如果边界超出图像范围
            return True
    if pixel_pose is not None:
        xaxis = np.array(pixel_pose)[:,0]
        yaxis = np.array(pixel_pose)[:,1]
        if np.min(xaxis) <= 0 or np.min(yaxis) <= 0 or np.max(xaxis) >= W or np.max(yaxis) >= H:  # 如果像素位置超出图像范围
            return True

    return False  # 返回未截断


def convert_ros_pointcloud_to_numpy(pointcloud: PointCloud2):
    pc = ros_numpy.numpify(pointcloud)  # 将ROS点云转换为NumPy数组
    points = np.zeros((pc.shape[0],4))  # 初始化4D数组
    points[:,0] = pc['x']  # X坐标
    points[:,1] = pc['y']  # Y坐标
    points[:,2] = pc['z']  # Z坐标

    return points  # 返回点云数组


def print2screen_lidar(pred_boxes3d, pred_labels, pred_scores):
    label2class = {1: 'Car', 2: 'Pedestrian', 3: 'Bicycle'}  # 标签到类别的映射
    print("+-------------------------------------------------------------------------------------------+")
    print("num_car: ", len(pred_boxes3d))  # 打印检测到的车辆数量
    for i in range(len(pred_boxes3d)):
        print(i+1, " ==> ", label2class[int(pred_labels[i])], "  score: ", pred_scores[i])  # 打印每个车辆的类别和置信度
        print("  ", pred_boxes3d[i][0:3], " ", pred_boxes3d[i][3:6], " ", pred_boxes3d[i][6])  # 打印3D盒子的坐标和角度
    print("+-------------------------------------------------------------------------------------------+\n")


def print2screen_match(match, image, lidar):
    """
    match: [相机编号, 车辆编号(激光雷达), 2D框编号(相机)]
    image: [[1,],[2,],...,[8,]]
    lidar: [[相机编号, 车辆编号]]
    """
    print("+-------------------------------------------------------------------------------------------+")
    print("match: ", match)  # 打印匹配结果
    print("image: ", image)  # 打印图像检测结果
    print("lidar: ", lidar)  # 打印激光雷达检测结果
    print("+-------------------------------------------------------------------------------------------+\n")


def eval_fusion():
    limit = 30000  # 评估的最大次数
    counter = 0
    while(counter != limit):
        fusion(counter=counter)  # 执行融合
        counter += 1
    import sys
    sys.exit(0)  # 结束程序


if __name__ == '__main__':
    # 获取根目录
    ROOT_DIR = Path('/home/ecs-user/catkin_ws/src/site_model')
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", help="配置文件路径", metavar="FILE", required=False, default= str(ROOT_DIR / 'config/config.yaml'))
    parser.add_argument("--geometry", help="几何文件路径", metavar="FILE", required=False, default= str(ROOT_DIR / 'config/geometry.yaml'))
    parser.add_argument("--eval", help="指定评估模式", action='store_true', required=False)
    parser.add_argument("--gt", help="是否计算gt_boxes", action='store_true', required=False)
    parser.add_argument("--re", help="在线评估运行", action='store_true', required=False)
    parser.add_argument("--disp", help="是否显示到rviz", action='store_true', required=False)
    parser.add_argument("--savem", help="是否保存匹配结果", action='store_true', required=False)
    parser.add_argument("--printl", help="是否将激光雷达结果打印到屏幕", action='store_true', required=False)
    parser.add_argument("--printm", help="是否将匹配结果打印到屏幕", action='store_true', required=False)
    params = parser.parse_args()
    with open(params.config, 'r') as f:
        try:
            config = yaml.load(f, Loader=yaml.FullLoader)  # 读取配置文件
        except:
            print(colored('配置文件无法读取。','red'))
            exit(1)
    with open(params.geometry, 'r') as z:
        try:
            geometry = yaml.load(z, Loader=yaml.FullLoader)  # 读取几何文件
        except:
            print(colored('几何文件无法读取。','red'))
            exit(1)

    pointcloud_detector = RT_Pred(ROOT_DIR, config)  # 创建点云检测器
    yolo = YOLO(ROOT_DIR)  # 创建YOLO检测器
    calib_dir = str(ROOT_DIR.joinpath(config['calib']['calib_dir']))  # 获取标定文件路径
    calib = np.loadtxt(os.path.join(calib_dir, 'calib.txt'))

    if params.eval or params.re:
        import os
        import datetime
        log_dir = str(ROOT_DIR)+'/src/LidCamFusion/eval/%s/' % datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        os.makedirs(log_dir, exist_ok=True)  # 创建日志目录
        eval = eval3d(log_dir)  # 创建评估器

    if params.eval:
        odoms = np.loadtxt(os.path.join(str(ROOT_DIR / 'dataset' / 'test_dataset' / 'odom'), '30000.txt'))
        eval_fusion()  # 执行评估融合
    else:
        rospy.init_node('lidar_camera_fusion', anonymous=True)  # 初始化ROS节点

        fps = 0
        sub_pointcloud = message_filters.Subscriber('/point_cloud_combined', PointCloud2)  # 订阅点云数据
        sub_camera = message_filters.Subscriber('/camera_msgs_combined', MsgCamera)  # 订阅相机数据
        
        if params.savem or params.re:
            # TODO: 定义车辆数量
            sub_odom = message_filters.Subscriber('/car1/base_pose_ground_truth', Odometry)  # 订阅里程计数据
            sync = message_filters.ApproximateTimeSynchronizer([sub_pointcloud, sub_camera, sub_odom], 1, 1)  # 同步时间戳
            sync.registerCallback(fusion)  # 注册融合回调
            print("激光雷达与相机融合（带gt_boxes）开始。")
            start_time = time.time()
            rospy.spin()  # 循环等待消息
        else:
            sync = message_filters.ApproximateTimeSynchronizer([sub_pointcloud, sub_camera], 1, 1)  # 同步时间戳
            sync.registerCallback(fusion)  # 注册融合回调
            print("激光雷达与相机融合开始。")
            start_time = time.time()
            rospy.spin()  # 循环等待消息
