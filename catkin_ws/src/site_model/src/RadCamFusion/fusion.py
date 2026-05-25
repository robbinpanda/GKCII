# !/usr/bin/env python3
"""
获取雷达和相机消息并进行雷达-相机融合。
"""

from time import perf_counter, sleep
from pathlib import Path
import argparse
import yaml
import numpy as np
from typing import List, Tuple

import cv2
from cv_bridge import CvBridge

import rospy
import message_filters
from sensor_msgs.msg import Image  # 相机消息
from msgs.msg._MsgRadarObject import MsgRadarObject
from msgs.msg._MsgRadar import MsgRadar  # 雷达消息
from msgs.msg._MsgRadCam import MsgRadCam  # 融合消息
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from ..utils.yolo.yolo import YOLO
from ..utils.kalman import Kalman
from ..utils.poi_and_roi import image_roi
from ..utils.visualization import VisualAssistant
from ..utils.sensor_and_obs import ObsBundle, RadarSensor, ImageSensor, SensorCluster

import warnings

warnings.filterwarnings("ignore")

time_counter = 0  # 时间计数器
frame_counter = 0  # 帧计数器
geometry = {}  # 存储几何信息的字典

CUDA = False  # 是否使用CUDA加速

# 路径变量
YOLO_DIR = Path()
OUTPUT_DIR = Path()
SAVE_DIR = Path()
LOAD_DIR = Path()
BASE_IMAGE_FILE = Path()

# 模型参数
MODEL_SIZE = 0
A = np.empty((0, 0))  # 状态转移矩阵
Q = np.empty((0, 0))  # 过程噪声协方差矩阵
MAX_AGE = 0  # 最大年龄
THRES_IOU = 0  # IOU阈值
THRES_SCENE = 0  # 场景阈值


def my_timer() -> float:
    """
    计时器函数，用于计算并输出当前帧的FPS。
    """
    global time_counter, frame_counter
    print('+------------------------+')
    frame_counter += 1
    my_now = perf_counter()
    del_time = my_now - time_counter  # 计算时间差
    print("\033[0;36mFrame {}, FPS: {:.2f}\033[0m".format(frame_counter, 1.0 / del_time))
    time_counter = my_now
    return del_time


def my_file_loader() -> None:
    """
    加载配置文件和几何文件，初始化全局变量。
    """
    global YOLO_DIR, OUTPUT_DIR, BASE_IMAGE_FILE, SAVE_DIR, LOAD_DIR
    global CUDA, A, Q, THRES_IOU, THRES_SCENE, MAX_AGE, MODEL_SIZE
    global geometry

    ROOT_DIR = Path(__file__).resolve().parents[2]  # 获取根目录
    YOLO_DIR = ROOT_DIR.joinpath("src/utils/yolo")  # YOLO模型路径

    # 加载配置文件
    config_file = ROOT_DIR.joinpath("config/config.yaml")
    with open(config_file, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)  # 读取配置
    CUDA = config['use_cuda']  # 是否使用CUDA
    OUTPUT_DIR = ROOT_DIR.joinpath(config['output']['RadCamFusion_dir'])  # 输出路径
    SAVE_DIR = OUTPUT_DIR.joinpath("save")  # 保存路径
    SAVE_DIR.mkdir(exist_ok=True)  # 创建保存目录
    LOAD_DIR = OUTPUT_DIR.joinpath("load")  # 加载路径
    BASE_IMAGE_FILE = ROOT_DIR.joinpath(config['visual_assistant']['base_image'])  # 基础图像文件路径

    # 加载几何文件
    geometry_file = ROOT_DIR.joinpath("config/geometry.yaml")
    with open(geometry_file, 'r') as f:
        geometry = yaml.load(f, Loader=yaml.FullLoader)  # 读取几何信息
    MODEL_SIZE = geometry['scene']['kalman']['model_size']  # 模型大小
    A = np.array(geometry['scene']['kalman']['A']).reshape((MODEL_SIZE, MODEL_SIZE))  # 状态转移矩阵
    Q = np.array(geometry['scene']['kalman']['Q']).reshape((MODEL_SIZE, MODEL_SIZE))  # 过程噪声协方差矩阵
    MAX_AGE = geometry['scene']['kalman']['max_age']  # 最大年龄
    THRES_IOU = geometry['scene']['kalman']['iou_thres']  # IOU阈值
    THRES_SCENE = geometry['scene']['kalman']['scene_thres']  # 场景阈值


def raw_radar_process(radar: List[MsgRadarObject]) -> np.ndarray:
    """
    将雷达对象转换为NumPy数组格式。
    :param radar: 雷达对象列表
    :return: 包含距离、角度和速度的数组
    """
    if len(radar) == 0:
        return np.empty((0, 3))  # 如果没有雷达对象，返回空数组
    else:
        radar_data = np.array([np.array([obj.distance, obj.angle_centroid, obj.velocity]) for obj in radar])
        return radar_data  # 返回雷达数据


def msg2data(raw_radar_data: List[List[MsgRadarObject]], images: List[Image]) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    将原始雷达和图像数据转换为处理后的格式。
    :param raw_radar_data: 原始雷达数据列表
    :param images: 图像数据列表
    :return: 处理后的雷达数据和图像数据
    """
    global yolo
    radar_data, image_data = [], []
    for rd in raw_radar_data:
        radar_data.append(raw_radar_process(rd))  # 处理雷达数据
    for img in images:
        image_data.append(image_roi(img, yolo))  # 处理图像数据
    return radar_data, image_data


def msg2save(frame: int, save_path: Path, raw_radar_data: List[List[MsgRadarObject]], images: List[Image],
             odoms: List[Odometry], sensor_cluster: SensorCluster) -> None:
    """
    保存当前帧的雷达和图像数据。
    :param frame: 当前帧编号
    :param save_path: 保存路径
    :param raw_radar_data: 原始雷达数据
    :param images: 图像数据
    :param odoms: 里程计数据
    :param sensor_cluster: 传感器集群
    """
    radar_data = list(map(raw_radar_process, raw_radar_data))  # 处理雷达数据
    p = save_path.joinpath(str(frame))  # 创建帧保存目录
    p.mkdir(exist_ok=True)  # 创建目录
    for r, s in zip(radar_data, sensor_cluster.radar_sensors):
        np.savetxt(str(p.joinpath("{}.txt".format(s.name))), r)  # 保存雷达数据到文件
    for i, s in zip(images, sensor_cluster.image_sensors):
        i = CvBridge().imgmsg_to_cv2(i, 'bgr8')  # 转换图像格式
        cv2.imwrite(str(p.joinpath("{}.png".format(s.name))), i)  # 保存图像到文件
    for i, o in enumerate(odoms):
        pos = o.pose.pose.position  # 获取位置
        ori = o.pose.pose.orientation  # 获取方向
        ar, ap, ay = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])  # 四元数转欧拉角
        data = np.array([pos.x, pos.y, pos.z, ar, ap, ay])  # 创建数据数组
        np.savetxt(str(p.joinpath("odom_{}.txt".format(i))), data)  # 保存里程计数据到文件

    print("\033[0;32m保存帧 {} 的雷达和图像数据成功。\033[0m".format(frame))


def save2data(frame: int, load_path: Path, sensor_cluster: SensorCluster) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    从存储中加载指定帧的雷达和图像数据。
    :param frame: 帧编号
    :param load_path: 加载路径
    :param sensor_cluster: 传感器集群
    :return: 加载的雷达和图像数据
    """
    p = load_path.joinpath(str(frame))  # 获取帧加载目录
    if not p.exists():
        raise FileNotFoundError("该帧不存在。")  # 如果目录不存在，抛出异常
    radar_data, image_data = [], []
    for s in sensor_cluster.radar_sensors:
        d = np.loadtxt(str(p.joinpath("{}.txt".format(s.name))), dtype=float)  # 加载雷达数据
        d = d.reshape((-1, s.box_size))  # 重塑数组
        radar_data.append(d)
    for s in sensor_cluster.image_sensors:
        d = np.loadtxt(str(p.joinpath("{}.txt".format(s.name))), dtype=int)  # 加载图像数据
        d = d.reshape((-1, s.box_size))  # 重塑数组
        image_data.append(d)
    print("\033[0;32m加载帧 {} 的雷达和图像数据成功。\033[0m".format(frame))
    return radar_data, image_data


def fusion(radar: MsgRadar, image_2: Image, image_3: Image, image_5: Image, image_6: Image, image_7: Image, odom_1: Odometry,
           odom_2: Odometry) -> None: #完整高架路径
    """
    执行雷达和图像的融合处理。
    :param radar: 雷达消息
    :param image_2: 第二张图像
    :param image_3: 第三张图像
    :param image_5: 第五张图像
    :param image_6: 第六张图像
    :param image_7: 第七张图像
    :param odom_1: 第一辆车的里程计消息
    :param odom_2: 第二辆车的里程计消息
    """
    print("fusion working")
    global frame_counter
    global sensor_cluster, kf, va, args
    # 输出FPS和帧信息
    _ = my_timer()
    # 关闭YOLO模式（仅保存雷达和原始图像数据）
    if args.mode == 'off-yolo':
        msg2save(frame_counter, SAVE_DIR, [radar.objects_left, radar.objects_right],
                 [image_2, image_3, image_5, image_6, image_7], [odom_1, odom_2], sensor_cluster)
        sleep(0.05)  # 约20 FPS
        return
    # 获取雷达和图像数据
    radar_data, image_data = msg2data([radar.objects_left, radar.objects_right], [image_2, image_3, image_5, image_6, image_7])
    zs = fusion_core(radar_data, image_data, sensor_cluster)  # 执行融合核心处理
    # 可视化助手
    if args.save:
        va.scene_output(frame_counter, zs, kf)  # 保存可视化结果
    # 发布融合消息
    msg_rad_cam = MsgRadCam()
    msg_rad_cam.num_overpass = zs.total_objs  # 设置经过的目标数量
    msg_rad_cam.header.stamp = rospy.Time.now()  # 设置时间戳
    pub.publish(msg_rad_cam)  # 发布消息
    
def fusion_0(radar: MsgRadar, image_2: Image, odom_1: Odometry, odom_2: Odometry) -> None: #离开高架
    print("fusion_0 working")
    global frame_counter
    global sensor_cluster, kf, va, args
    # 输出FPS和帧信息
    _ = my_timer()
    # 关闭YOLO模式（仅保存雷达和原始图像数据）
    if args.mode == 'off-yolo':
        msg2save(frame_counter, SAVE_DIR, [radar.objects_left, radar.objects_right],
                 [image_2], [odom_1, odom_2], sensor_cluster)
        sleep(0.05)  # 约20 FPS
        return
    # 获取雷达和图像数据
    radar_data, image_data = msg2data([radar.objects_left, radar.objects_right], [image_2])
    zs = fusion_core(radar_data, image_data, sensor_cluster)  # 执行融合核心处理
    # 可视化助手
    if args.save:
        va.scene_output(frame_counter, zs, kf)  # 保存可视化结果
    # 发布融合消息
    msg_rad_cam = MsgRadCam()
    msg_rad_cam.num_overpass = zs.total_objs  # 设置经过的目标数量
    msg_rad_cam.header.stamp = rospy.Time.now()  # 设置时间戳
    pub.publish(msg_rad_cam)  # 发布消息
def fusion_1(radar: MsgRadar, image_3: Image, odom_1: Odometry, odom_2: Odometry) -> None: #进入高架
    print("fusion_1 working")
    global frame_counter
    global sensor_cluster, kf, va, args
    # 输出FPS和帧信息
    _ = my_timer()
    # 关闭YOLO模式（仅保存雷达和原始图像数据）
    if args.mode == 'off-yolo':
        msg2save(frame_counter, SAVE_DIR, [radar.objects_left, radar.objects_right],
                 [image_3], [odom_1, odom_2], sensor_cluster)
        sleep(0.05)  # 约20 FPS
        return
    # 获取雷达和图像数据
    radar_data, image_data = msg2data([radar.objects_left, radar.objects_right], [image_3])
    zs = fusion_core(radar_data, image_data, sensor_cluster)  # 执行融合核心处理
    # 可视化助手
    if args.save:
        va.scene_output(frame_counter, zs, kf)  # 保存可视化结果
    # 发布融合消息
    msg_rad_cam = MsgRadCam()
    msg_rad_cam.num_overpass = zs.total_objs  # 设置经过的目标数量
    msg_rad_cam.header.stamp = rospy.Time.now()  # 设置时间戳
    pub.publish(msg_rad_cam)  # 发布消息

def fusion_core(radar_data: List[np.ndarray], image_data: List[np.ndarray], sensor_cluster: SensorCluster) -> ObsBundle:
    """
    执行融合核心处理，包括传感器更新和观测。
    :param radar_data: 雷达数据
    :param image_data: 图像数据
    :param sensor_cluster: 传感器集群
    :return: 观测数据
    """
    global frame_counter
    global kf, va, args
    global A
    # 更新传感器并获取观测
    sensor_cluster.update(radar_data, image_data)
    zs = sensor_cluster.observe()  # 获取观测数据
    print("\033[0;36m检测结果\033[0m", zs, sep='\n')
    # 卡尔曼滤波
    kf.flush(A, zs)  # 更新卡尔曼滤波器
    print("\033[0;36m卡尔曼滤波结果\033[0m", kf, sep='\n')
    return zs


if __name__ == '__main__':
    # 设置命令行参数
    parser = argparse.ArgumentParser()
    parser.add_argument("-m",
                        "--mode",
                        choices=['normal', 'off-yolo', 'from-save'],
                        type=str,
                        default='normal',
                        required=False,
                        help="模式选择。")
    parser.add_argument("-s",
                        "--save",
                        action='store_true',
                        default=False,
                        required=False,
                        help="保存目标轨迹为图像。")
    args = parser.parse_args()

    my_file_loader()  # 加载配置和几何信息

    # 初始化
    # YOLO
    if args.mode == 'normal':
        #yolo = YOLO(YOLO_DIR, cuda=CUDA)  # 初始化YOLO
        yolo = YOLO(YOLO_DIR, cuda=False)  # 初始化YOLO
        print("\033[0;32mYOLO成功初始化。\033[0m")
    else:
        yolo = None
        print("\033[0;33m运行在关闭YOLO模式。\033[0m")
    # 雷达初始化
    rad_2 = RadarSensor("Radar_2", geometry['radars']['radar_2'])  # 第二个雷达
    rad_3 = RadarSensor("Radar_3", geometry['radars']['radar_3'])  # 第三个雷达
    # 相机初始化
    cam_2 = ImageSensor("Image_2", geometry['cameras']['camera_2'], rad_2.offset[2])  # 第二个相机
    cam_3 = ImageSensor("Image_3", geometry['cameras']['camera_3'], rad_3.offset[2])  # 第三个相机
    cam_5 = ImageSensor("Image_5", geometry['cameras']['camera_5'], rad_2.offset[2])  # 第五个相机
    cam_6 = ImageSensor("Image_6", geometry['cameras']['camera_6'], rad_3.offset[2])  # 第六个相机
    cam_7 = ImageSensor("Image_7", geometry['cameras']['camera_7'], rad_2.offset[2])  # 第七个相机
    # 传感器集群
    #sensor_cluster = SensorCluster([rad_2, rad_3], [cam_2, cam_3, cam_5, cam_6, cam_7]) #fusion
    #sensor_cluster = SensorCluster([rad_2, rad_3], [cam_2]) #fusion_0
    sensor_cluster = SensorCluster([rad_2, rad_3], [cam_3]) #fusion_1
    # 卡尔曼滤波器初始化
    kf = Kalman(MODEL_SIZE, Q, THRES_SCENE, MAX_AGE)
    # 可视化助手初始化
    va = VisualAssistant(BASE_IMAGE_FILE, OUTPUT_DIR)

    # 从保存模式
    if args.mode == 'from-save':
        frame = 0
        while True:
            try:
                frame += 1
                radar_data, image_data = save2data(frame, LOAD_DIR, sensor_cluster)  # 加载数据
            except FileNotFoundError:
                print("\033[0;32m遍历完所有 {} 帧。\033[0m".format(frame))
                exit(0)
            finally:
                zs = fusion_core(radar_data, image_data, sensor_cluster)  # 执行融合核心处理
                # 可视化助手
                if args.save:
                    va.scene_output(frame, zs, kf)  # 保存可视化结果

    # 初始化发布者
    pub = rospy.Publisher('/radar_camera_fused', MsgRadCam, queue_size=10)
    # 初始化ROS节点
    rospy.init_node('radar_camera_fusion', anonymous=True)
    # 订阅消息(进入高架fusion_1只需订阅image_raw_3，离开高架fusion_0只需订阅image_raw_2)
    msg_radar = message_filters.Subscriber('/radar_msgs_combined', MsgRadar)  # 雷达消息订阅
    msg_image_2 = message_filters.Subscriber('/image_raw_2', Image)  # 第二张图像订阅
    msg_image_3 = message_filters.Subscriber('/image_raw_3', Image)  # 第三张图像订阅
    msg_image_5 = message_filters.Subscriber('/image_raw_5', Image)  # 第五张图像订阅
    msg_image_6 = message_filters.Subscriber('/image_raw_6', Image)  # 第六张图像订阅
    msg_image_7 = message_filters.Subscriber('/image_raw_7', Image)  # 第七张图像订阅

    msg_odom_1 = message_filters.Subscriber('/car1/base_pose_ground_truth', Odometry)  # 第一辆车的里程计订阅
    msg_odom_2 = message_filters.Subscriber('/car2/base_pose_ground_truth', Odometry)  # 第二辆车的里程计订阅
    # 同步时间戳
    #完整高架路径fusion
    sync = message_filters.ApproximateTimeSynchronizer([msg_radar, msg_image_2, msg_image_3, msg_image_5, msg_image_6, msg_image_7, msg_odom_1, msg_odom_2], 1, 1)
    sync.registerCallback(fusion)  # 注册回调函数
    #离开高架fusion_0
    #sync = message_filters.ApproximateTimeSynchronizer([msg_radar, msg_image_2, msg_odom_1, msg_odom_2], 1, 1)
    #sync.registerCallback(fusion_0)
    #进入高架fusion_1
    #sync = message_filters.ApproximateTimeSynchronizer([msg_radar, msg_image_3, msg_odom_1, msg_odom_2], 1, 1)
    #sync.registerCallback(fusion_1)
    print("\033[0;32m雷达-相机融合成功初始化。\033[0m")
    rospy.spin()  # 保持节点运行
