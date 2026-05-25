#!/usr/bin/env python3
import rospy
import ros_numpy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from threading import Lock, Thread
class PersistentViewVisualizer:
    def __init__(self):
        # 初始化Open3D窗口
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="PointCloud", width=1280, height=720)
        self.pcd = o3d.geometry.PointCloud()
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.5)
        # 视角控制组件
        self.view_ctl = self.vis.get_view_control()
        self.current_view = None  # 存储当前视角参数
        
        # ROS订阅
        rospy.Subscriber("/point_cloud_combined", PointCloud2, self.pcl_callback, queue_size=1, buff_size=2**24)  # 64MB缓冲区
        rospy.loginfo("启动点云查看器")
        
        self.lock = Lock()  # 添加线程锁
        self.render_thread = Thread(target=self._render_loop)
        self.render_thread.start()
    def _render_loop(self):  # 独立渲染线程
        while not rospy.is_shutdown():
            self.vis.poll_events()
            self.vis.update_renderer()
            rospy.sleep(0.01)  # 控制60Hz刷新
        self.vis.destroy_window()
            
    def pcl_callback(self, msg):
        try:
            # 添加预处理滤波（需在回调函数内实现）
            cl, ind = self.pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            #self.pcd = self.pcd.select_by_index(ind)  # 移除离群点
            filtered_pcd = self.pcd.select_by_index(ind)
            self.pcd.points = filtered_pcd.points  # 复用对象
            
            # 转换ROS点云为Open3D格式
            pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            xyz = ros_numpy.point_cloud2.get_xyz_points(pc_array).reshape(-1,3)
            
            # 处理颜色数据
            if 'rgb' in pc_array.dtype.names:
                rgb = ros_numpy.point_cloud2.split_rgb_field(pc_array)['rgb']
                self.pcd.colors = o3d.utility.Vector3dVector(rgb.reshape(-1,3)/255.0)
            
            # 更新点云坐标
            self.pcd.points = o3d.utility.Vector3dVector(xyz)
            
            # 保存当前视角参数
            self.current_view = self.view_ctl.convert_to_pinhole_camera_parameters()
            
            # 动态更新几何体
            #self.vis.clear_geometries()  # 清空旧对象
            #self.vis.add_geometry(self.pcd)
            # 初始化时添加一次几何体
            if not hasattr(self, '_geometry_added'):
                self.vis.add_geometry(self.pcd)
                self._geometry_added = True

            # 直接更新点云数据
            self.pcd.points = o3d.utility.Vector3dVector(xyz)
            self.vis.update_geometry(self.pcd)
            
            # 恢复用户调整后的视角
            self.view_ctl.convert_from_pinhole_camera_parameters(self.current_view)
            
            
            
            
        except Exception as e:
            rospy.logerr(f"点云处理异常: {str(e)}")

if __name__ == "__main__":
    rospy.init_node("persistent_view_pcl")
    visualizer = PersistentViewVisualizer()
