/*###########################################################
#   此C++文件订阅激光雷达主题并发布集成的点云信息。   #
#############################################################*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>  // 导入点云消息类型
#include <pcl_conversions/pcl_conversions.h>  // PCL与ROS消息转换
#include <pcl/io/pcd_io.h>  // 用于读取和写入PCD文件
#include <pcl/point_cloud.h>  // 点云类型定义
#include <pcl/filters/statistical_outlier_removal.h>  // 离群值剔除
#include <pcl/filters/extract_indices.h>  // 提取点云中的内点
#include <pcl/sample_consensus/method_types.h>  // 采样一致性方法类型
#include <pcl/sample_consensus/model_types.h>  // 模型类型定义
#include <pcl/segmentation/sac_segmentation.h>  // RANSAC分割
#include <pcl/filters/voxel_grid.h>  // 体素网格滤波
#include <pcl/ModelCoefficients.h>  // 模型系数
#include <pcl/point_types.h>  // 点类型定义
#include <msgs/ListPointCloud.h>  // 自定义点云列表消息

using namespace std;

// 定义点云格式为XYZI（包括强度信息）
typedef pcl::PointXYZI PointT; 
typedef pcl::PointCloud<PointT> PointCloudT;  // 定义点云类型

// 激光雷达的位置偏移定义
#define lidar11_x -0.55740706
#define lidar11_y 1.25432157
#define lidar11_z 0.006

#define lidar12_x 0.43254243
#define lidar12_y 0.26437207
#define lidar12_z 0.006

#define lidar2_x 9.74248361e-04
#define lidar2_y -9.89292350e-01
#define lidar2_z 1.91780397e-01

class pointcloud_combiner
{
private:
    ros::NodeHandle nh;  // ROS节点句柄
    ros::Publisher pub;  // 点云发布者
    ros::Subscriber sub;  // 点云订阅者
public:
    pointcloud_combiner()
    {
        pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_combined", 1);  // 创建点云组合的发布者
        sub = nh.subscribe("/pointcloud_list", 1, &pointcloud_combiner::callback, this);  // 订阅点云列表
    }

    void callback(const msgs::ListPointCloud& pointcloud_list)
    {
        // 定义点云类型：sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 cloud11, cloud12, cloud2;
        cloud11 = pointcloud_list.pointcloud[0];  // 获取第一个点云
        cloud12 = pointcloud_list.pointcloud[1];  // 获取第二个点云
        cloud2 = pointcloud_list.pointcloud[2];  // 获取第三个点云

        // 将ROS消息转换为PCL点云格式
        PointCloudT cloud11_pcl, cloud12_pcl, cloud2_pcl;
        pcl::fromROSMsg(cloud11, cloud11_pcl);
        pcl::fromROSMsg(cloud12, cloud12_pcl);
        pcl::fromROSMsg(cloud2, cloud2_pcl);

        // 调整点云位置
        for(int i = 0; i < cloud2_pcl.points.size(); i++)
        {
            // 调整cloud11的位置
            if(i < cloud11_pcl.points.size())
            {
                cloud11_pcl.points[i].x += lidar11_x;  // X坐标偏移
                cloud11_pcl.points[i].y += lidar11_y;  // Y坐标偏移
                cloud11_pcl.points[i].z += lidar11_z;  // Z坐标偏移
            }
            // 调整cloud12的位置
            if(i < cloud12_pcl.points.size())
            {
                cloud12_pcl.points[i].x += lidar12_x;  // X坐标偏移
                cloud12_pcl.points[i].y += lidar12_y;  // Y坐标偏移
                cloud12_pcl.points[i].z += lidar12_z;  // Z坐标偏移
            }
            // 调整cloud2的位置
            cloud2_pcl.points[i].x += lidar2_x;  // X坐标偏移
            cloud2_pcl.points[i].y += lidar2_y;  // Y坐标偏移
            cloud2_pcl.points[i].z += lidar2_z;  // Z坐标偏移
        }

        // 合并点云并去除地面
        PointCloudT cloud = cloud11_pcl + cloud12_pcl + cloud2_pcl;  // 合并点云
        PointCloudT cloud_filtered, cloud_segmented;

        // 使用统计离群值剔除
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> statFilter;
        statFilter.setInputCloud(cloud.makeShared());  // 设置输入点云
        statFilter.setMeanK(10);  // 使用10个邻域点
        statFilter.setStddevMulThresh(0.2);  // 设置标准差阈值
        statFilter.filter(cloud_filtered);  // 过滤点云

        // 使用RANSAC算法进行平面分割
        pcl::ModelCoefficients coefficients;  // 模型系数初始化
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  // 索引参数初始化
        pcl::SACSegmentation<pcl::PointXYZI> segmentation;  // 创建分割算法
        segmentation.setModelType(pcl::SACMODEL_PLANE);  // 设置模型类型为平面
        segmentation.setMethodType(pcl::SAC_RANSAC);  // 设置算法为RANSAC
        segmentation.setMaxIterations(1000);  // 设置最大迭代次数
        segmentation.setDistanceThreshold(0.01);  // 设置最大距离阈值
        segmentation.setInputCloud(cloud_filtered.makeShared());  // 输入点云
        segmentation.segment(*inliers, coefficients);  // 输出分割结果

        // 发布模型系数
        pcl_msgs::ModelCoefficients ros_coefficients;
        pcl_conversions::fromPCL(coefficients, ros_coefficients);  // PCL转换为ROS消息
            
        // 发布内点索引
        pcl_msgs::PointIndices ros_inliers;
        pcl_conversions::fromPCL(*inliers, ros_inliers);

        // 创建分割点云，从点云中提取内点
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_filtered.makeShared());  // 设置输入点云
        extract.setIndices(inliers);  // 设置内点索引
        extract.setNegative(true);  // 提取外点
        extract.filter(cloud_segmented);  // 过滤点云
        cout << "Combined cloud size: " << cloud_segmented.points.size() << endl;  // 输出合并后点云的大小

        // 发布合并后的点云
        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_combined", 1);  // 创建发布者
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud_segmented, output);  // 将PCL点云转换为ROS消息
        output.header.frame_id = "point_cloud";  // 设置坐标框架ID
        output.header.stamp = ros::Time::now();  // 设置时间戳
        pub.publish(output);  // 发布合并后的点云
    }
};

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pointcloud_combiner");  // 初始化ROS节点
    pointcloud_combiner ptc;  // 创建点云组合器对象

    cout << "Pointcloud Combine Begin." << endl;  // 输出启动信息
    ros::spin();  // 保持节点运行

    return 0;
}