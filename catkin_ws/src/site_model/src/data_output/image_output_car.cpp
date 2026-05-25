#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

void car1_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car1_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car1_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car1_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car1_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car1_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car2_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car2_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car2_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car2_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car2_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car2_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car3_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car3_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car3_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car3_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car3_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car3_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car4_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car4_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car4_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car4_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car4_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car4_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car5_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car5_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car5_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car5_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car5_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car5_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car6_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car6_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car6_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car6_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car6_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car6_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car7_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car7_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car7_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car7_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car7_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car7_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car8_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car8_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car8_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car8_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car8_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car8_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car9_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car9_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car9_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car9_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car9_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car9_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car10_left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car10_left.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car10_left.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}

void car10_right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//sensor_msgs::Image ROS中image传递的消息形式
  try
  {
    cv::imshow("image2", cv_bridge::toCvShare(msg, "bgr8")->image);
    if(cv::imwrite("/home/sunhao/catkin_ws/src/site_model/dataset/camera_data/car10_right.jpg",cv_bridge::toCvShare(msg)->image)>=0)
    {std::cerr << "Saved car10_right.jpg" << std::endl;}
   // cv::WaitKey(3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_output_car");
  ros::NodeHandle nh;
  // cv::namedWindow("image2");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub1_1 = it.subscribe("/car1/car1/camera/zed_left/image_rect_color_left", 1, car1_left_imageCallback);
  image_transport::Subscriber sub1_2 = it.subscribe("/car1/car1/camera/zed_right/image_rect_color_right", 1, car1_right_imageCallback);
  image_transport::Subscriber sub2_1 = it.subscribe("/car2/car2/camera/zed_left/image_rect_color_left", 1, car2_left_imageCallback);
  image_transport::Subscriber sub2_2 = it.subscribe("/car2/car2/camera/zed_right/image_rect_color_right", 1, car2_right_imageCallback);
  image_transport::Subscriber sub3_1 = it.subscribe("/car3/car3/camera/zed_left/image_rect_color_left", 1, car3_left_imageCallback);
  image_transport::Subscriber sub3_2 = it.subscribe("/car3/car3/camera/zed_right/image_rect_color_right", 1, car3_right_imageCallback);
  image_transport::Subscriber sub4_1 = it.subscribe("/car4/car4/camera/zed_left/image_rect_color_left", 1, car4_left_imageCallback);
  image_transport::Subscriber sub4_2 = it.subscribe("/car4/car4/camera/zed_right/image_rect_color_right", 1, car4_right_imageCallback);
  image_transport::Subscriber sub5_1 = it.subscribe("/car5/car5/camera/zed_left/image_rect_color_left", 1, car5_left_imageCallback);
  image_transport::Subscriber sub5_2 = it.subscribe("/car5/car5/camera/zed_right/image_rect_color_right", 1, car5_right_imageCallback);
  image_transport::Subscriber sub6_1 = it.subscribe("/car6/car6/camera/zed_left/image_rect_color_left", 1, car6_left_imageCallback);
  image_transport::Subscriber sub6_2 = it.subscribe("/car6/car6/camera/zed_right/image_rect_color_right", 1, car6_right_imageCallback);
  image_transport::Subscriber sub7_1 = it.subscribe("/car7/car7/camera/zed_left/image_rect_color_left", 1, car7_left_imageCallback);
  image_transport::Subscriber sub7_2 = it.subscribe("/car7/car7/camera/zed_right/image_rect_color_right", 1, car7_right_imageCallback);
  image_transport::Subscriber sub8_1 = it.subscribe("/car8/car8/camera/zed_left/image_rect_color_left", 1, car8_left_imageCallback);
  image_transport::Subscriber sub8_2 = it.subscribe("/car8/car8/camera/zed_right/image_rect_color_right", 1, car8_right_imageCallback);
  image_transport::Subscriber sub9_1 = it.subscribe("/car9/car9/camera/zed_left/image_rect_color_left", 1, car9_left_imageCallback);
  image_transport::Subscriber sub9_2 = it.subscribe("/car9/car9/camera/zed_right/image_rect_color_right", 1, car9_right_imageCallback);
  image_transport::Subscriber sub10_1 = it.subscribe("/car10/car10/camera/zed_left/image_rect_color_left", 1, car10_left_imageCallback);
  image_transport::Subscriber sub10_2 = it.subscribe("/car10/car10/camera/zed_right/image_rect_color_right", 1, car10_right_imageCallback);
  ros::spin();
  // cv::destroyWindow("image2");  //窗口
}