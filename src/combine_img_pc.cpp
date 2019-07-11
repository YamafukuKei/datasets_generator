// Merge point cloud and grayscale image acquired by phoxi. And save the colored cloud as pcd data

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <sstream>

#include "pcl_ros/transforms.h"
#include <fstream>
#include <iostream>
#include <stdio.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

ros::Publisher pub;

int row = 772;
int col = 1032;
cv::Mat rgb_image;

void
image_cb (const sensor_msgs::Image::ConstPtr sensor_image)
{
  cv_bridge::CvImageConstPtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvCopy(sensor_image, sensor_msgs::image_encodings::TYPE_32FC1);
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  row = cv_img_ptr->image.rows;
  col = cv_img_ptr->image.cols;

  // original unscaled image
  cv::Mat cv_image_ori(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
  cv_image_ori = cv_img_ptr->image;
  // scaled image
  cv::Mat cv_image_scaled(cv_image_ori);

  cv::MatConstIterator_<float> it = cv_image_ori.begin<float>(), it_end = cv_image_ori.end<float>();
  float max_val = *std::max_element(it, it_end);
  cv_image_ori.convertTo(cv_image_scaled, CV_32FC1, 1.0/max_val, 0); //[0, 4095] -> [0, 1]
  std::cout << max_val << std::endl;

  // equalize hist
  cv::Mat cv_image_gray(cv_image_scaled.rows, cv_image_scaled.cols, CV_8UC1);
  cv_image_scaled.convertTo(cv_image_gray, CV_8UC1, 255.0/1.0, 0); //[0, 1] -> [0, 255]
  cv::Mat cv_image_hist_equ(cv_image_gray);
  cv::equalizeHist(cv_image_gray, cv_image_hist_equ);

  cv::cvtColor(cv_image_hist_equ, rgb_image, CV_GRAY2BGR);

//  const std::string window_name = "test_cv_image";
//  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
//  cv::imshow(window_name, rgb_image);
//  cv::waitKey(0);

}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& sensor_cloud)
{
  std::cout << "ROS_time(record_cloud) : " << ros::Time::now() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (*sensor_cloud, *cloud);
  std::cout << row << std::endl;
  std::cout << col << std::endl;

  int i,j = 0;
  for (i = 0; i < row; i++) {
    for (j = 0; j < col; j++) {
//      std::cout << i*row + j << std::endl;
      cloud->points[i*col + j].r = rgb_image.at<cv::Vec3b>(i, j)[0];
      cloud->points[i*col + j].g = rgb_image.at<cv::Vec3b>(i, j)[1];
      cloud->points[i*col + j].b = rgb_image.at<cv::Vec3b>(i, j)[2];
    }
  }
  std::cout << i << ":" << j << std::endl;
  std::string filename = "cloud.pcd";

  pcl::io::savePCDFileASCII (filename, *cloud);
  std::cout << "save " << filename << std::endl;

  sensor_msgs::PointCloud2 colored_pc;
  pcl::toROSMsg(*cloud, colored_pc);
  colored_pc.header.frame_id = sensor_cloud->header.frame_id;
  colored_pc.header.stamp = ros::Time::now();

  pub.publish(colored_pc);
  //  std::cerr << "Saved " << cloud_filtered->points.size () << " data points to save.pcd." << std::endl;
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "pcd_record");
  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::PointCloud2>("/colored_pc", 1);
  ros::Subscriber sub1 = nh.subscribe("/phoxi_camera/pointcloud", 1, cloud_cb);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("/phoxi_camera/texture", 1, image_cb);

  ros::Rate loop_rate(1000);
  while (ros::ok()){
    ros::spinOnce();
  }

}
