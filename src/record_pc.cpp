// Save the point cloud to pcd in world coordinate system

#include <ros/ros.h>
#include <ros/package.h>

#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <sstream>

#include <fstream>
#include <iostream>
#include <stdio.h>

std::string model_filepath = ros::package::getPath("kinect_bringup") + "/pcds/";
std::string model_filename;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& source)
{
  tf::TransformListener tf_listener;
  tf::StampedTransform trans_model;

  sensor_msgs::PointCloud2 converted_cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr converted_model (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  pcl::fromROSMsg(*source, *model_cloud);

  try{
    tf_listener.waitForTransform("/world", "/kinect_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
    tf_listener.lookupTransform("/world", "/kinect_rgb_optical_frame", ros::Time(0), trans_model);
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(trans_model, eigen_transform);
    Eigen::Affine3f eigen_affine_transform(eigen_transform);
    pcl::transformPointCloud(*model_cloud, *converted_model, eigen_affine_transform);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }

  std::string model_filename = "cloud.pcd";

  pcl::io::savePCDFileASCII (model_filepath + model_filename, *converted_model);
  std::cout << "save " << model_filename << std::endl;

}

int
main (int argc, char** argv)
{
  int max_data = atoi(argv[2]);
  ros::init (argc, argv, "pcd_record");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("/kinect/hd/points", 1, cloud_cb);

  ros::Rate loop_rate(1000);
  while (ros::ok()){
    ros::spinOnce();
  }

}
