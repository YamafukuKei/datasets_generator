// Save the point cloud to pcd in world coordinate system

#include <ros/ros.h>
#include <ros/package.h>

#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <sstream>

#include <fstream>
#include <iostream>
#include <stdio.h>

std::string model_filepath = ros::package::getPath("kinect_bringup") + "/pcds/";
std::string model_filename;

class RecordCloud
{
public:
  RecordCloud(ros::NodeHandle nh);
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& source);
  void run();

private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber sub1_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr converted_model;
};

RecordCloud::RecordCloud(ros::NodeHandle nh)
  :nh_(nh)
  ,model_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ())
  ,converted_model (new pcl::PointCloud<pcl::PointXYZRGBA> ())
{
  sub1_ = nh_.subscribe("/kinect/hd/points", 1, &RecordCloud::cloud_cb, this);
}

void RecordCloud::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& source)
{
  tf::StampedTransform trans_model;
  ros::Duration(1.0).sleep();

  sensor_msgs::PointCloud2 transformed_cloud_msg;
  //transfomation of coordinate
  // try{
  std::string sensor_frame_id = source->header.frame_id;
  std::cout << "sensor_frame : " << sensor_frame_id << std::endl;


  try
  {
    ros::Time t = ros::Time(0);
    tf_listener_.waitForTransform("/world", sensor_frame_id, t, ros::Duration(1.0));
    pcl_ros::transformPointCloud("world", *source, transformed_cloud_msg, tf_listener_);
  }
  catch (tf::ExtrapolationException e)
  {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }
  pcl::fromROSMsg(transformed_cloud_msg, *converted_model);
  // pcl::fromROSMsg(*source, *converted_model);

  // pcl::visualization::CloudViewer viewer("Cloud Viewer");
  // viewer.showCloud(converted_model);
  // while (!viewer.wasStopped ())
  //     {
  //     }


  std::cout << "point cloud size : " << converted_model->points.size() << std::endl;
  if (converted_model->points.size()>0)
  {
    model_filename = "cloud.pcd";
    pcl::io::savePCDFileASCII (model_filepath + model_filename, *converted_model);
    std::cout << "save " << model_filename << std::endl;
  }
}

void RecordCloud::run()
{
  while (nh_.ok())
  {
    ros::spinOnce();
  }
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcd_record");
  ros::NodeHandle nh;

  RecordCloud record(nh);
  record.run();
  // ros::Subscriber sub1 = nh.subscribe("/kinect/hd/points", 1, cloud_cb);

  // while (ros::ok()){
  //   ros::spinOnce();
  // }

}
