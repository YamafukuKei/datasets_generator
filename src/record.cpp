#include <ros/ros.h>
// PCL specific includes
// #include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <sstream>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/Pose.h>

#include "pcl_ros/transforms.h"
#include <fstream>
#include <iostream>
#include <stdio.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
int t = 0;
int max_data = 10000;

ros::Publisher pub;

void
get_tf (const geometry_msgs::PoseStamped pose)
{
  //std::cout << "ROS_time(record_tf   ) : " << ros::Time::now() << std::endl;
  float tf[7] = {0};
  tf[0] = pose.pose.position.x;
  tf[1] = pose.pose.position.y;
  tf[2] = pose.pose.position.z;
  tf[3] = pose.pose.orientation.x;
  tf[4] = pose.pose.orientation.y;
  tf[5] = pose.pose.orientation.z;
  tf[6] = pose.pose.orientation.w;

  printf("%f,%f,%f,%f,%f,%f,%f\n",tf[0],tf[1],tf[2],tf[3],tf[4],tf[5],tf[6]);
  char filename[100];
  sprintf(filename, "tf_%d.csv", t);

  FILE *fp;
  fp = fopen(filename, "w");

  int i = 0;
  for(i = 0; i < 7; i++){
    fprintf(fp, "%f\n", tf[i]);
  }

  fclose(fp);
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
//  // kinect座標系からworld座標系へ変換
//  tf::TransformListener tf_;
//  sensor_msgs::PointCloud2 trans_cloud;
//
//  try
//  {
//    pcl_ros::transformPointCloud("world", *cloud_msg, trans_cloud, tf_);
//  } catch (tf::ExtrapolationException e)
//  {
//    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
//  }

  std::cout << "ROS_time(record_cloud) : " << ros::Time::now() << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PassThrough<pcl::PointXYZRGBA> pass;
//  pass.setInputCloud (cloud);
//  pass.setFilterFieldName ("y");
//  pass.setFilterLimits(-0.5,0.2);
//  pass.filter(*cloud_filtered);

  // nanを除去
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

  //segmentation 平面除去
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);

  double dist_th = 0.02;
  seg.setDistanceThreshold (dist_th);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_filtered);

  std::stringstream ss;
  ss << t;
  std::string filename = "cloud_" + ss.str() + ".pcd";

  pcl::io::savePCDFileASCII (filename, *cloud_filtered);
//  std::cout << filename << std::endl;
//  std::cerr << "Saved " << cloud_filtered->points.size () << " data points to save.pcd." << std::endl;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcd_record");
//  ros::NodeHandle n;
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub2 = nh.subscribe ("input", 1, cloud_cb);
  ros::Subscriber sub1 = nh.subscribe ("/posestamped_obj", 1, get_tf);
  ros::Rate loop_rate(10);
  // Spin
    while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    t += 1;
  }
  //ros::spin ();
}
