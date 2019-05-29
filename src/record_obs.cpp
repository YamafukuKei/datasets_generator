#include <ros/ros.h>
// PCL specific includes
// #include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <sstream>
#include <boost/format.hpp>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/TransformStamped.h>

#include "pcl_ros/transforms.h"
#include <fstream>
#include <iostream>
#include <stdio.h>

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
int t = 0;
int count = 0;

ros::Publisher pub;


//camera位置と姿勢(x,y,yaw)の取得
double x = 1;
double y = 0;
double z = 0.5;
double roll = 0;
double pitch = 0.35;
double yaw = -3.14;

class PoseObj
{
public:
  double pos_x;
  double pos_y;
  double pos_z;

  double ori_x;
  double ori_y;
  double ori_z;
  double ori_w;

  ros::Time time;
};

class PosPrincipal
{
public:
  double pos_x;
  double pos_y;
  double pos_z;
  ros::Time time;
};

PoseObj poseobj;
PosPrincipal pospri;

void
get_tf ()
{
  float tf[7] = {0};
  tf[0] = poseobj.pos_x;
  tf[1] = poseobj.pos_y;
  tf[2] = poseobj.pos_z;

  tf[3] = poseobj.ori_x;
  tf[4] = poseobj.ori_y;
  tf[5] = poseobj.ori_z;
  tf[6] = poseobj.ori_w;

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
pose_cb (const geometry_msgs::PoseStampedConstPtr& pose)
{
  if (count == 0)
  {
  poseobj.pos_x = pose->pose.position.x;
  poseobj.pos_y = pose->pose.position.y;
  poseobj.pos_z = pose->pose.position.z;

  poseobj.ori_x = pose->pose.orientation.x;
  poseobj.ori_y = pose->pose.orientation.y;
  poseobj.ori_z = pose->pose.orientation.z;
  poseobj.ori_w = pose->pose.orientation.w;

  poseobj.time = pose->header.stamp;

  count = 1;
  }
}

void
principal_cb (const geometry_msgs::PoseStampedConstPtr& principal)
{
  if (count == 1)
  {
  pospri.pos_x = principal->pose.position.x;
  pospri.pos_y = principal->pose.position.y;
  pospri.pos_z = principal->pose.position.z;

  pospri.time = principal->header.stamp;

  count = 2;
  }
}

void crop_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  float tf[7] = {0};
  tf[0] = poseobj.pos_x;
  tf[1] = poseobj.pos_y;
  tf[2] = poseobj.pos_z;
  float qx = poseobj.ori_x;
  float qy = poseobj.ori_y;
  float qz = poseobj.ori_z;
  float qw = poseobj.ori_w;

//  Eigen::Vector3f principal; // cadモデルの主成分
//  principal[0] = 0.065;
//  principal[1] = 0.1;
//  principal[2] = 0.25;
//
//  Eigen::Matrix3f transform; // 変換行列
//  transform(0,0) = qw*qw + qx*qx - qy*qy - qz*qz;
//  transform(0,1) = 2*(qx*qy-qw*qz);
//  transform(0,2) = 2*(qx*qz+qw*qy);
//  transform(1,0) = 2*(qx*qy+qw*qz);
//  transform(1,1) = qw*qw - qx*qx + qy*qy - qz*qx;
//  transform(1,2) = 2*(qy*qz-qw*qx);
//  transform(2,0) = 2*(qx*qz-qw*qy);
//  transform(2,1) = 2*(qy*qz+qw*qx);
//  transform(2,2) = qw*qw - qx*qx - qy*qy + qz*qz;
//
//  Eigen::Vector3f transformed; // cadモデルの主成分
//  transformed = transform * principal;
//  std::cout << "transformed : " << transformed[0] << "," << transformed[1] << "," << transformed[2] << std::endl;
//
//  float theta = 2 * std::acos(qw);
  Eigen::Vector3f lamda; // クオータニオンの回転軸
  lamda[0] = pospri.pos_x;
  lamda[1] = pospri.pos_y;
  lamda[2] = pospri.pos_z;
  std::cout << "lamda : " << lamda[0] << " , " << lamda[1] << " , " << lamda[2] << std::endl;
  float norm = lamda.norm();
  lamda = lamda/norm * 0.13;
  std::cout << "norm : " << norm << std::endl;
//  lamda[0] = qx/std::sin(theta/2);
//  lamda[1] = qy/std::sin(theta/2);
//  lamda[2] = qz/std::sin(theta/2);

//  std::cout << "theta : " << theta << std::endl;
//  std::cout << "lamda : " << lamda[0] << ", " << lamda[1] << ", " << lamda[2] << std::endl;
//  lamda = 0.15 * lamda;

  Eigen::Vector3f point1;
//  point1[0] = tf[0] + lamda[1];
//  point1[1] = tf[1] + lamda[1];
//  point1[2] = tf[2] + lamda[2];
  point1[0] = tf[0] + pospri.pos_x;
  point1[1] = tf[1] + pospri.pos_y;
  point1[2] = tf[2] + pospri.pos_z;

  std::cout << "tf    : " << tf[0] << ", " << tf[1] << ", " << tf[2] << std::endl;
//  std::cout << "point : " << point1[0] << ", " << point1[1] << ", " << point1[2] << std::endl;

  Eigen::Vector4f minPoint;
  minPoint[0] = tf[0] - 0.1;
  minPoint[1] = tf[1] - 0.1;
  minPoint[2] = tf[2] - 0.1;

  Eigen::Vector4f maxPoint;
  maxPoint[0] = tf[0] + 0.1;
  maxPoint[1] = tf[1] + 0.1;
  maxPoint[2] = tf[2] + 0.1;

  if(tf[0] < point1[0]){
    minPoint[0] = tf[0] - 0.01;
    maxPoint[0] = point1[0] + 0.01;
  } else if (tf[0] > point1[0]){
    minPoint[0] = point1[0] - 0.01;
    maxPoint[0] = tf[0] + 0.01;
  }

  if(tf[1] < point1[1]){
    minPoint[1] = tf[1] - 0.01;
    maxPoint[1] = point1[1] + 0.01;
  } else if (tf[1] > point1[1]){
    minPoint[1] = point1[1] - 0.01;
    maxPoint[1] = tf[1] + 0.01;
  }

  if(tf[2] < point1[2]){
    minPoint[2] = tf[2] - 0.01;
    maxPoint[2] = point1[2] + 0.01;
  } else if (tf[2] > point1[2]){
    minPoint[2] = point1[2] - 0.01;
    maxPoint[2] = tf[2] + 0.01;
  }

  std::cout << "minpoint : " << minPoint[0] << ", " << minPoint[1] << ", " << minPoint[2] << std::endl;
  std::cout << "maxpoint : " << maxPoint[0] << ", " << maxPoint[1] << ", " << maxPoint[2] << std::endl;
  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = 0;
  boxTranslatation[1] = 0;
  boxTranslatation[2] = 0;

  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0; // rotation around x-axis
  boxRotation[1] = 0; // rotation around y-axis
  boxRotation[2] = 0; // in radians rotation around z-axis. this rotates your

  Eigen::Affine3f boxTransform;

  pcl::CropBox<pcl::PointXYZ> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);
  cropFilter.setRotation(boxRotation);

  cropFilter.filter(*cloud_filtered);
  std::cout << cloud_filtered->points.size() << std::endl;
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& sensor_cloud)
{
  // 実験によりoffsetの値を算出
  double offset1 = -0.05;
  double offset2 = 0.025;

  if (count == 2 && poseobj.time + ros::Duration(offset1) < sensor_cloud->header.stamp && poseobj.time + ros::Duration(offset2) > sensor_cloud->header.stamp)
  {
    count = 0;
    std::cout << "ROS_time(record_cloud) : " << ros::Time::now() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*sensor_cloud, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // nanを除去
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

    crop_cloud(cloud, cloud_filtered);

    if (cloud_filtered->points.size() > 300){
      t += 1;
      get_tf(); //save tf data

      std::stringstream ss;
      ss << t;
      std::string filename = "cloud_" + ss.str() + ".pcd";
      pcl::io::savePCDFileASCII (filename, *cloud_filtered);
    //  std::cout << filename << std::endl;
    //  std::cerr << "Saved " << cloud_filtered->points.size () << " data points to save.pcd." << std::endl;
    }
  }else if (poseobj.time + ros::Duration(offset2) < sensor_cloud->header.stamp)
  {
    count = 0;
  }
}

int
main (int argc, char** argv)
{
  int max_data = atoi(argv[2]);
  // Initialize ROS
  ros::init (argc, argv, "pcd_record");
  //  ros::NodeHandle n;
  ros::NodeHandle nh;

//  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "input", 1);
//  message_filters::Subscriber<geometry_msgs::PoseStamped> sub2(nh, "/posestamped_obj", 1);
//  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
//  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), sub1, sub2); // mysyncpolicy(queu_size)
//  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  ros::Subscriber sub1 = nh.subscribe("input", 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe("/posestamped_obj", 1, pose_cb);
  ros::Subscriber sub3 = nh.subscribe("/principal", 1, principal_cb);

  ros::Rate loop_rate(1000);
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    if (t == max_data)
    {
      break;
    }
  }

  // Create a ROS subscriber for the input point cloud
//  ros::Subscriber sub1 = nh.subscribe ("/posestamped_obj", 1, get_tf);
  std::cout << "finish to record (" << t << " files)!!" << std::endl;
}
