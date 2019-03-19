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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/TransformStamped.h>

#include "pcl_ros/transforms.h"
#include <fstream>
#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
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

PoseObj poseobj;

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
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& sensor_cloud)
{
  // 実験によりoffsetの値を算出
  double offset1 = -0.05;
  double offset2 = 0.025;

  if (count == 1 && poseobj.time + ros::Duration(offset1) < sensor_cloud->header.stamp && poseobj.time + ros::Duration(offset2) > sensor_cloud->header.stamp)
  {
    count = 0;
    t += 1;
    get_tf();
    std::cout << "ROS_time(record_cloud) : " << ros::Time::now() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*sensor_cloud, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  //  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  //  pass.setInputCloud (cloud);
  //  pass.setFilterFieldName ("z");
  //  pass.setFilterLimits(0.01,0.6);
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
