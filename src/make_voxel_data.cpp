#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<Eigen/StdVector>
#include <fstream>
#include "H5Cpp.h"

#define VOXEL_N 50
#define FILE_N 50000

const std::string FileName("voxel_pos_ori_23500.hdf5");
const std::string DatasetName("voxelandtf");
const std::string member_voxel("voxel");
const std::string member_tf("tf");
//#include <pcl/visualization/cloud_viewer.h>
//#include <iostream>
//#include <pcl/io/io.h>
//
//int user_data;
//
//void
//viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
//{
//    viewer.setBackgroundColor (0.3, 0.3, 0.3);
//    pcl::PointXYZ o;
//    o.x = 1.0;
//    o.y = 0;
//    o.z = 0;
//    std::cout << "i only run once" << std::endl;
//
//}
//
//void
//viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
//{
//    static unsigned count = 0;
//    std::stringstream ss;
//    ss << "Once per viewer loop: " << count++;
//    viewer.removeShape ("text", 0);
//    viewer.addText (ss.str(), 200, 300, "text", 0);
//
//    //FIXME: possible race condition here:
//    user_data++;
//}



int m = 1;

class Size
{
public:
    double min_x;
    double min_y;
    double min_z;
    double max_x;
    double max_y;
    double max_z;
};

Size Min_Max(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Size size;
    size.max_x = cloud->points[0].x;
    size.max_y = cloud->points[0].y;
    size.max_z = cloud->points[0].z;
    size.min_x = cloud->points[0].x;
    size.min_y = cloud->points[0].y;
    size.min_z = cloud->points[0].z;

    for(size_t i = 0; i < cloud->points.size(); i++){
        if(size.max_z > cloud->points[i].z)
            size.max_z = size.max_z;
        else
            size.max_z = cloud->points[i].z;

        if(size.max_y > cloud->points[i].y)
            size.max_y = size.max_y;
        else
            size.max_y = cloud->points[i].y;

        if(size.max_x > cloud->points[i].x)
            size.max_x = size.max_x;
        else
            size.max_x = cloud->points[i].x;

        if(size.min_z < cloud->points[i].z)
            size.min_z = size.min_z;
        else
            size.min_z = cloud->points[i].z;

        if(size.min_y < cloud->points[i].y)
            size.min_y = size.min_y;
        else
            size.min_y = cloud->points[i].y;

        if(size.min_x < cloud->points[i].x)
            size.min_x = size.min_x;
        else
            size.min_x = cloud->points[i].x;
    }
    return size;
}

//int counter(int hako[VOXEL_N][VOXEL_N][VOXEL_N])
//{
//  int ct = 0;
//  for (int i = 0; i < VOXEL_N; i++){
//    for (int j = 0; j < VOXEL_N; j++){
//      for (int k = 0; k < VOXEL_N; k++){
//        for (int n = 0; n < VOXEL_N; n++){
//          if (voxel[i][j][k] == 1){
//            ct += 1;
//          }
//        }
//      }
//    }
//  }
//  return ct;
//}
void make_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resol, int voxel[VOXEL_N][VOXEL_N][VOXEL_N])
{
//   int ct = 0;
  for (int i = 0; i < VOXEL_N; i++){
    float min_x = i*resol;
    float max_x = (i+1)*resol;
    for (int j = 0; j < VOXEL_N; j++){
      float min_y = j*resol;
      float max_y = (j+1)*resol;
      for (int k = 0; k < VOXEL_N; k++){
        float min_z = k*resol;
        float max_z = (k+1)*resol;
        for (int n = 0; n < cloud->points.size(); n++){
          if ((cloud->points[n].x > min_x) && (cloud->points[n].x < max_x) &&
              (cloud->points[n].y > min_y) && (cloud->points[n].y < max_y) &&
              (cloud->points[n].z > min_z) && (cloud->points[n].z < max_z)){
            voxel[i][j][k] = 1;
//            ct += 1;
            break;
          }
        }
      }
    }
  }
//  std::cout << ct << std::endl;
}

void translation(float x, float y, float z, float diff, float*arr, float transed[7])
{
//  std::cout << "min_x = " << x << std::endl;
//  std::cout << "min_y = " << y << std::endl;
//  std::cout << "min_z = " << z << std::endl;

//  std::cout << "before: " << arr[0] << "," << arr[1] << "," << arr[2] << std::endl;
  transed[0] = (arr[0] - x) / diff;
  transed[1] = (arr[1] - y) / diff;
  transed[2] = (arr[2] - z) / diff;
  transed[3] = arr[3];
  transed[4] = arr[4];
  transed[5] = arr[5];
  transed[6] = arr[6];

//  std::cout << "after : " << transed[0] << "," << transed[1] << "," << transed[2] << std::endl;
  char newfile[100];
  sprintf(newfile, "learn_data/tf_%d.csv", m);
  FILE *fp;
  fp = fopen(newfile, "w");
  int i = 0;
  for(i = 0; i < 7; i++){
    fprintf(fp, "%f\n", transed[i]);
  }
  fclose(fp);
}

void save_data(int voxel[VOXEL_N][VOXEL_N][VOXEL_N], float transed_tf[7])
{
//  PersonalInformation person_list[] = {
//      { 18, 'M', "Mary",  152.0   },
//      { 32, 'F', "Tom",   178.6   },
//      { 29, 'M', "Tarou", 166.6   }
//  };
//  int length = sizeof(person_list) / sizeof(PersonalInformation);
//  // the array of each length of multidimentional data.
//  hsize_t dim[1];
//  dim[0] = sizeof(person_list) / sizeof(PersonalInformation);
//
//  // the length of dim
//  int rank = sizeof(dim) / sizeof(hsize_t);
//
//  // defining the datatype to pass HDF55
//  H5::CompType mtype(sizeof(PersonalInformation));
//  mtype.insertMember(member_voxel, HOFFSET(PersonalInformation, age), H5::PredType::NATIVE_INT);
//  mtype.insertMember(member_tf, HOFFSET(PersonalInformation, height), H5::PredType::NATIVE_FLOAT);
//
//  // preparation of a dataset and a file.
//  H5::DataSpace space(rank, dim);
//  H5::H5File *file = new H5::H5File(FileName, H5F_ACC_TRUNC);
//  H5::Group *group = new H5::Group(file->createGroup(GroupName, mtype, space));
//  H5::DataSet *dataset = new H5::DataSet(group->createDataSet(DatasetName, mtype, space));
//  // Write
//  dataset->write(person_list, mtype);
//
//  delete dataset;
//  delete group;
//  delete file;
}

int main (int argc, char** argv)
{
  std::cout << "start!" << std::endl;
  std::cout << "now converting..." << std::endl;
  for(int n = 0; n < atoi(argv[1])-10; n++){
    char tf_file[100];
    sprintf(tf_file, "tf_%d.csv", n+10);
    std::ifstream ifs;  // ファイル読み取り用ストリーム
    ifs.open(tf_file);
    if(ifs.fail()){
      std::cerr << "cannot open file_"<< n << "!!\n";
      continue;
    }

    char coo[256] = {0};
    float arr[7] = {0};
    for(int i=0 ; i<7 ; i++){
      ifs.getline(coo,sizeof(coo));	// 一行読み込んで…
      arr[i] = atof(coo);	// それを配列に格納
    }

    float min_coordinate[3] = {0};
    float max_coordinate[3] = {0};

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Replace the path below with the path where you saved your file
    char filename[100];
    sprintf(filename, "cloud_%d.pcd", n+10);
    pcl::io::loadPCDFile(filename, *cloud);
    pcl::io::loadPCDFile(filename, *new_cloud);

    float max_x = Min_Max(cloud).max_x + 0.02;
    float max_y = Min_Max(cloud).max_y + 0.02;
    float max_z = Min_Max(cloud).max_z + 0.02;
    float min_x = Min_Max(cloud).min_x - 0.02;
    float min_y = Min_Max(cloud).min_y - 0.02;
    float min_z = Min_Max(cloud).min_z - 0.02;

    float diff_x = max_x - min_x;
    float diff_y = max_y - min_y;
    float diff_z = max_z - min_z;

//    std::cout << "diff_x = " << max_x << " - " << min_x << " = " << diff_x << std::endl;
//    std::cout << "diff_y = " << max_y << " - " << min_y << " = " << diff_y << std::endl;
//    std::cout << "diff_z = " << max_z << " - " << min_z << " = " << diff_z << std::endl;

    float diff_max = 0;
    diff_max = diff_x;
    if( diff_max < diff_y ) {
      diff_max = diff_y;
    }
    if( diff_max < diff_z ) {
      diff_max = diff_z;
    }

    if (
    ((arr[0]-min_x)/diff_max < 0) || ((arr[0]-min_x)/diff_max > 1) ||
    ((arr[1]-min_y)/diff_max < 0) || ((arr[1]-min_y)/diff_max > 1) ||
    ((arr[2]-min_z)/diff_max < 0) || ((arr[2]-min_z)/diff_max > 1)){
      continue;
    }

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      new_cloud->points[i].x = (cloud->points[i].x - min_x)/diff_max;
      new_cloud->points[i].y = (cloud->points[i].y - min_y)/diff_max;
      new_cloud->points[i].z = (cloud->points[i].z - min_z)/diff_max;
    }

    int voxel[VOXEL_N][VOXEL_N][VOXEL_N] = {0};
    make_voxel(new_cloud, (1/float(VOXEL_N)), voxel);

    char newfile[100];
    sprintf(newfile, "learn_data/voxel_%d.csv", m);
    FILE *fp;
    fp = fopen(newfile, "w");
    for(int i = 0; i < VOXEL_N; i++){
      for(int j = 0; j < VOXEL_N; j++){
        for(int k = 0; k < VOXEL_N; k++){
          fprintf(fp, "%d\n", voxel[i][j][k]);
        }
      }
    }
    fclose(fp);

    float transed_tf[7] = {0};
    translation(min_x, min_y, min_z, diff_max, arr, transed_tf);

    m += 1;

    save_data(voxel, transed_tf);

//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    viewer.showCloud(new_cloud);
//    viewer.runOnVisualizationThreadOnce (viewerOneOff);
//    viewer.runOnVisualizationThread (viewerPsycho);
//    while (!viewer.wasStopped ())
//    {
//    user_data++;
//    }
  //  ros::NodeHandle n;
//  ros::NodeHandle nh;
//
//  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "input", 1);
//  message_filters::Subscriber<geometry_msgs::PoseStamped> sub2(nh, "/posestamped_obj", 1);
//  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
//  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), sub1, sub2);
//  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));
////    ros::spin();
//
//  ros::Rate loop_rate(1000);
//  while (ros::ok()){
//    ros::spinOnce();
//    loop_rate.sleep();
//    if (t == max_data)
//    {
//      break;
//    }
//  }


  }

  std::cout << "finish !!" << std::endl;
  return (0);
}
