// Save point cloud in specified coordinate system in csv format

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<Eigen/StdVector>
#include <fstream>

#define FILE_N 50000

const std::string FileName("pc_pos_ori_10000.hdf5");
const std::string DatasetName("pcandtf");
const std::string member_pc("pc");
const std::string member_tf("tf");


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

void translation(float x, float y, float z, float diff, float*arr, float transed[7])
{
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

int main (int argc, char** argv)
{
  std::cout << "start!" << std::endl;
  std::cout << "now converting..." << std::endl;
  for(int n = 1; n < atoi(argv[1])+1; n++){
//    std::cout << "n : " << n << std::endl;
    char coo[256] = {0};

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr normarized_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Replace the path below with the path where you saved your file
    char filename[100];
    sprintf(filename, "cloud_%d.pcd", n);
    pcl::io::loadPCDFile(filename, *cloud);
    pcl::io::loadPCDFile(filename, *normarized_cloud);

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

    // processing tf
    char tf_file[100];

    sprintf(tf_file, "tf_%d.csv", n);

    std::ifstream ifs;  // ファイル読み取り用ストリーム
    ifs.open(tf_file);
    if(ifs.fail()){
      std::cerr << "cannot open file_"<< n << "!!\n";
      continue;
    }
    float arr[7] = {0};
    for(int i=0 ; i<7 ; i++){
      ifs.getline(coo,sizeof(coo));	// 一行読み込んで…
      arr[i] = atof(coo);	// それを配列に格納
    }
    // for validation
//    std::cout << min_x  << " : " <<  max_x << std::endl;
//    std::cout << min_y  << " : " <<  max_y << std::endl;
//    std::cout << min_z  << " : " <<  max_z << std::endl;
//    std::cout << (arr[0]-min_x)/diff_max << std::endl;
//    std::cout << (arr[1]-min_y)/diff_max << std::endl;
//    std::cout << (arr[2]-min_z)/diff_max << std::endl;
    if (
    ((arr[0]-min_x)/diff_max < -0.2) || ((arr[0]-min_x)/diff_max > 1.2) ||
    ((arr[1]-min_y)/diff_max < -0.2) || ((arr[1]-min_y)/diff_max > 1.2) ||
    ((arr[2]-min_z)/diff_max < -0.2) || ((arr[2]-min_z)/diff_max > 1.2)){
      std::cout << "origin position is out of vocel : " << n << std::endl;
      continue;
//      break;
    }
    float transed_tf[7] = {0};
    translation(min_x, min_y, min_z, diff_max, arr, transed_tf);
    std::cout << "m : " << m << std::endl;

    // processing to normalize cloud
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      normarized_cloud->points[i].x = (cloud->points[i].x - min_x)/diff_max;
      normarized_cloud->points[i].y = (cloud->points[i].y - min_y)/diff_max;
      normarized_cloud->points[i].z = (cloud->points[i].z - min_z)/diff_max;
    }

    //save point cloud
    char newfile[100];
    sprintf(newfile, "learn_data/normarized_pc%d.csv", m);

    FILE *fp;
    fp = fopen(newfile, "w");

    for(int i = 0; i < normarized_cloud->points.size(); i++){
          fprintf(fp, "%f,%f,%f\n", normarized_cloud->points[i].x, normarized_cloud->points[i].y, normarized_cloud->points[i].z);
    }

    fclose(fp);
    m += 1;

//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    viewer.showCloud(new_cloud);
//    viewer.runOnVisualizationThreadOnce (viewerOneOff);
//    viewer.runOnVisualizationThread (viewerPsycho);
//    while (!viewer.wasStopped ())
//    {
//    user_data++;
//    }
//
  }

  std::cout << "finish !!" << std::endl;
  return (0);
}
