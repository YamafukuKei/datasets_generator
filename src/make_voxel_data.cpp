#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<Eigen/StdVector>
#include <fstream>

#define VOXEL_N 50
#define FILE_N 10000

int ratio = 1;  //number of pointcloud
float ratio_size = 0.001; //object_size
int voxel[VOXEL_N][VOXEL_N][VOXEL_N] = {0};
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

void make_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float diff)
{
  for (int i = 0; i < VOXEL_N; i++){
    float min_x = i*diff;
    float max_x = (i+1)*diff;
    for (int j = 0; j < VOXEL_N; j++){
      float min_y = j*diff;
      float max_y = (j+1)*diff;
      for (int k = 0; k < VOXEL_N; k++){
        float min_z = k*diff;
        float max_z = (k+1)*diff;
        for (int n = 0; n < VOXEL_N; n++){
          if (cloud->points[n].x > min_x && cloud->points[n].x < max_x &&
              cloud->points[n].y > min_y && cloud->points[n].y < max_y &&
              cloud->points[n].z > min_z && cloud->points[n].z < max_z ){
            voxel[i][j][k] = 1;
            break;
          }
        }
      }
    }
  }
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

void translation(float x, float y, float z, float diff, float*arr)
{
//  std::cout << arr[0] << arr[1] << arr[2] << std::endl;
  float transed[7] = {0};
  transed[0] = (arr[0] - x) / diff;
  transed[1] = (arr[1] - y) / diff;
  transed[2] = (arr[2] - z) / diff;
  transed[3] = arr[3];
  transed[4] = arr[4];
  transed[5] = arr[5];
  transed[6] = arr[6];

//  std::cout << transed[0] << transed[1] << transed[2] << std::endl;
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
  for(int n = 1; n < atoi(argv[1]); n++){

    char tf_file[100];
    sprintf(tf_file, "tf_%d.csv", n+10);
    std::ifstream ifs;  // ファイル読み取り用ストリーム
    ifs.open(tf_file);
    if(ifs.fail()){	// ファイルオープンに失敗したらそこで終了
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

    float max_x = Min_Max(cloud).max_x;
    float max_y = Min_Max(cloud).max_y;
    float max_z = Min_Max(cloud).max_z;
    float min_x = Min_Max(cloud).min_x;
    float min_y = Min_Max(cloud).min_y;
    float min_z = Min_Max(cloud).min_z;

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

    for (size_t i = 0; i+1 < cloud->points.size(); ++i)
    {
      new_cloud->points[i].x = (cloud->points[i].x - min_x)/diff_max;
      new_cloud->points[i].y = (cloud->points[i].y - min_y)/diff_max;
      new_cloud->points[i].z = (cloud->points[i].z - min_z)/diff_max;
    }

  //  float leafsize = diff_max / VOXEL_N;
  //  std::cout << "leaf_size = " << leafsize << std::endl;

    make_voxel(new_cloud, 1/VOXEL_N);

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
  translation(min_x, min_y, min_z, diff_max, arr);

  m += 1;
  }

  std::cout << "finish !!" << std::endl;
  return (0);
}
