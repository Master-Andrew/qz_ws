#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "common.h"

using namespace std;

int main(int argc, char** argv)
{
  float x, y, z, yaw, pitch, roll;
  x = y = z = yaw = pitch = roll = 0.0;

  if (argc < 3)
  {
    cout << "please input two pcd file name!!!";
    return 1;
  }

  if (argc > 8)
  {
    x = atof(argv[3]);
    y = atof(argv[4]);
    z = atof(argv[5]);
    yaw = atof(argv[6]);
    pitch = atof(argv[7]);
    roll = atof(argv[8]);
    cout << "xyz, ypr" << x << " " << y << " " << z << " "
      << yaw << " " << pitch << " " << roll << endl;
  }

  string pcd_file_1 = argv[1];
  string pcd_file_2 = argv[2];
  string pcd_file_merge = argv[3];

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl_1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl_2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pcl_init_merge(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl_2_aligen(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pcl_aligen_merge(new pcl::PointCloud<pcl::PointXYZI>);

  pc_pcl_1 = LoadPcd(pcd_file_1);
  pc_pcl_2 = LoadPcd(pcd_file_2);

  Eigen::Matrix4f init_rt;
  GetTransform(x, y, z, yaw, pitch, roll, init_rt);
  pcl::transformPointCloud(*pc_pcl_2, *pc_pcl_2, init_rt);

  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pp_list = { pc_pcl_1, pc_pcl_2 };
  pc_pcl_init_merge = MergePcd(pp_list);

  string init_merge_pcl_file = pcd_file_2.substr(0, pcd_file_2.size() - 4) + "_init_merge_ndt.pcd";
  pcl::io::savePCDFileASCII(init_merge_pcl_file, *pc_pcl_init_merge);
  cout << "save pcd init merge file : " << init_merge_pcl_file << endl;


  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);

  approximate_voxel_filter.setInputCloud(pc_pcl_1);
  approximate_voxel_filter.filter(*pc_pcl_1);

  approximate_voxel_filter.setInputCloud(pc_pcl_2);
  approximate_voxel_filter.filter(*pc_pcl_2);

  cout << "approximate_voxel_filter" << endl;

  Eigen::Matrix4f  transformation;
  DoNDT(pc_pcl_2, pc_pcl_1, pc_pcl_2_aligen, transformation, 1.0, 50);


  string aligen_pcl_file = pcd_file_2.substr(0, pcd_file_2.size() - 4) + "_aligen_ndt.pcd";
  pcl::io::savePCDFileASCII(aligen_pcl_file, *pc_pcl_2_aligen);
  cout << "save pcd aligen  file : " << aligen_pcl_file << endl;

  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pp_list_2 = { pc_pcl_1, pc_pcl_2_aligen };
  pc_pcl_aligen_merge = MergePcd(pp_list_2);
  string aligen_merge_pcl_file = pcd_file_2.substr(0, pcd_file_2.size() - 4) + "_aligen_merge_ndt.pcd";
  pcl::io::savePCDFileASCII(aligen_merge_pcl_file, *pc_pcl_aligen_merge);
  cout << "save pcd file : " << aligen_merge_pcl_file << endl;
}

