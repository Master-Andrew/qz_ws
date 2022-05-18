#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

using namespace std;

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudTP = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using PointN = pcl::PointNormal;
using PointCloudN = pcl::PointCloud<pcl::PointNormal>;
using PointCloudNP = pcl::PointCloud<pcl::PointNormal>::Ptr;

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    cout << "please input two pcd file name!!!";
    return 1;
  }

  string pcd_file_1 = argv[1];
  string pcd_file_2 = argv[2];
  string pcd_file_merge = argv[3];

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl_1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl_2(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pcl_3(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_pcl_4(new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_1, *pc_pcl_1) == -1)
  {
    PCL_ERROR("Couldn't read file \n");
    return (-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_2, *pc_pcl_2) == -1)
  {
    PCL_ERROR("Couldn't read file \n");
    return (-1);
  }

  cout << "loaded " << pcd_file_1 << " pcd, get " << pc_pcl_1->size() << " points!!" << endl;
  cout << "loaded " << pcd_file_2 << " pcd, get " << pc_pcl_2->size() << " points!!" << endl;

  std::vector<int> indices1;
  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*pc_pcl_1, *pc_pcl_1, indices1);
  pcl::removeNaNFromPointCloud(*pc_pcl_2, *pc_pcl_2, indices2);

  cout << "loaded " << pcd_file_1 << " pcd, get " << pc_pcl_1->size() << " points!!" << endl;
  cout << "loaded " << pcd_file_2 << " pcd, get " << pc_pcl_2->size() << " points!!" << endl;

  Eigen::AngleAxisf rotation(0.99, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f translation(0.2, 0.2, 0.2);
  Eigen::Matrix4f init_rt = (translation * rotation).matrix();

  PointCloudTP pc_pcl_tmp(new PointCloudT);
  pcl::transformPointCloud(*pc_pcl_2, *pc_pcl_tmp, init_rt);

  *pc_pcl_tmp += *pc_pcl_1;
  // *pc_pcl_tmp += *pc_pcl_2;

  pcl::io::savePCDFileASCII("tmp.pcd", *pc_pcl_tmp);
  cout << "save pcd file : tmp.pcd" << endl;

  PointCloudTP pc_pcl_2_aligen(new PointCloudT);

  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // icp.setInputCloud(pc_pcl_2);
  // icp.setInputTarget(pc_pcl_1);

  // // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  // icp.setMaxCorrespondenceDistance(1.0);
  // // Set the maximum number of iterations (criterion 1)
  // icp.setMaximumIterations(50);
  // // Set the transformation epsilon (criterion 2)
  // icp.setTransformationEpsilon(1e-8);
  // // Set the euclidean distance difference epsilon (criterion 3)
  // icp.setEuclideanFitnessEpsilon(1);

  // icp.align(*pc_pcl_2_aligen);
  // Eigen::Matrix4f transformation = icp.getFinalTransformation();

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);

  approximate_voxel_filter.setInputCloud(pc_pcl_1);
  approximate_voxel_filter.filter(*pc_pcl_1);

  approximate_voxel_filter.setInputCloud(pc_pcl_2);
  approximate_voxel_filter.filter(*pc_pcl_2);

  cout << "approximate_voxel_filter" << endl;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.1);
  ndt.setStepSize(0.5);
  ndt.setResolution(1.0);
  ndt.setMaximumIterations(35);
  ndt.setInputSource(pc_pcl_2);
  ndt.setInputTarget(pc_pcl_1);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*pc_pcl_2_aligen, init_rt);
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << std::endl;

  pcl::PointXYZI pointi;
  for (PointT &point : *pc_pcl_1)
  {
    pointi.x = point.x;
    pointi.y = point.y;
    pointi.z = point.z;
    pointi.intensity = 1;
    pc_pcl_3->push_back(pointi);
  }
  cout << "pc_pcl_3" << endl;

  for (PointT &point : *pc_pcl_2_aligen)
  {
    pointi.x = point.x;
    pointi.y = point.y;
    pointi.z = point.z;
    pointi.intensity = 2;
    pc_pcl_4->push_back(pointi);
  }
  cout << "pc_pcl_4" << endl;

  *pc_pcl_3 += *pc_pcl_4;

  pcl::io::savePCDFileASCII("tmp_2.pcd", *pc_pcl_3);
  cout << "save pcd file : " << endl;

  return (0);
}

// Eigen::Matrix4f initial_estimate = init_transform.mat().cast<float>();
