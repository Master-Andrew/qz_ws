#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

#include "common.h"


void DoICP(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligen, Eigen::Matrix4f transformation,
  double max_cor_distance = 1.0, int max_iter = 50)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(1.0);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(1);

  icp.align(*aligen);

  transformation = icp.getFinalTransformation();
}

void DoNDT(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligen, Eigen::Matrix4f transformation,
  double resolution = 1.0, int max_iter = 50)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);

  approximate_voxel_filter.setInputCloud(source);
  approximate_voxel_filter.filter(*source);

  approximate_voxel_filter.setInputCloud(target);
  approximate_voxel_filter.filter(*target);

  cout << "approximate_voxel_filter" << endl;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.1);
  ndt.setStepSize(0.5);
  ndt.setResolution(resolution);
  ndt.setMaximumIterations(max_iter);
  ndt.setInputSource(source);
  ndt.setInputTarget(target);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*aligen);
  transformation = ndt.getFinalTransformation();

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
    << " score: " << ndt.getFitnessScore() << std::endl;
}

// Eigen::Matrix4f initial_estimate = init_transform.mat().cast<float>();

// http://pointclouds.org/documentation/classpcl_1_1registration_1_1_transformation_estimation_point_to_plane.html
void PclPointToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
  pcl::PointCloud<pcl::PointXYZ>::Ptr target,
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligen, Eigen::Matrix4f transformation,
  double dis_thres = 1.0, int kMaxIteration = 20)
{

  //点云初始化
  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*target, *target_cloud_normals);

  //法向估计
  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> normal_estimation;
  normal_estimation.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(
    new pcl::search::KdTree<pcl::PointNormal>));

  normal_estimation.setKSearch(20);
  normal_estimation.setInputCloud(target_cloud_normals);
  normal_estimation.setViewPoint(0, 0, 10);
  normal_estimation.compute(*target_cloud_normals);
  cout << "normal_estimation " << endl;

  //对应关系
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
  int iteration = 0;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // Convergence criteria

  //收敛准则初始化
  pcl::registration::DefaultConvergenceCriteria<float> convergence_criteria(
    iteration, transform, *correspondences);
  convergence_criteria.setMaximumIterations(kMaxIteration);
  convergence_criteria.setMaximumIterationsSimilarTransforms(5);
  convergence_criteria.setRelativeMSE(1e-5);
  cout << "convergence_criteria" << endl;

  // Correspondence estimation
  boost::shared_ptr<pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>>
    cor_est_closest_point(
      new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>());
  cor_est_closest_point->setInputTarget(target);
  cout << "cor_est_closest_point" << endl;

  // Transformation estimation
  pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZ, pcl::PointNormal>
    transformation_estimation;


  const float correspondence_threshold = dis_thres;
  do
  {
    cor_est_closest_point->setInputSource(source);
    cor_est_closest_point->determineCorrespondences(*correspondences,
      correspondence_threshold);

    transform.setIdentity();
    transformation_estimation.estimateRigidTransformation(
      *source, *target_cloud_normals, *correspondences, transform);

    pcl::transformPointCloud(*source, *aligen, transform);

    iteration++;
  } while (
    !convergence_criteria.hasConverged());
}


void DoRegistration(int argc, char** argv,
  void(*p)(pcl::PointCloud<pcl::PointXYZ>::Ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix4f,
    double, int))
{
  float x, y, z, yaw, pitch, roll;
  x = y = z = yaw = pitch = roll = 0.0;

  if (argc < 3)
  {
    cout << "please input two pcd file name!!!";
    exit(0);
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

  string init_merge_pcl_file = pcd_file_2.substr(0, pcd_file_2.size() - 4) + "_init_merge.pcd";
  pcl::io::savePCDFileASCII(init_merge_pcl_file, *pc_pcl_init_merge);
  cout << "save pcd init merge file : " << init_merge_pcl_file << endl;
  system(("pcl_viewer " + init_merge_pcl_file).c_str());

  Eigen::Matrix4f transformation;

  // do registration
  (*p)(pc_pcl_2, pc_pcl_1, pc_pcl_2_aligen, transformation, 1.0, 20);
  // do registration

  string aligen_pcl_file = pcd_file_2.substr(0, pcd_file_2.size() - 4) + "_aligen.pcd";
  pcl::io::savePCDFileASCII(aligen_pcl_file, *pc_pcl_2_aligen);
  cout << "save pcd aligen  file : " << aligen_pcl_file << endl;
  // system(("pcl_viewer " + aligen_pcl_file).c_str());

  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pp_list_2 = { pc_pcl_1, pc_pcl_2_aligen };
  pc_pcl_aligen_merge = MergePcd(pp_list_2);
  string aligen_merge_pcl_file = pcd_file_2.substr(0, pcd_file_2.size() - 4) + "_aligen_merge.pcd";
  pcl::io::savePCDFileASCII(aligen_merge_pcl_file, *pc_pcl_aligen_merge);
  cout << "save pcd file : " << aligen_merge_pcl_file << endl;
  system(("pcl_viewer " + aligen_merge_pcl_file).c_str());
}


