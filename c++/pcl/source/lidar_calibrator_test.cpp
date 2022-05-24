#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

//http://pointclouds.org/documentation/classpcl_1_1registration_1_1_transformation_estimation_point_to_plane.html


using namespace std;

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudTP = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using PointN = pcl::PointNormal;
using PointCloudN = pcl::PointCloud<pcl::PointNormal>;
using PointCloudNP = pcl::PointCloud<pcl::PointNormal>::Ptr;

int kMaxIteration = 10;
double dis_thres = 1.0;

Eigen::Matrix4f PclPointToPlane(const PointCloudTP ref_cloud,
  const PointCloudTP src_cloud,
  const Eigen::Matrix4f& init_transform,
  double dis_thres) {

  //点云初始化
  PointCloudTP source_aligned(new PointCloudT);
  PointCloudNP ref_cloud_normals(new PointCloudN);
  pcl::copyPointCloud(*ref_cloud, *ref_cloud_normals);

  //法向估计
  pcl::NormalEstimation< PointN, PointN> normal_estimation;
  normal_estimation.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(
    new pcl::search::KdTree<pcl::PointNormal>));

  normal_estimation.setKSearch(20);
  normal_estimation.setInputCloud(ref_cloud_normals);
  normal_estimation.setViewPoint(0, 0, 10);
  normal_estimation.compute(*ref_cloud_normals);
  cout << "normal_estimation" << endl;

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
  boost::shared_ptr<
    pcl::registration::CorrespondenceEstimation<PointT, PointT>>
    cor_est_closest_point(
      new pcl::registration::CorrespondenceEstimation<PointT, PointT>());
  cor_est_closest_point->setInputTarget(ref_cloud);
  cout << "cor_est_closest_point" << endl;

  // Transformation estimation
  pcl::registration::TransformationEstimationPointToPlane<PointT, PointN>
    transformation_estimation;

  pcl::transformPointCloud(*src_cloud, *source_aligned, init_transform);
  Eigen::Matrix4f final_transform = init_transform;
  cout << "transformPointCloud" << endl;

  const float correspondence_threshold = dis_thres;
  do {
    cor_est_closest_point->setInputSource(source_aligned);
    cor_est_closest_point->determineCorrespondences(*correspondences,
      correspondence_threshold);

    transform.setIdentity();
    transformation_estimation.estimateRigidTransformation(
      *source_aligned, *ref_cloud_normals, *correspondences, transform);
    final_transform = transform * final_transform;

    pcl::transformPointCloud(*src_cloud, *source_aligned, final_transform);

    iteration++;
  } while (
    !convergence_criteria.hasConverged());

  cout << "done" << endl;
  return final_transform;
}




int main(int argc, char** argv)
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
    return(-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_2, *pc_pcl_2) == -1)
  {
    PCL_ERROR("Couldn't read file \n");
    return(-1);
  }

  cout << "loaded " << pcd_file_1 << " pcd, get " << pc_pcl_1->size() << " points!!" << endl;
  cout << "loaded " << pcd_file_2 << " pcd, get " << pc_pcl_2->size() << " points!!" << endl;

  std::vector<int> indices1;
  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*pc_pcl_1, *pc_pcl_1, indices1);
  pcl::removeNaNFromPointCloud(*pc_pcl_2, *pc_pcl_2, indices2);

  cout << "loaded " << pcd_file_1 << " pcd, get " << pc_pcl_1->size() << " points!!" << endl;
  cout << "loaded " << pcd_file_2 << " pcd, get " << pc_pcl_2->size() << " points!!" << endl;

  Eigen::AngleAxisf rotation(-2.325, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f translation(0, 0, 2);
  Eigen::Matrix4f init_rt = (translation * rotation).matrix();

  PointCloudTP pc_pcl_tmp(new PointCloudT);
  pcl::transformPointCloud(*pc_pcl_2, *pc_pcl_tmp, init_rt);

  *pc_pcl_tmp += *pc_pcl_1;

  pcl::io::savePCDFileASCII("tmp.pcd", *pc_pcl_tmp);
  cout << "save pcd file : tmp.pcd" << endl;


  PointCloudTP pc_pcl_2_aligen(new PointCloudT);


  Eigen::Matrix4f transform = PclPointToPlane(pc_pcl_1, pc_pcl_2, init_rt, dis_thres);
  cout << "transform 1" << endl;



  pcl::transformPointCloud(*pc_pcl_2, *pc_pcl_2_aligen, transform);
  cout << "transformPointCloud" << endl;

  pcl::PointXYZI pointi;
  for (PointT& point : *pc_pcl_1)
  {
    pointi.x = point.x;
    pointi.y = point.y;
    pointi.z = point.z;
    pointi.intensity = 1;
    pc_pcl_3->push_back(pointi);
  }
  cout << "pc_pcl_3" << endl;

  for (PointT& point : *pc_pcl_2_aligen)
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


  transform = PclPointToPlane(pc_pcl_1, pc_pcl_2, transform, dis_thres);
  cout << "transform 2" << endl;

  pcl::transformPointCloud(*pc_pcl_2, *pc_pcl_2_aligen, transform);
  cout << "transformPointCloud" << endl;

  pcl::io::savePCDFileASCII("aligen_plane.pcd", *pc_pcl_2_aligen);
  cout << "save pcd file : " << endl;


  for (PointT& point : *pc_pcl_1)
  {
    pointi.x = point.x;
    pointi.y = point.y;
    pointi.z = point.z;
    pointi.intensity = 1;
    pc_pcl_3->push_back(pointi);
  }
  cout << "pc_pcl_3" << endl;

  for (PointT& point : *pc_pcl_2_aligen)
  {
    pointi.x = point.x;
    pointi.y = point.y;
    pointi.z = point.z;
    pointi.intensity = 2;
    pc_pcl_4->push_back(pointi);
  }
  cout << "pc_pcl_4" << endl;

  *pc_pcl_3 += *pc_pcl_4;

  pcl::io::savePCDFileASCII("tmp_3.pcd", *pc_pcl_3);
  cout << "save pcd file : " << endl;

  return (0);
}

//Eigen::Matrix4f initial_estimate = init_transform.mat().cast<float>();
