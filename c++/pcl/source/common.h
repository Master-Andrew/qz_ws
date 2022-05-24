#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <string>
#include <vector>


using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr LoadPcd(string pcd_file)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pc) == -1)
  {
    PCL_ERROR("Couldn't read file %s\n", pcd_file);
    return (pc);
  }

  cout << "loaded " << pcd_file << " pcd, get " << pc->size() << " points!!" << endl;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pc, *pc, indices);

  cout << "filter nan " << pcd_file << " pcd, get " << pc->size() << " points!!" << endl;

  return pc;
}

template<typename P>
pcl::PointCloud<pcl::PointXYZI>::Ptr MergePcd(vector<P> pp_list)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr merge_pp(new pcl::PointCloud<pcl::PointXYZI>);
  int index = 0;
  pcl::PointXYZI pointi;

  for (auto pp : pp_list)
  {
    for (auto& point : *pp)
    {
      pointi.x = point.x;
      pointi.y = point.y;
      pointi.z = point.z;
      pointi.intensity = index;
      merge_pp->push_back(pointi);
    }
    index += 1;
    cout << "merge pointcloud index : " << index << endl;
  }

  return merge_pp;
}

void GetTransform(const float x, const  float y, const  float z,
  const float yaw, const  float pitch, const  float roll, Eigen::Matrix4f& translation)
{
  translation = (Eigen::Translation3f(x, y, z)
    * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).matrix();
}

//https://blog.csdn.net/lemonxiaoxiao/article/details/123596114
void GetPose(const Eigen::Matrix4f translation, float& x, float& y, float& z,
  float& yaw, float& pitch, float& roll)
{
  x = translation(0, 3);
  y = translation(1, 3);
  z = translation(2, 3);
  Eigen::Matrix3f rotation = translation.block(0, 0, 3, 3);
  Eigen::Vector3f euler_angle = rotation.eulerAngles(2, 1, 0);
  yaw = euler_angle(0);
  pitch = euler_angle(1);
  roll = euler_angle(2);
}

void DoICP(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligen, Eigen::Matrix4f  transformation,
  float max_cor_distance = 1.0, int max_iter = 50)
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligen, Eigen::Matrix4f  transformation,
  float resolution = 1.0, int max_iter = 50)
{
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
