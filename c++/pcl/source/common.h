#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr LoadPcd(string pcd_file, bool remove_nan = true)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pc) == -1)
  {
    PCL_ERROR("Couldn't read file %s\n", pcd_file);
    return (pc);
  }

  // cout << "loaded " << pcd_file << " pcd, get " << pc->size() << " points!!" << endl;

  if (remove_nan)
  {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc, *pc, indices);
  }

  // cout << "filter nan " << pcd_file << " pcd, get " << pc->size() << " points!!" << endl;

  return pc;
}

template <typename P>
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

void GetTransform(const float x, const float y, const float z,
  const float yaw, const float pitch, const float roll, Eigen::Matrix4f& translation)
{
  translation = (Eigen::Translation3f(x, y, z) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).matrix();
}

// https://blog.csdn.net/lemonxiaoxiao/article/details/123596114
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

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor(1.0, 0.5, 1.0);
  pcl::PointXYZ o;
  o.x = 1.0;
  o.y = 0;
  o.z = 0;
  viewer.addSphere(o, 0.25, "sphere", 0);
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
  static unsigned count = 0;
  std::stringstream ss;
  ss << "Once per viewer loop: " << count++;
  viewer.removeShape("text", 0);
  viewer.addText(ss.str(), 200, 300, "text", 0);
  // FIXME: possible race condition here:
  // user_data++;
}

template <typename T>
void showPointcloud(T input, string windows_name)
{
  pcl::visualization::CloudViewer viewer(windows_name);
  viewer.showCloud(input);
  // viewer.runOnVisualizationThreadOnce(viewerOneOff);
  // viewer.runOnVisualizationThread(viewerPsycho);
  while (!viewer.wasStopped())
  {
  }
}
