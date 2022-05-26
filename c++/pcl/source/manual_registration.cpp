#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h> // TicToc
#include <string>

#include "registration_common.h"

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool update = false;
bool exit_app = false;
float t_step = 0.01;
float r_step = 0.005;

float x, y, z, yaw, pitch, roll;

void print4x4Matrix(const Eigen::Matrix4d& matrix)
{
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym() == "y" && event.keyDown())
  {
    x -= t_step;
    update = true;
  }
  else if (event.getKeySym() == "u" && event.keyDown())
  {
    x += t_step;
    update = true;
  }

  else if (event.getKeySym() == "g" && event.keyDown())
  {
    y -= t_step;
    update = true;
  }
  else if (event.getKeySym() == "h" && event.keyDown())
  {
    y += t_step;
    update = true;
  }

  else if (event.getKeySym() == "v" && event.keyDown())
  {
    z -= t_step;
    update = true;
  }
  else if (event.getKeySym() == "b" && event.keyDown())
  {
    z += t_step;
    update = true;
  }

  else if (event.getKeySym() == "i" && event.keyDown())
  {
    yaw -= r_step;
    update = true;
  }
  else if (event.getKeySym() == "o" && event.keyDown())
  {
    yaw += r_step;
    update = true;
  }

  else if (event.getKeySym() == "j" && event.keyDown())
  {
    pitch -= r_step;
    update = true;
  }
  else if (event.getKeySym() == "k" && event.keyDown())
  {
    pitch += r_step;
    update = true;
  }

  else if (event.getKeySym() == "n" && event.keyDown())
  {
    roll -= r_step;
    update = true;
  }
  else if (event.getKeySym() == "m" && event.keyDown())
  {
    roll += r_step;
    update = true;
  }

  else if (event.getKeySym() == "enter" && event.keyDown())
  {
    exit_app = true;
  }
}

int main(int argc, char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr pc_target(new PointCloudT); // Original point cloud
  PointCloudT::Ptr pc_source(new PointCloudT); // Transformed point cloud
  PointCloudT::Ptr pc_aligen(new PointCloudT); // ICP output point cloud

  // Checking program arguments
  if (argc < 3)
  {
    printf("Usage :\n");
    printf("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR("Provide one ply file.\n");
    return (-1);
  }
  x = y = z = yaw = pitch = roll = 0.0;
  if (argc > 8)
  {
    x = atof(argv[3]);
    y = atof(argv[4]);
    z = atof(argv[5]);
    yaw = atof(argv[6]);
    pitch = atof(argv[7]);
    roll = atof(argv[8]);
    cout << "xyz, ypr : " << x << " " << y << " " << z << " "
      << yaw << " " << pitch << " " << roll << endl;
  }

  string pcd_target_file = argv[1];
  string pcd_source_file = argv[2];

  pc_target = LoadPcd(pcd_target_file);
  pc_source = LoadPcd(pcd_source_file);
  *pc_aligen = *pc_source;

  Eigen::Matrix4f translation;

  GetTransform(x, y, z, yaw, pitch, roll, translation);
  pcl::transformPointCloud(*pc_source, *pc_aligen, translation);

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

  // Visualization
  pcl::visualization::PCLVisualizer viewer("registration demo");
  // Create two vertically separated viewports
  int v1(0);
  int v2(1);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0; // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(pc_target, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
    (int)255 * txt_gray_lvl);
  viewer.addPointCloud(pc_target, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud(pc_target, cloud_in_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(pc_source, 20, 180, 20);
  viewer.addPointCloud(pc_source, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(pc_aligen, 180, 20, 20);
  viewer.addPointCloud(pc_aligen, cloud_icp_color_h, "cloud_icp_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "origin", v1);
  viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "registration", v2);

  std::stringstream ss;
  ss << "xyz, ypr" << x << " " << y << " " << z << " "
    << yaw << " " << pitch << " " << roll << endl;
  std::string translation_result = ss.str();
  viewer.addText(translation_result, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "translation_result", v2);

  // Set background color
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize(1280, 1024); // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

  string manual_aligen_file = pcd_source_file.substr(0, pcd_source_file.size() - 4) + "_manual_aligen.pcd";

  // Display the visualiser
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();

    // The user pressed "space" :
    if (update)
    {
      GetTransform(x, y, z, yaw, pitch, roll, translation);
      pcl::transformPointCloud(*pc_source, *pc_aligen, translation);

      pcl::io::savePCDFileASCII(manual_aligen_file, *pc_aligen);
      cout << "save pcd file : " << manual_aligen_file << endl;

      cout << "xyz, ypr : " << x << " " << y << " " << z << " "
        << yaw << " " << pitch << " " << roll << endl;

      ss.str("");
      ss << "xyz, ypr : " << x << " " << y << " " << z << " "
        << yaw << " " << pitch << " " << roll << endl;
      std::string translation_result = ss.str();
      viewer.updateText(translation_result, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "translation_result");
      viewer.updatePointCloud(pc_aligen, cloud_icp_color_h, "cloud_icp_v2");
    }
    update = false;

    if (exit_app)
      break;
  }
  return (0);
}
