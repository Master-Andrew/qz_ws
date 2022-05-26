#include "common.h"

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>

void Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
  float min_z, float max_z,
  pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
  // cout << "get points : " << input->size() << endl;
  pcl::PassThrough<pcl::PointXYZ > pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z, max_z);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*output);
  // cout << "outpu points : " << output->size() << endl;
}

double GetPlaneError(pcl::ModelCoefficients::Ptr coefficients, pcl::PointXYZ& point)
{
  return  coefficients->values[0] * point.x
    + coefficients->values[1] * point.y
    + coefficients->values[2] * point.z
    + coefficients->values[3];
}

void FindPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
  pcl::PointCloud<pcl::PointXYZ>::Ptr output, float threshold,
  pcl::ModelCoefficients::Ptr coefficients)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(threshold);
  seg.setInputCloud(input);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  seg.segment(*inliers, *coefficients);

  // cout << "segment done!!" << endl;

  // [normal_x normal_y  normal_z d]
  // ax+by+cz+d

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  vector<pcl::ModelCoefficients::Ptr> coefficients_list;
  coefficients_list.push_back(coefficients);
  // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
  //   << coefficients->values[1] << " "
  //   << coefficients->values[2] << " "
  //   << coefficients->values[3] << std::endl;

  // std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
  // for (size_t i = 0; i < inliers->indices.size(); ++i)
  //     std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
  //               << cloud->points[inliers->indices[i]].y << " "
  //               << cloud->points[inliers->indices[i]].z << std::endl;

  for (size_t i = 0; i < inliers->indices.size(); ++i)
  {
    pcl::PointXYZ& point = input->points[inliers->indices[i]];
    // point.intensity = abs(GetPlaneError(coefficients, point));
    output->push_back(point);
  }

}



void SplitM1PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& output_list)
{
  for (int i = 0; i < 5;i++)
  {
    output_list.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
  }

  for (int index = 0; index < input->size();index++)
  {
    int channel = index % 5;
    auto& point = input->points[index];
    output_list[channel]->push_back(point);
  }

  std::vector<int> indices;
  for (int i = 0; i < 5;i++)
  {
    pcl::removeNaNFromPointCloud(*output_list[i], *output_list[i], indices);
  }
}

void SaveSplitPointCloud(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& output_list,
  string file_name, string type)
{

  string output_file = "";
  for (int i = 0;i < output_list.size();i++)
  {
    auto& pp = output_list[i];
    if (pp->size() == 0)
    {
      continue;
    }

    int n1 = file_name.find_last_of('/') + 1;
    int n2 = file_name.find('.');
    int length = n2 - n1;
    string   output_file = file_name.substr(0, n1) + type + "/"
      + file_name.substr(n1, length) + "_" + to_string(i) + ".pcd";
    // cout << "save pcd file : " << output_file << endl;
    pcl::io::savePCDFileASCII(output_file, *pp);
    // cout << "save pcd file : " << output_file << endl;
  }
}

void FindAllPlane(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& input_list,
  float min_z, float max_z, float threshold,
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& output_list,
  vector<pcl::ModelCoefficients::Ptr>& coefficients_list)
{

  for (auto input : input_list)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_filted(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_plane(new pcl::PointCloud<pcl::PointXYZ>);

    Filter(input, min_z, max_z, input_filted);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    FindPlane(input_filted, input_plane, threshold, coefficients);
    coefficients_list.push_back(coefficients);

    output_list.push_back(input_plane);

  }
}

void PlaneRadiuSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr plane_1,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr plane_2, const float search_radius,
  std::vector<int>& source_index_list, std::vector<int>& target_index_list)
{
  for (int source_index = 0; source_index < plane_1->points.size();source_index++)
  {
    auto& source_point = plane_1->points[source_index];
    int target_index = 0;
    float min_radius = 100.;
    for (int search_index = 0; search_index < plane_2->points.size();search_index++)
    {
      auto& search_point = plane_2->points[search_index];
      float radius = sqrt((source_point.x - search_point.x)
        * (source_point.x - search_point.x) +
        (source_point.y - search_point.y)
        * (source_point.y - search_point.y));

      if (radius <= search_radius && radius < min_radius)
      {
        target_index = search_index;
        min_radius = radius;
      }
    }
    if (min_radius < search_radius) {
      source_index_list.push_back(source_index);
      target_index_list.push_back(target_index);
    }
  }
  // cout << source_index_list.size() << " " << target_index_list.size() << endl;

}

void CalculateGap(pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_1,
  pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_2,
  pcl::ModelCoefficients::Ptr& coefficients_2,
  pcl::PointCloud<pcl::PointXYZ>::Ptr& gap_point,
  float tolerance,
  vector<float>& gap_list)
{
  pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
  kdtree.setInputCloud(plane_2);

  std::vector<int>pointIdxNKNSearch;
  std::vector<float>pointNKNSquaredDistance;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  std::vector<int> source_index_list;
  std::vector<int> target_index_list;
  PlaneRadiuSearch(plane_1, plane_2, tolerance,
    source_index_list, target_index_list);

  for (int i = 0;i < source_index_list.size();i++)
  {
    auto& point = plane_1->points[source_index_list[i]];
    double gap = GetPlaneError(coefficients_2, point);
    gap_list.push_back(gap);
    gap_point->push_back(point);
    gap_point->push_back(plane_2->points[target_index_list[i]]);
  }


  // for (auto point : plane_1->points)
  // {
  //   if (kdtree.nearestKSearch(point, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  //   {
  //     for (int target_index : pointIdxNKNSearch)
  //     {
  //       auto target_point = plane_2->points[target_index];

  //     }

  //     double gap = GetPlaneError(coefficients_2, point);
  //     cout << gap << " " << pointNKNSquaredDistance[0] << " | ";
  //     if (abs(gap - sqrt(pointNKNSquaredDistance[0])) < tolerance)
  //     {
  //       cout << "true" << " ";
  //       cout << pointIdxNKNSearch[0] << " " << sqrt(pointNKNSquaredDistance[0]) << " |";
  //       gap_list.push_back(gap);
  //       gap_point->push_back(point);
  //       gap_point->push_back(plane_2->points[pointIdxNKNSearch[0]]);
  //     }
  //   }
  //   cout << endl;

    // if (kdtree.radiusSearch(point, tolerance, pointIdxRadiusSearch, pointRadiusSquaredDistance) > min_k)
    // {
    //   double gap = GetPlaneError(coefficients_2, point);

    //   gap_list.push_back(gap);
    //   gap_point->push_back(point);
    //   gap_point->push_back(plane_2->points[pointIdxRadiusSearch[0]]);

    // }

    // kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0
    // std::vector<int> pointIdxRadiusSearch;
    // std::vector<float> pointRadiusSquaredDistance;
  // }
}


void CalculateMeanStd(vector<float> data_list, float& mean, float& std)
{
  if (data_list.size() == 0)
  {
    cout << "input is empty!!!" << endl;
  }

  for (float data : data_list)
  {
    mean += data;
  }
  mean /= float(data_list.size());


  for (float data : data_list)
  {
    std += (mean - data) * (mean - data);
  }
  std /= float(data_list.size());
  std = sqrt(std);

}

void CalculateAllGap(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& plane_list,
  vector<pcl::ModelCoefficients::Ptr>& coefficients_list,
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& gap_point_list,
  float tolerance, vector<float>& mean_gap_list)
{
  for (int i = 0;i < plane_list.size() - 1;i++)
  {
    vector<float> gap_list;
    pcl::PointCloud<pcl::PointXYZ>::Ptr gap_point(new pcl::PointCloud<pcl::PointXYZ>);
    CalculateGap(plane_list[i], plane_list[i + 1], coefficients_list[i + 1],
      gap_point, tolerance, gap_list);
    gap_point_list.push_back(gap_point);

    float mean, std;
    CalculateMeanStd(gap_list, mean, std);
    mean_gap_list.push_back(mean);
  }
}

void CoutVector(vector<float> data_list)
{
  float mean = 0;
  for (float data : data_list)
  {
    cout << data << " ";
    mean += abs(data);
  }
  mean /= data_list.size();
  cout << mean << " ";
  cout << endl;
}

void PclViewerPC(string file_name, string type)
{
  int n1 = file_name.find_last_of('/') + 1;
  int n2 = file_name.find('.');
  int length = n2 - n1;
  string   output_file = file_name.substr(0, n1) + type + "/"
    + file_name.substr(n1, length) + "_*";
  string cmd = "pcl_viewer " + output_file;
  // cout << cmd;
  system(cmd.c_str());
}

void CalculatePlaneAngle(pcl::ModelCoefficients::Ptr coefficients_1,
  pcl::ModelCoefficients::Ptr coefficients_2, float& angle)
{
  Eigen::Vector3d v1(coefficients_1->values[0], coefficients_1->values[1], coefficients_1->values[2]);
  Eigen::Vector3d v2(coefficients_2->values[0], coefficients_2->values[1], coefficients_2->values[2]);
  float cos_value = v1.dot(v2) / (v1.norm() * v2.norm()); //角度cos值
  angle = acos(cos_value) * 180 / M_PI;
}

void CalculateAllPlaneAngle(vector<pcl::ModelCoefficients::Ptr> coefficients_list,
  vector<float>& angle_list)
{
  for (int i = 0;i < coefficients_list.size() - 1;i++)
  {
    float angle;
    CalculatePlaneAngle(coefficients_list[i], coefficients_list[i + 1], angle);
    angle_list.push_back(angle);
  }
}


int main(int argc, char** argv)
{
  if (argc < 6)
  {
    cout << "./m1_analyze m1_file.pcd min_z max_z plane_thick search_radius show_result" << endl;
    exit(0);
  }
  string file_name = argv[1];

  pcl::PointCloud<pcl::PointXYZ>::Ptr m1_pc;
  m1_pc = LoadPcd(file_name, false);

  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m1_pc_split;
  SplitM1PointCloud(m1_pc, m1_pc_split);

  // SaveSplitPointCloud(m1_pc_split, file_name, "split");
  // PclViewerPC(file_name, "split");

  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m1_pc_split_plane;
  vector<pcl::ModelCoefficients::Ptr> coefficients_list;
  FindAllPlane(m1_pc_split, atof(argv[2]), atof(argv[3]), atof(argv[4]),
    m1_pc_split_plane, coefficients_list);

  int n = file_name.find_last_of('/') + 1;
  cout << file_name.substr(n, 12) << " ";

  vector<float> angle_list;
  CalculateAllPlaneAngle(coefficients_list, angle_list);
  CoutVector(angle_list);

  // SaveSplitPointCloud(m1_pc_split_plane, file_name, "plane");
  // PclViewerPC(file_name, "plane");

  vector<float> mean_gap_list;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> gap_point_list;
  CalculateAllGap(m1_pc_split_plane, coefficients_list,
    gap_point_list, atof(argv[5]), mean_gap_list);

  // CoutVector(mean_gap_list);

  if (argc > 6)
  {
    if (argv[6] == "true")
    {
      SaveSplitPointCloud(gap_point_list, file_name, "gap");
      PclViewerPC(file_name, "gap");
    }
  }


}
