#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <glob.h>
#include <vector>
#include <string>
using namespace std;

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

void getFiles(string path, vector<string>& filenames)
{
  DIR* pDir;
  struct dirent* ptr;
  if (!(pDir = opendir(path.c_str()))) {
    cout << "Folder doesn't Exist!" << endl;
    return;
  }
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      filenames.push_back(ptr->d_name);
    }
  }
  closedir(pDir);
}

int user_data;
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
  //FIXME: possible race condition here:
  user_data++;
}

void mergerPointcloud(const string& filePath, const  vector<string>& files, string& header, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_final)
{
  int header_length = header.length();
  string output = header + "_merge.pcd";
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < files.size(); i++) {
    // cout << files[i] << endl;
    string header_i = files[i].substr(0, header_length);
    // cout << header_i << "\\" << header << "\\" << header_i.compare(header) << endl;
    if (header_i.compare(header) == 0 && files[i].compare(output) != 0)
    {
      cout << files[i] << endl;
      pcl::io::loadPCDFile(filePath + files[i], *cloud_tmp);
      *cloud_final += *cloud_tmp;
    }
  }
}

void showPointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_final)
{
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(cloud_final);
  viewer.runOnVisualizationThreadOnce(viewerOneOff);
  viewer.runOnVisualizationThread(viewerPsycho);
  while (!viewer.wasStopped())
  {
    user_data++;
  }
}

int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    cout << "No enough arguments, you must give an argument at least!" << endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZI>);

  vector <string> files;

  string filePath = argv[1];
  string header = argv[2];
  string output = header + "_merge.pcd";

  getFiles(filePath, files);

  mergerPointcloud(filePath, files, header, cloud_final);

  pcl::io::savePCDFileASCII(output, *cloud_final);

  showPointcloud(cloud_final);

  return 0;
}
