#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;
void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
  viewer.setBackgroundColor(1.0, 0.5, 1.0);
  pcl::PointXYZ o;
  o.x = 1.0;
  o.y = 0;
  o.z = 0;
  viewer.addSphere(o, 0.25, "sphere", 0);
}

void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
  static unsigned count = 0;
  std::stringstream ss;
  ss << "Once per viewer loop: " << count++;
  viewer.removeShape("text", 0);
  viewer.addText(ss.str(), 200, 300, "text", 0);
  //FIXME: possible race condition here:
  user_data++;
}

int
main(int argc, char* argv[])
{
  if (argc < 2)
  {
    cout << "No arguments, you must give an argument at least!" << endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(argv[1], *cloud);
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //showCloud������ͬ���ģ��ڴ˴��ȴ�ֱ����Ⱦ��ʾΪֹ
  viewer.showCloud(cloud);
  //��ע�ắ���ڿ��ӻ�ʱֻ����һ��
  viewer.runOnVisualizationThreadOnce(viewerOneOff);
  //��ע�ắ������Ⱦ���ʱÿ�ζ�����
  viewer.runOnVisualizationThread(viewerPsycho);
  while (!viewer.wasStopped())
  {
    //�ڴ˴�����������������
    user_data++;
  }
  return 0;
}
