
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
 
int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read("..\\source\\pig.pcd"/*argv[1]*/,*cloud);
 
	//定义2D平面点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	boundingbox_ptr->push_back(pcl::PointXYZ(0.1, 0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(0.1, -0.1,0 ));
	boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, 0.1,0 ));
	boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, -0.1,0 ));
	boundingbox_ptr->push_back(pcl::PointXYZ(0.15, 0.1,0 ));
 
	pcl::ConvexHull<pcl::PointXYZ> hull;                  //创建凸包对象
	hull.setInputCloud(boundingbox_ptr);                  //设置输入点云
	hull.setDimension(2);                                 //设置凸包维度
	std::vector<pcl::Vertices> polygons;                  //设置向量，用于保存凸包定点
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);//该点运用于描述凸包形状
	hull.reconstruct(*surface_hull, polygons);            //计算2D凸包结果
 
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropHull<pcl::PointXYZ> bb_filter;               //创建crophull对象
	bb_filter.setDim(2);                                  //设置维度：该维度需要与凸包维度一致
	bb_filter.setInputCloud(cloud);                       //设置需要滤波的点云
	bb_filter.setHullIndices(polygons);                   //输入封闭多边形的顶点
	bb_filter.setHullCloud(surface_hull);                 //输入封闭多边形的形状
	bb_filter.filter(*objects);                           //执行CropHull滤波，存出结果在objects
	std::cout << objects->size() << std::endl;            //
 
	//visualize
	boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v (new pcl::visualization::PCLVisualizer ("crophull display"));
	for_visualizer_v->setBackgroundColor(255,255,255);
 
	int v1(0);//显示原始点云
	for_visualizer_v->createViewPort (0.0, 0.0, 0.33, 1, v1);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v1);
	for_visualizer_v->addPointCloud (cloud,"cloud",v1);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"cloud");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud");
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull,0,.069*255,0.2*255,"backview_hull_polyline1",v1);
 
	int v2(0);//显示封闭2D多边形凸包
	for_visualizer_v->createViewPort (0.33, 0.0, 0.66, 1, v2);	
	for_visualizer_v->setBackgroundColor (255, 255, 255, v2);
	for_visualizer_v->addPointCloud (surface_hull,"surface_hull",v2);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"surface_hull");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"surface_hull");
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull,0,.069*255,0.2*255,"backview_hull_polyline",v2);
 
	int v3(0);//显示滤波结果
	for_visualizer_v->createViewPort (0.66, 0.0, 1, 1, v3);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v3);
	for_visualizer_v->addPointCloud (objects,"objects",v3);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"objects");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"objects");
 
	while (!for_visualizer_v->wasStopped())
	{
		for_visualizer_v->spinOnce(1000);
	}
	system("pause");
}