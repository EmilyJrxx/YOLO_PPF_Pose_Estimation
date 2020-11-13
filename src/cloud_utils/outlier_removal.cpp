#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core.hpp>

using namespace std;
using namespace pcl;

PointCloud<PointNormal>::Ptr
subsampleOnly (PointCloud<PointNormal>::Ptr cloud, float leaf_size)
{
  Eigen::Vector4f subsampling_leaf_size (leaf_size, leaf_size, leaf_size, leaf_size);
  PointCloud<PointNormal>::Ptr cloud_subsampled (new PointCloud<PointNormal> ());
  VoxelGrid<PointNormal> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  PCL_INFO ("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size (), cloud_subsampled->points.size ());

  return cloud_subsampled;
}

void help(const string& message)
{
    cout << "Program init error: " << message << endl;
    cout << "\nUsage : [program_name] [input_cloud] [save_cloud] [downsample_size] [so_meanK] [so_thresh]"<< endl;
    cout << "[input cloud]:          input cloud (.ply)" << endl
         << "[output cloud]:         output cloud(.ply)" << endl
         << "[downsample size]:      leaf size of voxelgrid for subsampling" << endl
         << "[so_meanK]:             meanK of statistical outlier removal" << endl
         << "[so_thresh]:            distance threshold for statistical outlier removal" << endl;
    cout << "\nPlease start again with new parameters"<< endl;
}
int main (int argc, char** argv)
{
  if (argc < 6){
      help("Not Enough Arguments");
      exit(1);
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

  // Fill in the cloud data
  pcl::PLYReader reader;
  
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointNormal> (argv[1], *cloud);
  
  // Voxel DownSampling
  cloud = subsampleOnly(cloud, atof(argv[3]));
  
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (atoi(argv[4]));
  sor.setStddevMulThresh (atof(argv[5]));
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PLYWriter writer;
  writer.write<pcl::PointNormal> (argv[2], *cloud_filtered, false);

  // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>  green (cloud, 0 , 160, 0);
  // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>  red (cloud, 255,50,50);
  // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (model, 0,50,0);
  // viewer.setBackgroundColor(0, 0, 0);
  // // viewer.addPointCloud(cloud, "original");
  // viewer.addPointCloud(cloud_filtered, "filtered");
  // // viewer.addPointCloud(model, green, "model");
  // // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original" );
  // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered" );
  // // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model" );
  
  // while(!viewer.wasStopped()){
  //     viewer.spinOnce();
  // }
//   sor.setNegative (true);
//   sor.filter (*cloud_filtered);
//   writer.write<pcl::PointXYZ> (argv[3], *cloud_filtered, false);

  return (0);
}