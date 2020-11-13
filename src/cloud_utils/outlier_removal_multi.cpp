#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>

#include <opencv2/core.hpp>
#include <cmath>

using namespace std;
using namespace pcl;

const string input_dir = "../clouds/cup_top";
const string output_dir = "../clouds/cup_top/outlier_removal/";

PointCloud<PointXYZRGB>::Ptr
subsampleOnly (PointCloud<PointXYZRGB>::Ptr cloud, float leaf_size)
{
  Eigen::Vector4f subsampling_leaf_size (leaf_size, leaf_size, leaf_size, leaf_size);
  PointCloud<PointXYZRGB>::Ptr cloud_subsampled (new PointCloud<PointXYZRGB> ());
  VoxelGrid<PointXYZRGB> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  // PCL_INFO ("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size (), cloud_subsampled->points.size ());

  return cloud_subsampled;
}
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  pcl::PLYReader reader;
  
  vector<cv::String> filenames;
  cv::glob(input_dir, filenames);
  for (int i = 0; i < filenames.size(); i++)
  {
    cout << "reading: " << filenames[i] << endl;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZRGB> (filenames[i].c_str(), *cloud);
    
    // Voxel DownSampling
    cloud = subsampleOnly(cloud, atof(argv[1]));
    
    // std::cerr << "Cloud before filtering: " << std::endl;
    // std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (atoi(argv[2]));
    sor.setStddevMulThresh (atof(argv[3]));
    sor.filter (*cloud_filtered);

    // std::cerr << "Cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;

    // Normal Estimation
  
    int start = filenames[i].find_first_not_of(input_dir);
    int end = filenames[i].find(".ply");
    string num = filenames[i].substr(start, end - start);
    if (atoi(num.c_str()) < 10)
    {
      num = "00" + num;
    }
    else if (atoi(num.c_str()) < 100)
    {
      num = "0" + num;
    }
    string ply_filename = (string)(output_dir + num + ".ply");
    pcl::PLYWriter writer;
    cout << "writing: " << ply_filename << endl;
    writer.write<pcl::PointXYZRGB> (ply_filename, *cloud_filtered, false);  
  }

//   sor.setNegative (true);
//   sor.filter (*cloud_filtered);
//   writer.write<pcl::PointXYZ> (argv[3], *cloud_filtered, false);

  return (0);
}