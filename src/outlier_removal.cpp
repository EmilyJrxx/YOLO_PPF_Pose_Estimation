#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace pcl;

const Eigen::Vector4f subsampling_leaf_size (3.0f, 3.0f, 3.0f, 3.0f);

typedef PointNormal PointT;

PointCloud<PointT>::Ptr
subsampleOnly (const PointCloud<PointT>::Ptr& cloud)
{
  PointCloud<PointT>::Ptr cloud_subsampled (new PointCloud<PointT> ());
  VoxelGrid<PointT> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  PCL_INFO ("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size (), cloud_subsampled->points.size ());

  return cloud_subsampled;
}
int main(int argc, char** argv)
{
    string input = argv[1];
    string output = argv[2];

    pcl::PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    pcl::io::loadPLYFile(input, *cloud);

    // get rid of outliers
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << cloud->size() << std::endl;

    sor.setInputCloud (cloud);
    sor.setMeanK (10);
    sor.setStddevMulThresh (1.2);
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << cloud_filtered->size() << std::endl;

    pcl::PointCloud<PointT>::Ptr cloud_subsampled (new pcl::PointCloud<PointT>);
    // subsampling

    cout << "Subsampling" << endl;
    cloud_subsampled = subsampleOnly(cloud_filtered);

    // saving
    string writefile = output;
    cout << "Normalized pc writing to : " << writefile << endl; 
    pcl::io::savePLYFileASCII(writefile.c_str(), *cloud_subsampled);

    // visualization
    visualization::PCLVisualizer viewer ("Cloud viewer");
    viewer.setBackgroundColor(0,0,0);
    visualization::PointCloudColorHandlerCustom<PointT> red (cloud, 200, 0, 0);
    visualization::PointCloudColorHandlerCustom<PointT> green (cloud_subsampled, 0, 255, 0);
    viewer.addPointCloud(cloud, red, "cloud");
    viewer.addPointCloud(cloud_subsampled, green, "cloud_sub");
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_sub");
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}