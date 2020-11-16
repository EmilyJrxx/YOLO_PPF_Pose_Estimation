#include <iostream>
// #include <pcl/point_types.h>
// #include <pcl/impl/point_types.hpp>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/utility.hpp>
#include <vector>
#include <string>

using namespace std;
using namespace pcl;
// string origPCfilename = "./data/scenes/rs1.ply";
const Eigen::Vector4f subsampling_leaf_size (1.2f, 1.2f, 1.2f, 1.2f);

typedef PointXYZ PointT;

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

int main (int argc, char** argv)
{
     
    int if_scene = atoi(argv[4]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::io::loadPLYFile(argv[1], *cloud);
    // Get the geometry properties of the model
    Eigen::Vector4f cloud_centroid;

    // get rid of outliers
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (30);
    sor.setStddevMulThresh (1.2);
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    // Subsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled (new pcl::PointCloud<pcl::PointXYZ>);
    if (cloud_filtered->points.size() > 10000){
        cloud_subsampled = subsampleOnly(cloud_filtered);
    }

    // Normal Estimation
    ne.setInputCloud(cloud_subsampled);
    // create the normal estimation class, and pass the input dataset to it.
    ne.setNumberOfThreads(12); // mannually set the number of threads
    //create an empty kdtree representation, and pass it to the normal estimation object
    //its content will be filled inside the object, based on the given input dataset.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);
    //output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);        //use all neighbors in a sphere of radius 3cm
    ne.setKSearch(std::atoi(argv[3]));
    ne.setViewPoint(0.0, 0.0, 0.0);
    ne.compute(*normals);
    // flip the normals outwards
    for (size_t i = 0; i < normals->points.size(); i++)
    {
      PointXYZ point = cloud_subsampled->points[i];
      flipNormalTowardsViewpoint(point, 0.0, 0.0, 0.0, 
        normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    
    // pcl::copyPointCloud(*cloud, *cloud_normals);
    // pcl::copyPointCloud(*normals, *cloud_normals);
    cout << cloud_subsampled->size() << endl;
    cout << normals->size() << endl;
    pcl::concatenateFields(*cloud_subsampled, *normals, *cloud_normals);
    
    pcl::io::savePLYFileASCII(argv[2], *cloud_normals);
    //Save the normals with the points into a PLY file
        
    // pcl::io::savePLYFileASCII("./data/scenes/rs1_normals.ply", *normals);
    // Visualize normals 
    // int v1(0),v2(0);
    // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud, 255,0,0);
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> red (normals, 255,0,0);
    // viewer.setBackgroundColor(0.0, 0.0, 0.5);
    // viewer.addPointCloud(cloud, red, "points", v1);
    // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 100, 7, "cloud_normal");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_normal");
    // // viewer.addCoordinateSystem(200, "reference");
    // // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points");
    // // viewer.addCoordinateSystem(1.0);
    // // viewer.addPointCloud<pcl::PointXYZ>(cloud);

    // while(!viewer.wasStopped()){
    //     viewer.spinOnce();
    // } 
    return 0;
}