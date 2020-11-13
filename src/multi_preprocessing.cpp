#include <iostream>
#include <pcl/point_types.h>
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

const Eigen::Vector4f subsampling_leaf_size (3.0f, 3.0f, 3.0f, 3.0f);

PointCloud<PointXYZ>::Ptr
subsampleOnly (const PointCloud<PointXYZ>::Ptr& cloud)
{
  PointCloud<PointXYZ>::Ptr cloud_subsampled (new PointCloud<PointXYZ> ());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  PCL_INFO ("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size (), cloud_subsampled->points.size ());

  return cloud_subsampled;
}

int main(int argc, char **argv)
{
    string input_dir = argv[1];
    string output_dir = argv[2];
    int knn_k = atoi(argv[3]);

    vector<string> fn;
    cv::glob(argv[1], fn, false);
    for (int k=0; k < fn.size(); k++)
    {
        cout << fn[k] << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        pcl::io::loadPLYFile(fn[k], *cloud);

        // get rid of outliers
        std::cerr << "Cloud before filtering: " << std::endl;
        std::cerr << cloud->size() << std::endl;

        sor.setInputCloud (cloud);
        sor.setMeanK (10);
        sor.setStddevMulThresh (1.2);
        sor.filter (*cloud_filtered);

        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << cloud_filtered->size() << std::endl;

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subsampled (new pcl::PointCloud<pcl::PointXYZ>);
        // subsampling

        cout << "Subsampling" << endl;
        cloud_subsampled = subsampleOnly(cloud_filtered);

        // surface normal estimation
        ne.setInputCloud(cloud_subsampled);
        ne.setNumberOfThreads(12); // mannually set the number of threads   
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(knn_k);
        ne.compute(*normals);    

        pcl::concatenateFields(*cloud_subsampled, *normals, *cloud_normals);

        // saving
        string writefile = output_dir + (string)"rs" + to_string(k+1) + (string)"st_or_multi1.5_knnk8.ply";
        cout << "Normalized pc writing to : " << writefile << endl; 
        pcl::io::savePLYFileASCII(writefile.c_str(), *cloud_normals);
    }

    return 0;
}
