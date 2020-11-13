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
const Eigen::Vector4f subsampling_leaf_size (2.5f, 2.5f, 2.5f, 2.5f);

PointCloud<PointNormal>::Ptr
subsampleOnly (const PointCloud<PointNormal>::Ptr& cloud)
{
  PointCloud<PointNormal>::Ptr cloud_subsampled (new PointCloud<PointNormal> ());
  VoxelGrid<PointNormal> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  PCL_INFO ("Cloud dimensions before / after subsampling: %u / %u\n", cloud->points.size (), cloud_subsampled->points.size ());

  return cloud_subsampled;
}

int main (int argc, char** argv)
{
     
    int if_scene = atoi(argv[4]);

    vector<string> fn;
    cv::glob(argv[1], fn, false);
    for (int k = 0; k < fn.size(); k++){
        cout << fn[k] << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        pcl::io::loadPLYFile(fn[k], *cloud);
        // Get the geometry properties of the model
        Eigen::Vector4f cloud_centroid;
        if (!if_scene){
            compute3DCentroid(*cloud, cloud_centroid);
            ne.setInputCloud(cloud);
        }
        else{
        // get rid of outliers
            std::cerr << "Cloud before filtering: " << std::endl;
            std::cerr << *cloud << std::endl;
            // Create the filtering object
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud (cloud);
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.2);
            sor.filter (*cloud_filtered);
            std::cerr << "Cloud after filtering: " << std::endl;
            std::cerr << *cloud_filtered << std::endl;

            ne.setInputCloud(cloud_filtered);
        }
        PointXYZ xyz_centroid(cloud_centroid(0)*cloud_centroid(3), cloud_centroid(1)*cloud_centroid(3), cloud_centroid(2)*cloud_centroid(3));
        // create the normal estimation class, and pass the input dataset to it.      
        ne.setNumberOfThreads(12); // mannually set the number of threads     
        //create an empty kdtree representation, and pass it to the normal estimation object
        //its content will be filled inside the object, based on the given input dataset.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod(tree);
        //output datasets
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(std::atoi(argv[3]));
        ne.compute(*normals);
        // flip the normals outwards
        if (!if_scene){
            cout << " filp viewpoint " << endl; 
            for (int i = 0; i < normals->points.size(); i++){
                PointXYZ point = cloud->points[i];
                // Normal normal = normals->points[i];
                // Eigen::Vector4f normal;
                flipNormalTowardsViewpoint(point, 2*point.x-xyz_centroid.x, 2*point.y-xyz_centroid.y, 2*point.z-xyz_centroid.z,
                normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
            }
        }

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_subsampled (new pcl::PointCloud<pcl::PointNormal>);
        // pcl::copyPointCloud(*cloud, *cloud_normals);
        // pcl::copyPointCloud(*normals, *cloud_normals);
        if(!if_scene){
            pcl::concatenateFields(*cloud, *normals, *cloud_normals);
        }
        else{
            pcl::concatenateFields(*cloud_filtered, *normals, *cloud_normals);
        }
        // Subsample the cloud_with_normals
        string resultdir = (string)argv[2];
        string writefile = resultdir + (string)"rs" + to_string(k+1) + (string)"st_or_multi1.5_knnk8.ply";
        cout << "Normalized pc writing to : " << writefile << endl; 
        if (cloud_normals->points.size() > 20000 && if_scene == 1){
            cout << "Subsampling" << endl;
            cloud_normals_subsampled = subsampleOnly(cloud_normals);
            pcl::io::savePLYFileASCII(writefile.c_str(), *cloud_normals_subsampled);
        }
        else{
            pcl::io::savePLYFileASCII(writefile.c_str(), *cloud_normals);
        }
    }
    

    return 0;
}