#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

using namespace std;

int
main (int argc, char** argv)
{

    string input_name = (string)argv[1];
    string output_name = (string)argv[2];
  // All the objects needed
  pcl::PLYReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PLYWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);

  // Read in the cloud data
  reader.read (input_name, *cloud);
  std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;

//   // Build a passthrough filter to remove spurious NaNs
//   pass.setInputCloud (cloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (0, 1.5);
//   pass.filter (*cloud_filtered);
//   std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);
  cout << "normal computed" << endl;
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the sphere inliers and coefficients
  seg.segment (*inliers_sphere, *coefficients_sphere);
  std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;

  // Write the sphere inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_sphere);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_sphere);
  if (cloud_sphere->points.empty ()) 
    std::cerr << "Can't find the spherical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the spherical component: " << cloud_sphere->size () << " data points." << std::endl;
	  writer.write (output_name, *cloud_sphere, false);
  }

  // draw sphere center point and axis
  pcl::PointXYZ cyl_centroid;
  pcl::PointCloud<pcl::PointXYZ>::Ptr markers (new pcl::PointCloud<pcl::PointXYZ>);
  cyl_centroid.x = coefficients_sphere->values[0];
  cyl_centroid.y = coefficients_sphere->values[1];
  cyl_centroid.z = coefficients_sphere->values[2];
  markers->push_back(cyl_centroid);

  // visualization
  pcl::visualization::PCLVisualizer viewer ("cloud viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (cloud_sphere, 100,100,255);
  viewer.addPointCloud(cloud_sphere, blue, "cloud");
  viewer.addPointCloud(markers, "markers");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "markers");

  while(!viewer.wasStopped())
  {
    viewer.spinOnce();
  }  

  return (0);
}