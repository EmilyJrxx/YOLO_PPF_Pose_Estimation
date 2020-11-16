#define PCL_NO_PRECOMPILE
# include <iostream>
# include <pcl/point_types.h>
# include <pcl/point_cloud.h>
# include <pcl/io/ply_io.h>
# include <pcl/kdtree/kdtree_flann.h>
# include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace pcl;

struct PointNormalContactLabel{
    PCL_ADD_POINT4D;
    PCL_ADD_NORMAL4D;
    float curvature;
    int contact;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (
    PointNormalContactLabel,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
    (int, contact, contact)
)

typedef PointNormal PointT;
typedef PointNormalContactLabel PointT_2;

PointXYZ drawArrow(PointXYZ& start, Eigen::Vector3f& direc, float length = 1.0)
{
    PointXYZ end;
    end.x = start.x + length * direc[0];
    end.y = start.y + length * direc[1];
    end.z = start.z + length * direc[2];
    
    return end;
}
int main(int argc, char** argv)
{
    // arguments parsing
    string input_file = argv[1];
    string output_file = argv[2];
    // load model file
    PointCloud<PointNormalContactLabel>::Ptr cloud_labeled (new PointCloud<PointNormalContactLabel>);
    io::loadPLYFile(input_file, *cloud_labeled);
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    copyPointCloud(*cloud_labeled, *cloud);
    // cylinder segmentation - centroid, z axis
    PassThrough<PointT> pass;
    NormalEstimation<PointT, pcl::Normal> ne;
    SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    ExtractIndices<PointT> extract;
    ExtractIndices<pcl::Normal> extract_normals;
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>);
    
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);
    PointCloud<PointT>::Ptr cylinder_cloud (new PointCloud<PointT>);
    ModelCoefficients::Ptr coefficients_cylinder (new ModelCoefficients);
    PointIndices::Ptr inliers_cylinder (new PointIndices);
    
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (30);
    ne.compute (*cloud_normals);
    
    /// remove planar parts
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);
    ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
    PointIndices::Ptr inliers_plane (new PointIndices);
    seg.segment(*inliers_plane, *coefficients_plane);

    PointCloud<Normal>::Ptr cloud_normals_2 (new PointCloud<Normal>);
    PointCloud<PointT>::Ptr cloud_2 (new PointCloud<PointT>);
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_plane);
    extract.setNegative (true);
    extract.filter(*cloud_2);
    extract_normals.setNegative(true);
    extract_normals.setIndices(inliers_plane);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.filter(*cloud_normals_2);  

    // segment cylindrical points
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 50);
    seg.setInputCloud (cloud_2);
    seg.setInputNormals (cloud_normals_2);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cylinder_cloud);
    PointXYZ cylinder_centroid;
    if (cylinder_cloud->empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cylinder_cloud->size () << " data points." << std::endl;
        std::cerr << *coefficients_cylinder << endl;
        computeCentroid(*cylinder_cloud, cylinder_centroid);
    }
    Eigen::Vector3f z_axis (coefficients_cylinder->values[3], 
                            coefficients_cylinder->values[4],
                            coefficients_cylinder->values[5]);
    
    
    // contact surface extraction : with OMP?
    ModelCoefficients::Ptr coefficients_contact (new ModelCoefficients);
    PointCloud<PointT_2>::Ptr cloud_contact (new PointCloud<PointT_2>);
    PointIndices::Ptr idx_contact (new PointIndices);
    for (size_t i = 0; i < cloud_labeled->size(); i++)
    {
        if (cloud_labeled->points[i].contact == 1)
        {
            idx_contact->indices.push_back(i);
        }
    }
    ExtractIndices<PointT_2> extract_contact;
    extract_contact.setInputCloud(cloud_labeled);
    extract_contact.setIndices(idx_contact);
    extract_contact.setNegative(false);
    extract_contact.filter(*cloud_contact);
    // contact surface normal statistics - x & y axis
    int seed_idx = (rand() % cloud_contact->size());
    PointNormalContactLabel contact_point = cloud_contact->points[seed_idx];
    Eigen::Vector3f x_axis (contact_point.normal_x,
                            contact_point.normal_y,
                            contact_point.normal_z);
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    
    // visualization
    visualization::PCLVisualizer viewer ("cloud_viewer");
    visualization::PointCloudColorHandlerCustom<PointT> blue (cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, blue, "cloud");
    float length = 10.0;
    PointXYZ z_end = drawArrow(cylinder_centroid, z_axis, length);
    PointXYZ x_end = drawArrow(cylinder_centroid, x_axis, length);
    PointXYZ y_end = drawArrow(cylinder_centroid, y_axis, length);
    viewer.addArrow(cylinder_centroid, x_end, 255, 0, 0);
    viewer.addArrow(cylinder_centroid, y_end, 0, 255, 0);
    viewer.addArrow(cylinder_centroid, z_end, 0, 0, 255);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cloud");

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;

}