#define PCL_NO_PRECOMPILE
# include <iostream>
# include <pcl/point_types.h>
# include <pcl/point_cloud.h>
# include <pcl/io/ply_io.h>
# include <pcl/kdtree/kdtree_flann.h>

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

int main(int argc, char** argv)
{
    string input_name = argv[1];
    string contact_parts_file = argv[2];
    string output_name = argv[3];
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    PointCloud<PointT>::Ptr contact_parts (new PointCloud<PointT>);
    io::loadPLYFile(input_name, *cloud);
    io::loadPLYFile(contact_parts_file, *contact_parts);
    
    // compare Contact_parts & Original Cloud
    KdTreeFLANN<PointT> tree;
    tree.setInputCloud(cloud);
    int k = 3;
    vector<int> contact_idx;
    for (size_t i = 0; i < contact_parts->size(); i++)
    {
        PointT seed = contact_parts->points[i];
        vector<int> idxSearch(k);
        vector<float> pointDistance(k);
        if (tree.nearestKSearch(seed, k, idxSearch, pointDistance) > 0)
        {
            contact_idx.push_back(idxSearch[0]);
        }
    }

    // label contact points
    PointCloud<PointNormalContactLabel>::Ptr cloud_labeled (new PointCloud<PointNormalContactLabel>);
    for (size_t i = 0; i < cloud->size(); i++)
    {
        cloud_labeled->points[i].x = cloud->points[i].x;
        cloud_labeled->points[i].y = cloud->points[i].y;
        cloud_labeled->points[i].z = cloud->points[i].z;
        cloud_labeled->points[i].normal_x = cloud->points[i].normal_x;
        cloud_labeled->points[i].normal_y = cloud->points[i].normal_y; 
        cloud_labeled->points[i].normal_z = cloud->points[i].normal_z;
        cloud_labeled->points[i].curvature = cloud->points[i].curvature;
        cloud_labeled->points[i].contact = 0;
    }
    for (int i = 0; i < contact_idx.size(); i++)
    {
        cloud_labeled->points[contact_idx[i]].contact = 1;
    }
    
    // saving
    io::savePLYFileASCII(output_name, *cloud_labeled);
    
    return 0;
}