# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl/features/normal_3d_omp.h>
# include <pcl/kdtree/kdtree_flann.h>
# include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

typedef PointNormal PointT;

int main(int argc, char** argv)
{
    string input_name = argv[1];
    string output_name = argv[2];
    float relative_threshold = atof(argv[3]);
    bool recompute_normal = false;
    if (argc > 4){
        recompute_normal = atoi(argv[4]);
    }

    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    io::loadPLYFile(input_name, *cloud);
    
    PointCloud<PointXYZ>::Ptr cloud_xyz (new PointCloud<PointXYZ>);
    copyPointCloud(*cloud, *cloud_xyz);

    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    NormalEstimationOMP<PointXYZ, Normal> ne;
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
    ne.setInputCloud(cloud_xyz);
    ne.setNumberOfThreads(12);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.compute(*normals);

    vector<float> curvs;
    for (size_t i = 0; i < normals->size(); i++)
    {
        curvs.push_back(normals->points[i].curvature);
    }
    auto max_curv_idx = max_element(curvs.begin(), curvs.end());
    auto min_curv_idx = min_element(curvs.begin(), curvs.end());
    float max_curv = *max_curv_idx;
    float min_curv = *min_curv_idx;
    float curv_threshold = min_curv + (max_curv - min_curv) * relative_threshold;
    PointCloud<PointT>::Ptr edges (new PointCloud<PointT>);
    for (size_t i = 0; i < curvs.size(); i++)
    {
        if (curvs[i] >= curv_threshold)
        { 
            if (recompute_normal == false){
                edges->points.push_back(cloud->points[i]);
            }
        }
    }
    io::savePLYFileASCII(output_name, *edges);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red (cloud, 255,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue (edges, 0,0,255);
    viewer.addPointCloud(cloud, red, "points");
    viewer.addPointCloud(edges, blue, "edges");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "edges");
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}