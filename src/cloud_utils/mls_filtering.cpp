# include <iostream>

# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl/kdtree/kdtree_flann.h>
# include <pcl/surface/mls.h>
# include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
    const string input_name = (string)argv[1];
    const string output_name = (string)argv[2];
    const float mls_radius = atof(argv[3]);

    PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>);
    PointCloud<PointNormal>::Ptr filtered (new PointCloud<PointNormal>);
    io::loadPLYFile(input_name.c_str(), *cloud);
    cout << "size: " << cloud->size() << endl;

    search::KdTree<pcl::PointNormal>::Ptr tree (new search::KdTree<pcl::PointNormal>);
    pcl::MovingLeastSquares<PointNormal, PointNormal> mls;

    mls.setComputeNormals(false);
    // mls.setPolynomialFit(false);
    mls.setPolynomialOrder(4);
    mls.setInputCloud(cloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mls_radius);
    
    mls.process (*filtered);

    printf("Filtering: %d / %d", filtered->size(), cloud->size());
    io::savePLYFileASCII(output_name.c_str(), *filtered);

    visualization::PCLVisualizer viewer_1 ("MLS Filtered");
    viewer_1.setBackgroundColor(100, 100, 200);
    visualization::PointCloudColorHandlerCustom<PointNormal> blue (filtered, 50, 50, 250);
    viewer_1.addPointCloud(filtered, blue, "filtered");

    visualization::PCLVisualizer viewer_2 ("Original");
    viewer_2.setBackgroundColor(100, 100, 200);
    visualization::PointCloudColorHandlerCustom<PointNormal> red (cloud, 250, 50, 50);
    viewer_2.addPointCloud(cloud, red, "original");

    while(!viewer_1.wasStopped() && !viewer_2.wasStopped())
    {
        viewer_1.spinOnce();
        viewer_2.spinOnce();
    }
    return 0;
}