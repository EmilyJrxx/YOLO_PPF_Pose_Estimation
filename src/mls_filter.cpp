# include <iostream>

# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl/kdtree/kdtree_flann.h>
# include <pcl/surface/mls.h>
# include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

typedef PointNormal PointT;

int main(int argc, char** argv)
{
    const string input_name = (string)argv[1];
    const string output_name = (string)argv[2];

    const float mls_radius = atof(argv[3]);

    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    PointCloud<PointT>::Ptr filtered (new PointCloud<PointT>);
    io::loadPLYFile(input_name, *cloud);
    cout << "size: " << cloud->size() << endl;

    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>);
    MovingLeastSquares<PointT, PointT> mls;

    mls.setComputeNormals(false);
    mls.setPolynomialOrder(4);
    mls.setInputCloud(cloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mls_radius);
    mls.setSqrGaussParam(mls_radius * mls_radius);
    mls.setUpsamplingMethod (MovingLeastSquares<PointT, PointT>::NONE);
    mls.setPointDensity (60000 * int (mls_radius));
    mls.setUpsamplingRadius (0.025);
    mls.setUpsamplingStepSize (0.015);
    mls.setDilationIterations (2);
    mls.setDilationVoxelSize (0.01f);

    mls.process(*filtered);

    printf("Filtering: %d / %d\n", filtered->size(), cloud->size());
    io::savePLYFileASCII(output_name, *filtered);

    // visualization
    visualization::PCLVisualizer viewer1 ("original");
    visualization::PCLVisualizer viewer2 ("filtered");
    viewer1.setBackgroundColor(255,255,255);
    viewer2.setBackgroundColor(255,255,255);
    visualization::PointCloudColorHandlerCustom<PointT> red (cloud, 255, 0, 0);
    visualization::PointCloudColorHandlerCustom<PointT> blue (filtered, 0, 0, 255);
    viewer1.addPointCloud(cloud, red, "cloud");
    viewer2.addPointCloud(filtered, blue, "filtered");
    viewer1.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer2.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered");
    while(!viewer1.wasStopped() && !viewer2.wasStopped())
    {
        viewer1.spinOnce();
        viewer2.spinOnce();
    }

    return 0;
}
