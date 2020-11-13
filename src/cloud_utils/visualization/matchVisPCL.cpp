#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>

using namespace std;


int main(int argc, char** argv){
    string sceneFileName = argv[1];
    string transFileName = argv[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(sceneFileName.c_str(), *scene);
    pcl::io::loadPLYFile(transFileName.c_str(), *trans);
    // pcl::io::loadPLYFile(modelFileName, *model);

    int v1(0),v2(0);
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (scene, 255,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (trans, 0,0,125);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (model, 0,50,0);
    viewer.setBackgroundColor(220, 200, 200);
    viewer.addPointCloud(scene, red, "scene");
    viewer.addPointCloud(trans, blue, "trans");
    // viewer.addCoordinateSystem(2.5, "reference");
    // viewer.addPointCloud(model, green, "model");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene" );
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "trans" );
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model" );
    
    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }
    
    return 0;

}