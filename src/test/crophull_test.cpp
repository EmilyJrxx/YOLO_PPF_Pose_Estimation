#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_hull.h>
// #include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
    string cloud_file = argv[1];
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    io::loadPLYFile(cloud_file, *cloud);

    PointXYZ left_top, left_bot, right_top, right_bot;
    left_top.x = -0.3885; left_top.y = -0.5662; left_top.z = 1.4197;
    left_bot.x = -0.3885; left_bot.y = 0.0485; left_bot.z = 1.4197;
    right_top.x = -0.0396; right_top.y = -0.5662; right_top.z = 1.4197;
    right_bot.x = -0.0396; right_bot.y = 0.0485; right_bot.z = 1.4197;

    left_top.z  += 0.15;
    left_bot.z  += 0.15;
    right_top.z += 0.15;
    right_bot.z += 0.15;

    PointCloud<PointXYZ>::Ptr boundingbox_3d (new PointCloud<PointXYZ>);
    boundingbox_3d -> push_back(left_top);
    boundingbox_3d -> push_back(left_bot);
    boundingbox_3d -> push_back(right_top);
    boundingbox_3d -> push_back(right_bot);
    boundingbox_3d -> push_back(PointXYZ(0.0, 0.0, 0.0));

    cout << "creating hull" << endl; //debug
    ConvexHull<PointXYZ> hull;
    hull.setInputCloud(boundingbox_3d);
    hull.setDimension(3);
    vector<Vertices> polygons;
    PointCloud<PointXYZ>::Ptr surface_hull (new PointCloud<PointXYZ>);
    cout << "reconstructing hull" << endl; //debug
    hull.reconstruct(*surface_hull, polygons);
    cout << "creating crophull" << endl; //debug
    cout << "surface hull check: " << endl;
    cout << "size: " << surface_hull->size() << endl;
    io::savePLYFileASCII("surface_hull.ply", *surface_hull);
    cout << "polygons check:" << endl;
    cout << "size: " << polygons.size() << endl;
    
    
    // PointCloud<PointXYZ>::Ptr cropped (new PointCloud<PointXYZ>);
    // CropHull<PointXYZ> bb_filter; 
    // // bb_filter.setDim(3);
    // bb_filter.setInputCloud(cloud);
    // bb_filter.setHullIndices(polygons);
    // bb_filter.setHullCloud(surface_hull);
    // cout << "cropping cloud" << endl;
    // bb_filter.filter(*cropped);

    // printf("Cropping : %d / %d \n", cloud->size(), cropped->size());
    return 0;
}
