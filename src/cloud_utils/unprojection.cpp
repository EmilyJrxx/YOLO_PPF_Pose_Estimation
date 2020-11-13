# include "Camera.h"

# include <iostream>
# include <fstream>
# include <string>
// # include <ros/ros.h>

# include <pcl/features/normal_3d_omp.h>
# include <pcl/kdtree/kdtree_flann.h>
# include <pcl/filters/voxel_grid.h>
# include <pcl/filters/statistical_outlier_removal.h>
# include <pcl/filters/crop_hull.h>
# include <pcl/surface/convex_hull.h>

# include <opencv2/core.hpp>
# include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace pcl;
using namespace cv;

double fx = 614.384;
double fy = 614.365;
double ppx = 638.121;
double ppy = 364.01;
int main(int argc, char** argv)
{
    string rgb_file = argv[1];
    string depth_file = argv[2];

    Mat rgb = imread(rgb_file);
    Mat depth = imread(depth_file, IMREAD_ANYDEPTH);

    int rows = rgb.rows;
    int cols = rgb.cols;

    PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            PointXYZRGB point;
            
            float z = depth.at<float>(i, j);
            if (z > 0){
                float x = (j - ppx) * z / fx;
                float y = (i - ppy) * z / fy;
                point.z = z;
                point.x = x;
                point.y = y;
                point.r = rgb.at<Vec3b>(i, j)[0];
                point.g = rgb.at<Vec3b>(i, j)[1];
                point.b = rgb.at<Vec3b>(i, j)[2];
                cloud->push_back(point);                
            }
            else continue;
        }
    }
    io::savePLYFileASCII(argv[3], *cloud);

    return 0;
}