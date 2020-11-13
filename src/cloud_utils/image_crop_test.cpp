# include <iostream>

# include <opencv2/core.hpp>
# include <opencv2/highgui.hpp>

# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>
# include <pcl/visualization/pcl_visualizer.h>

# include <k4a/k4a.hpp>
# include <k4a/k4a.h>

using namespace std;
using namespace cv;
using namespace pcl;

// double fx = 983.016;
// double fy = 982.984;
// double ppx = 1021.29;
// double ppy = 774.724;
double fx = 614.384;
double fy = 614.365;
double ppx = 638.121;
double ppy = 364.01;
void deprojection(Mat rgb, Mat depth, PointCloud<PointXYZRGB>::Ptr cloud)
{
    PointCloud<PointXYZRGB> tmp;
    int rows = rgb.rows;
    int cols = rgb.cols;
    if (rgb.rows != depth.rows || rgb.cols != depth.cols)
    {
        printf("RGB image and Depth image don't have same sizes.\n");
        return;
    }
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            PointXYZRGB point;
            float z = depth.at<float>(i, j);
            if (z > 0){
                cout << z << endl;
                // float x = j * z;
                // float y = i * z;
                float x = (j - ppx) * z / fx;
                float y = (i - ppy) * z / fy;
                // float x = fx*j + ppx*z;
                // float y = fy*i + ppy*z;
                point.x = x; // millimeters to meters;
                point.y = y;
                point.z = z;
                point.r = rgb.at<Vec3b>(i, j)[0];
                point.g = rgb.at<Vec3b>(i, j)[1];
                point.b = rgb.at<Vec3b>(i, j)[2];
                tmp.push_back(point);              
            }
            else{
                // printf("[%d, %d] unreasonable z: %f\n", i, j, z);
                continue;
            }
        }
    }
    *cloud = tmp;
}
int main(int argc, char** argv)
{
    Mat rgb = imread(argv[1]);
    Mat depth = imread(argv[2], IMREAD_ANYDEPTH);

    int left = 443;
    int top = 167;
    int right = 553;
    int bottom = 379;
    int width = right - left;
    int height = bottom - top;

    cv::Rect roi;
    roi.x = left;
    roi.y = top;
    roi.width = width;
    roi.height = height;

    Mat rgb_cropped = rgb(roi);
    Mat depth_cropped = depth(roi);
    cout << "Depth type: " << depth.type() << endl;

    PointCloud<PointXYZRGB>::Ptr cropped (new PointCloud<PointXYZRGB>);
    deprojection(rgb_cropped, depth_cropped, cropped);
    io::savePLYFileASCII("cropped.ply", *cropped);

    imshow("rgb_cropped", rgb_cropped);
    imshow("depth_cropped", depth_cropped);

    visualization::PCLVisualizer viewer ("Cloud viewer");
    viewer.setBackgroundColor(200, 200, 230);
    viewer.addPointCloud(cropped, "cloud");
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");


    // waitKey(0);
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    

    return 0;
}