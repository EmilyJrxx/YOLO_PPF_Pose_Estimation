# include <iostream>

# include <opencv4/opencv2/core/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>

# include <pcl/io/ply_io.h>
# include <pcl/point_types.h>

using namespace pcl;
using namespace cv;
using namespace std;

namespace ppf_utils
{
    class Camera
    {
        private:
            double fx;
            double fy;
            double ppx;
            double ppy;
        public:
            Camera(double fx_input=1.0, double fy_input=1.0, double ppx_input=0.0, double ppy_input=0.0){
                fx = fx_input;
                fy = fy_input;
                ppx = ppx_input;
                ppy = ppy_input;
            }
            ~Camera(){}
            PointXYZ back_projection(Mat depth, int u, int v);
            PointXYZ back_projection_bbox(float depth, int u, int v);
    };
    PointXYZ Camera::back_projection(Mat depth, int u, int v){
        // std::cout << "v: " << v << std::endl
                //   << "u: " << u << std::endl;   // debug   
        // if (u < 0) u = 0;
        // if (v < 0) v = 0; 
        // cout << "Mat size: " << depth.size() << endl;
        float z = depth.at<float>(v, u);
        // z = 3000;
        cout << "depth shape: " << depth.size() << " " << depth.dims << endl; 
        cout << "2D Point: " << u << " " << v << " " << z << endl;
        PointXYZ point;
        point.x = (float)(u - ppx) * z / fx; // mm to m
        point.y = (float)(v - ppy) * z / fy; // mm to m
        point.z = z; // mm to m
        cout << "3D Point: " << point.x << " " << point.y << " " << point.z << endl;
        return point;
    }
    PointXYZ Camera::back_projection_bbox(float depth, int u, int v)
    {
        float z = depth;
        // z = 3000;
        cout << "2D Point: " << u << " " << v << " " << z << endl;
        PointXYZ point;
        point.x = (float)(u - ppx) * z / fx; // mm to m
        point.y = (float)(v - ppy) * z / fy; // mm to m
        point.z = z; // mm to m
        cout << "3D Point: " << point.x << " " << point.y << " " << point.z << endl;
        return point;
    }
}