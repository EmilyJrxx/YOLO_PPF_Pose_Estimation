# include <k4a/k4a.h>
# include <k4a/k4a.hpp>

# include <iostream>
# include <vector>

# include <opencv4/opencv2/core/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>
# include <opencv2/xfeatures2d.hpp>

# include <pcl/io/ply_io.h>
# include <pcl/point_cloud.h>
# include <pcl/point_types.h>
# include <pcl/visualization/pcl_visualizer.h>
# include <pcl/filters/passthrough.h>

# include "Pixel.h"
# include "DepthPixelColorizer.h"
# include "StaticImageProperties.h"
# include "k4a_grabber.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace boost;

double xmin, ymin, zmin, xmax, ymax, zmax;

int main(int argc, char** argv){
    if (argc < 3)
    {
        cerr << "Not enough Args: k4a_cloud_test [output_dir] [saving num]." << endl;
        exit(1);
    }
    const uint32_t deviceNum = k4a::device::get_installed_count();
    if (deviceNum == 0){
        cerr << "No Azure Kinect Device Detected. " << endl;
        exit(1);
    }
    const string output_dir = (string)argv[1];

    // Parameters Specification
    xmin = -1000; ymin = -1000; zmin = -1000;
    xmax = 1000; ymax = 1000; zmax = 1000;

    // PCL visualizer
    boost::shared_ptr<visualization::PCLVisualizer> viewer(
        new visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setCameraPosition(0.0, 0.0, -2500.0, 1.0, -.0, 1.0);

    // PCL pointlcloud 
    PointCloud<PointXYZRGBA>::ConstPtr cloud;

    // Callback Function
    boost::mutex mutex;
    boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr&)> func = 
    [&cloud, &mutex](const PointCloud<PointXYZRGBA>::ConstPtr& ptr){
        // cout << "Callback" << endl;
        boost::mutex::scoped_lock lock(mutex);
        cloud = ptr->makeShared();
    };

    // KinectAzureGrabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::KinectAzureDKGrabber>(
        0, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);
    
    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback(func);
    
    // Start Grabber
    grabber->start();

    int frame_saved = atoi(argv[2]);
    while(!viewer->wasStopped())
    {
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock(mutex);
        if (lock.owns_lock() && cloud){
            // Update Point Cloud
            // cout << "update" << endl;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            copyPointCloud(*cloud, *input_cloud);
            // pcl::PassThrough<PointXYZRGBA> pass;
            // pass.setInputCloud (input_cloud);
            // pass.setFilterFieldName ("x");
            // pass.setFilterLimits (xmin, xmax);
            // pass.filter (*input_cloud);
            
            // pass.setInputCloud (input_cloud);
            // pass.setFilterFieldName ("y");
            // pass.setFilterLimits (ymin, ymax);
            // pass.filter (*input_cloud);

            // pass.setInputCloud (input_cloud);
            // pass.setFilterFieldName ("z");
            // pass.setFilterLimits (zmin, zmax);
            // pass.filter (*input_cloud);

            // cout << "Cloud FPS: " << grabber->getFramesPerSecond() << endl;
            if (!viewer->updatePointCloud(input_cloud, "cloud")){
                viewer->addPointCloud(input_cloud, "cloud");
                string output_name = output_dir + to_string(frame_saved) + ".ply";
                pcl::io::savePLYFileASCII(output_name, *input_cloud);
                cout << "saving: " << output_name << endl;
            }
        }

    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if (connection.connected())
    {
        connection.disconnect();
    }
    return 0;
}