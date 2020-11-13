# include <k4a/k4a.h>
# include <k4a/k4a.hpp>

# include <iostream>
# include <vector>
# include <string>

# include <opencv4/opencv2/core/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>

# include <pcl/io/ply_io.h>
# include <pcl/point_cloud.h>
# include <pcl/point_types.h>
# include <pcl/visualization/pcl_visualizer.h>
# include <pcl/visualization/cloud_viewer.h>
# include <pcl/filters/passthrough.h>

# include "Pixel.h"
# include "DepthPixelColorizer.h"
# include "StaticImageProperties.h"
# include "k4a_grabber.h"

using namespace std;
using namespace pcl;
using namespace sen;
using namespace cv;

const string OUT_DIR = "../clouds/scan_output/";
double xmin, ymin, zmin, xmax, ymax, zmax;

int main(int argc, char** argv){
    // Pre-check
    const uint32_t deviceNum = k4a::device::get_installed_count();
    if (deviceNum == 0){
        cerr << "No Azure Kinect Device Detected. " << endl;
        exit(1);
    }    
    xmin = -1000; xmax = 1000;
    ymin = -1000; ymax = 1000;
    zmin = -1000; zmax = 1000;

    uint32_t frame_saved = 0;
    bool save_flag = false;
    boost::shared_ptr<visualization::PCLVisualizer> viewer(
        new visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setCameraPosition(0.0, 0.0, -2500.0, 1.0, -1.0, 1.0);
    
    boost::mutex mutex;
    PointCloud<PointXYZRGBA>::ConstPtr color_cloud;
    boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr&)> func =
    [&color_cloud, &mutex, &viewer, &save_flag, &frame_saved](const PointCloud<PointXYZRGBA>::ConstPtr &cloud){
        boost::mutex::scoped_lock lock(mutex);
        // cout << "Callback" << endl; // debug
        sleep(1);
        if(!viewer->wasStopped()){
            // Lock
            PointCloud<PointXYZRGBA>::Ptr input_cloud (new PointCloud<PointXYZRGBA>);
            copyPointCloud (*cloud, *input_cloud);
            // cout << "Input cloud size: " << input_cloud->height << endl; // debug
            // Cropping
            PassThrough<PointXYZRGBA> pass;
            pass.setInputCloud(input_cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(xmin, xmax);
            pass.filter(*input_cloud);

            pass.setInputCloud(input_cloud);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(ymin, ymax);
            pass.filter(*input_cloud);

            pass.setInputCloud(input_cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(zmin, zmax);
            pass.filter(*input_cloud);
            
            
            // Saving
            if(save_flag)
            {
                // Continous Saving
                std::stringstream out;
                out << frame_saved;
                std::string name = OUT_DIR + out.str() + ".ply";
                cout << "Saving frame @ " << name << endl;
                frame_saved ++;
                io::savePLYFileASCII(name, *input_cloud); 
            }
        }
        color_cloud = cloud->makeShared();
    };

    // Kinect Azure Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::KinectAzureDKGrabber>(
        0, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);
    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback(func);
    // Start Grabber
    grabber->start();

    while(!viewer->wasStopped()){
        // Update Viewer
        viewer->spinOnce();
        boost::mutex::scoped_try_lock lock(mutex);
        if (lock.owns_lock() && color_cloud)
        {
            // cout << "Updating cloud " << endl; // debug
            if (!viewer->updatePointCloud(color_cloud, "cloud")){
                viewer->addPointCloud(color_cloud, "cloud");
            }    
        }

        cout << "Enter c for changing crop box dimension and s for starting Saving. " << endl;

        if( waitKey(10) == 's'){
            cout << "Saving frame: " << frame_saved << endl;
            frame_saved ++; // ATTENTION HERE
            save_flag = true;
            break;
        }
        else if(waitKey(10) == 'x'){
            cout << "Saving Stopped." << endl;
            save_flag = false;
            break;            
        }
        else if(waitKey(10) == 'c'){
            cout << "Enter xmin xmax ymin ymax zmin zmax" << endl;
            cin >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax;
            cout << "xmin = "<< xmin << " xmax = " << xmax << endl
                    << "ymin = "<< ymin << " ymax = " << ymax << endl
                    << "zmin = "<< zmin << " zmax = " << zmax << endl;            
        }
        // char command = cv::waitKey(30);
        // cout << "command: " << command << endl;
        // switch (command)
        // {
        // case 's':
        //     cout << "Saving frame: " << frame_saved << endl;
        //     frame_saved ++; // ATTENTION HERE
        //     save_flag = true;
        //     break;
        // case 'x':
        //     cout << "Saving Stopped." << endl;
        //     save_flag = false;
        //     break;
        // case 'c':
        //     cout << "Enter xmin xmax ymin ymax zmin zmax" << endl;
        //     cin >> xmin >> xmax >> ymin >> ymax >> zmin >> zmax;
        //     cout << "xmin = "<< xmin << " xmax = " << xmax << endl
        //             << "ymin = "<< ymin << " ymax = " << ymax << endl
        //             << "zmin = "<< zmin << " zmax = " << zmax << endl;
        // }
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