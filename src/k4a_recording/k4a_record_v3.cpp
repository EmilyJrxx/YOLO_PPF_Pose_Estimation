#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

# include "Pixel.h"
# include "DepthPixelColorizer.h"
# include "StaticImageProperties.h"
# include "k4a_grabber.h"

using namespace std;
using namespace sen;
using namespace pcl;

const string OUT_DIR = "../clouds/scan_output/";
double xmin, ymin, zmin, xmax,ymax,zmax;

class SimpleOpenNIViewer
{
    public:
        SimpleOpenNIViewer (): cloud_viewer ("PCL viewer")
        {
          frames_saved = 0;
          save_one = false;
          visualization::PCLVisualizer::Ptr viewer (
            new visualization::PCLVisualizer("PCL Cloud Viewer"));
          visualizer = viewer;
          visualizer->setBackgroundColor(0, 0, 0);
          visualizer->setCameraPosition(0.0, 0.0, -2500.0, 1.0, -.0, 1.0);
        }

        void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
        {
          // sleep (1);
            if (!cloud_viewer.wasStopped())
            {

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


                copyPointCloud  (*cloud,*input_cloud);
                // cout << "Cloud Size: " << input_cloud->size() << endl;
                pcl::PassThrough<pcl::PointXYZRGB> pass;

                pass.setInputCloud (input_cloud);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (xmin, xmax);
                pass.filter (*input_cloud);
                cloud_viewer.showCloud (input_cloud);
                
                pass.setInputCloud (input_cloud);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (ymin, ymax);
                pass.filter (*input_cloud);
                cloud_viewer.showCloud (input_cloud);

                pass.setInputCloud (input_cloud);
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (zmin, zmax);
                pass.filter (*input_cloud);
                cloud_viewer.showCloud (input_cloud);

                
                if (!visualizer->updatePointCloud(input_cloud, "cloud"))
                {
                  cout << "add cloud" << endl;
                  visualizer->addPointCloud(input_cloud, "cloud");
                }

                if( save_one )
                {
                    // save_one = false;
                    std::stringstream out;
                    out << frames_saved;
                    std::string name = OUT_DIR  + out.str() + ".ply";
                    cout << "Saving frame @" << name << ".\n";
                    frames_saved++;
                    pcl::io::savePLYFileASCII( name, *input_cloud );
                }
            }
        }

        void run ()
        {
            pcl::Grabber* grabber = new pcl::KinectAzureDKGrabber(
                0, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);
            // pcl::Grabber* interface = new pcl::OpenNIGrabber();

            boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

            // interface->registerCallback (f);

            // interface->start ();
            grabber->registerCallback(f);
            
            // k4a_device_configuration_t conf = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
            // conf.camera_fps   = K4A_FRAMES_PER_SECOND_30;
            // conf.depth_mode   = K4A_DEPTH_MODE_NFOV_UNBINNED;
            // conf.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
            // conf.color_resolution = K4A_COLOR_RESOLUTION_720P;
            // conf.synchronized_images_only = true; // only synchronized images allowed
            grabber->start();

            char c;

            while (!cloud_viewer.wasStopped())
            {


                cout << "enter c for changing crop box dimensions and s for starting Saving  ";
                cout << "Cloud Fps: " << grabber->getFramesPerSecond() << endl;
                c = getchar();
                if( c == 's' )
                {
                  cout << "Saving frame " << frames_saved << ".\n";
                  frames_saved++;
                  save_one = true;
                }

                else if( c == 'x' )
                {
                  cout << "Saving stopped .\n";
                  save_one = false;
                }

                else if(c=='c')
                {

                  cout << "Enter xmin xmax ymin ymax zmin zmax \n";
                  cin >> xmin>> xmax>> ymin >>ymax>> zmin>> zmax;
                  cout << " xmin= "<<xmin<< " xmax= "<<xmax<< " ymin= "<<ymin <<" ymax= "<<ymax<<" zmin= " << zmin<<" zmax= "<< zmax<< ".\n";

                }
            }
            grabber->stop();
        }
               
    private:
        int frames_saved;
        bool save_one;
        boost::shared_ptr<visualization::PCLVisualizer> visualizer;
        pcl::visualization::CloudViewer cloud_viewer;
};

int main ()
{
  xmin=-1000; ymin= -1000;zmin= -1000;
  xmax=1000; ymax= 1000;zmax=1000;
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}