# include <k4a/k4a.h>
# include <k4a/k4a.hpp>

# include <iostream>
# include <vector>

# include <opencv4/opencv2/core/core.hpp>
# include <opencv4/opencv2/highgui/highgui.hpp>

# include <pcl/io/ply_io.h>
# include <pcl/point_cloud.h>
# include <pcl/point_types.h>
# include <pcl/visualization/pcl_visualizer.h>

# include "Pixel.h"
# include "DepthPixelColorizer.h"
# include "StaticImageProperties.h"


using namespace std;
using namespace pcl;
using namespace cv;
using namespace boost;
using namespace sen;
// struct Pixel{
//     // using a BGRA8 pattern
//     uint8_t Blue;
//     uint8_t Green;
//     uint8_t Red;
//     uint8_t Alpha;
// };
int main(int argc, char** argv){
    const uint32_t deviceNum = k4a::device::get_installed_count();
    if (deviceNum == 0){
        cerr << "No Azure Kinect Device Detected. " << endl;
        exit(1);
    }
    
    // Configuration
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL; // init
    config.camera_fps   = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode   = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.synchronized_images_only = true; // only synchronized images allowed

    // Starting Device
    cout << "Opening K4A device ... " << endl;
    k4a::device dev = k4a::device::open(K4A_DEVICE_DEFAULT);
    dev.start_cameras(& config);
    cout << "K4A device Started." << endl;

    k4a::image depth_img;
    k4a::image color_img;

    vector<Pixel> depth_buffer;
    k4a::capture capture;
    while(1)
    {
        if (dev.get_capture(&capture, std::chrono::milliseconds(0)))
        {
            depth_img = capture.get_depth_image();
            color_img = capture.get_color_image();
            uint8_t *color_buffer = color_img.get_buffer();
            // uint8_t *depth_buffer = depth_img.get_buffer();
            ColorizeDepthImage(depth_img, DepthPixelColorizer::ColorizeBlueToRed, 
                GetDepthModeRange(config.depth_mode), &depth_buffer);

            uint16_t depth_h = depth_img.get_height_pixels();
            uint16_t depth_w = depth_img.get_width_pixels();
            uint16_t color_h = color_img.get_height_pixels();
            uint16_t color_w = color_img.get_width_pixels();

            Mat depth_frame = Mat(depth_h, depth_w, CV_8UC4, &depth_buffer[0]);
            Mat color_frame = Mat(color_h, color_w, CV_8UC4, color_buffer);

            imshow("Azure Kinect color", color_frame);
            imshow("Azure Kinect Depth", depth_frame);
            cout << "FPS: " << int(config.camera_fps) << endl;
            cout << "FPS: " << k4a_fps_t(config.camera_fps) << endl;
        }
        if (waitKey(1) == 27 || waitKey(1) == 'q')
        {
            cout << "quit" << endl;
            dev.close();
            break;
        }
    }
    
    return 0;
}
