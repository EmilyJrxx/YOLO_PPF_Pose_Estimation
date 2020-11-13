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
# include <pcl/filters/passthrough.h>

# include "Pixel.h"
# include "DepthPixelColorizer.h"
# include "StaticImageProperties.h"
# include "k4a_grabber.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace 