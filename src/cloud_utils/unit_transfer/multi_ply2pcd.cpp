# include <iostream>

# include <opencv4/opencv2/core.hpp>

# include <pcl/io/pcd_io.h>
# include <pcl/io/ply_io.h>
# include <pcl/point_types.h>

using namespace std;
using namespace cv;
using namespace pcl;

const string input_dir = "../clouds/cup2/";
const string output_dir = "../clouds/cup2_pcd/";

int main(int argc, char** argv)
{
    const string input_dir = (string)argv[1];
    const string output_dir = (string)argv[2];
    vector<cv::String> filenames;
    cv::glob(input_dir, filenames);
    PointCloud<PointXYZRGBA>::Ptr rgb_cloud (new PointCloud<PointXYZRGBA>);

    for (int i = 0; i < filenames.size(); i++)
    {
        cout << "[reading]: " << filenames[i] << endl;
        int start = filenames[i].find_first_not_of(input_dir);
        int end = filenames[i].find(".ply");
        string num = filenames[i].substr(start, end - start);
        io::loadPLYFile(filenames[i].c_str(), *rgb_cloud);
        string pcd_filename = (string)(output_dir + num + ".pcd");
        cout << "[writing]: " << pcd_filename << endl;
        io::savePCDFileASCII(pcd_filename.c_str(), *rgb_cloud);
    }
    return 0;
}