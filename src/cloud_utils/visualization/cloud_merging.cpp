# include <iostream>
# include <vector>
# include <opencv2/core.hpp>

# include <pcl/point_types.h>
# include <pcl/filters/voxel_grid.h>
# include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;
using namespace cv;

int main(int argc, char** argv)
{
    const string input_dir = (string)argv[1];
    const string output_dir = (string)argv[2];
    float leaf_size = atof(argv[3]);

    vector<cv::String> filenames;
    glob(input_dir, filenames);
    size_t n = filenames.size();

    PointCloud<PointNormal>::Ptr final (new PointCloud<PointNormal>);
    PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>);

    for (int i = 0; i < n; i++)
    {
        cout << "[Processing]: " << filenames[i] << endl;
        io::loadPLYFile(filenames[i].c_str(), *cloud);
        *final += *cloud;
    }
    VoxelGrid<PointNormal> subsampler;
    Eigen::Vector4f subsampling_leaf_size (leaf_size, leaf_size, leaf_size, leaf_size);
    subsampler.setLeafSize(subsampling_leaf_size);
    subsampler.setInputCloud(final);
    PointCloud<PointNormal>::Ptr final_filtered (new PointCloud<PointNormal>);
    subsampler.filter(*final_filtered);

    PCL_INFO("Subsampling: %d / %d\n", final_filtered->size(), final->size());
    string output_name = output_dir + "final_sub" + to_string(leaf_size) + "n" + ".ply";
    io::savePLYFileASCII(output_name, *final_filtered);

    return 0;
}