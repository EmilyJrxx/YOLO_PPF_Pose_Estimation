# include <iostream>
# include <cmath>

# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        cerr << "args not enough, milli_to_meter [input_file] [output_file]\n";
        exit(1);
    }
    string input_filename = argv[1];
    string output_filename = argv[2];

    PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>);
    io::loadPLYFile(input_filename, *cloud);

    for (size_t i = 0; i < cloud->size(); i++)
    {
        float nx = cloud->points[i].normal_x;
        float ny = cloud->points[i].normal_y;
        float nz = cloud->points[i].normal_z;
        float norm = sqrt(nx*nx + ny*ny + nz*nz);
        cloud->points[i].normal_x /= norm;
        cloud->points[i].normal_y /= norm;
        cloud->points[i].normal_z /= norm;
    }
    cout << "Processing Done. " << endl;
    io::savePLYFileASCII(output_filename, *cloud);
    return 0;
}