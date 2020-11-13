# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>

# include <iostream>

using namespace std;
using namespace pcl;

typedef PointNormal PointT;

int main(int argc, char** argv)
{
    string input_name = argv[1];
    string output_name = argv[2];
    float offset = atof(argv[3]);

    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    io::loadPLYFile(input_name, *cloud);

    // normalize normals
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
    // configurate offset
    for (size_t i = 0; i < cloud->size(); i++)
    {
        cloud->points[i].z += offset;
    }   
    io::savePLYFileASCII(output_name, *cloud);

    return 0;
}