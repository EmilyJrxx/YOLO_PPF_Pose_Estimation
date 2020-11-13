# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>

using namespace pcl;
using namespace std;

typedef PointNormal PointT;

int main(int argc, char** argv)
{
    string input = argv[1];
    // size_t tail = input_dir.find_first_of(".ply");
    string output = argv[2];

    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    io::loadPLYFile(input, *cloud);

    for(size_t i = 0; i < cloud->size(); i++)
    {
        cloud->points[i].x /= 1000;
        cloud->points[i].y /= 1000;
        cloud->points[i].z /= 1000;
    }

    io::savePLYFileASCII(output, *cloud);
    
    return 0;
}