# include <iostream>
# include <pcl/point_types.h>
# include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv){
    PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
    const string input_dir = "../clouds/cup/cropped/";
    const string output_dir = "../clouds/cup/cropped/";
    for (int num = 1y; num <=9; num++){  
        cout << "Processing " << num << "th cloud" << endl;
        string input_name = input_dir + to_string(num) + ".ply";
        string output_name = output_dir + "00" + to_string(num) + ".ply";
        io::loadPLYFile(input_name.c_str(), *cloud);
        uint32_t size = cloud->size();
        PointCloud<PointXYZRGB>::iterator begin = cloud->begin();
        for (uint32_t i = size-1; i > 0; i--)
        {
            PointXYZRGB point = cloud->points[i];
            if (point.x == 0 && point.y == 0 && point.z == 0)
                cloud->erase(begin + i);
            printf("\rProcessing: %d / %d", (size-i), size);
        }
        printf("\n");
        io::savePLYFileASCII(output_name.c_str(), *cloud);
    }

    return 0;
}