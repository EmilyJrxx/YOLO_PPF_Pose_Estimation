# include <iostream>
# include <fstream>
# include <opencv2/core/core.hpp>
# include <opencv2/highgui.hpp>
# include <opencv2/surface_matching.hpp>
# include <opencv2/surface_matching/ppf_helpers.hpp>
# include <pcl/io>

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

int main(int argc, char** argv)
{
    string model_file = argv[1];
    string detector_file = argv[2];

    Mat model = loadPLYSimple(model_file.c_str(), 1);
    
    PPF3DDetector detector (0.025, 0.05);
    int64 tick1 = getTickCount();
    detector.trainModel(model);
    int64 tick2 = getTickCount();
    double training_time = (double)(tick2 - tick1) / getTickFrequency();
    cout << training_time << "sec" << endl;

    ifstream detectorFile(detector_file);
    if (!detectorFile.good())
    {
        FileStorage fsOut(detector_file, FileStorage::WRITE);
        detector.write(fsOut);
        fsOut.release();
        cout << "Detector saved: " << detector_file << endl;
    }
    else
    {
        CV_Error(1, "SavingError: Cannot open writing file.");
        exit(1);
    }
    
    return 0;
}