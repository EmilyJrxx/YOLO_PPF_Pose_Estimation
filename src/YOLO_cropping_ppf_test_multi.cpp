# include "YOLO_Detection_class.h"
# include "CloudProcessing.h"
# include <opencv2/core/eigen.hpp>
// # include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace yolo;
using namespace ppf;
using namespace ppf_match_3d;

const int inpHeight = 416;
const int inpWidth = 416;
const double confThreshold = 0.5;
const double nmsThreshold = 0.4;
const string modelConfiguration = "/home/xxwang/Packages/darknet/cfg/yolov3.cfg"; // please configure this to correct path
const string modelWeights = "/home/xxwang/Packages/darknet/yolov3.weights"; // please configure this to correct path
const string classesFile = "/home/xxwang/Packages/darknet/data/coco.names"; // please configure this to correct path

void help(const string& message)
{
    cout << "Program init error: " << message << endl;
    cout << "\nUsage : [program_name] [rgb_file] [depth_file] [cloud_file] [subsample_leaf_size] [outlier_removal_thresh] [detector_file]"<< endl;
    cout << "[rgb file]:               input rgb image (.jpg, .png)" << endl
         << "[depth file]:             input depth map (.tiff, .exr)" << endl
         << "[cloud file]:             input point cloud (.ply)" << endl
         << "[subsample_leaf_size]:    leaf size of voxelgrid for subsampling" << endl
         << "[outlier_removal_thresh]: distance threshold for statistical outlier removal" << endl
         << "[detector_file]:          trained PPF_3D_Detector file" << endl;
    cout << "\nPlease start again with new parameters"<< endl;
}
int main(int argc, char** argv)
{   
    // Parameters
    Eigen::MatrixXd CameraIntr_tmp(3, 4);
    CameraIntr_tmp << 614.384, 0.0, 638.121, 0.0,
                  0.0, 614.365, 364.01, 0.0,
                  0.0, 0.0, 1.0, 0.0;
    Mat CameraIntr;
    eigen2cv(CameraIntr_tmp, CameraIntr);

    if (argc < 7){
        help("Not Enough Arguments");
        exit(1);
    }
    const string img_dir = argv[1];
    const string depth_dir = argv[2];
    const string cloud_dir = argv[3];
    const string detector_filename = argv[6];
    float leafsize = atof(argv[4]);
    float outlierremoval_thresh = atof(argv[5]);

    vector<string> img_files, depth_files, cloud_files;
    cv::glob(img_dir, img_files, false);
    cv::glob(depth_dir, depth_files, false);
    cv::glob(cloud_dir, cloud_files, false);
    
    ppf::CloudProcessor cloud_processor;
    string trained_detector = "../data/detector_bottle.xml";
    string bottle_file = "../data/bottle_remesh_meter_normalized.ply";
    Mat bottle = ppf_match_3d::loadPLYSimple(bottle_file.c_str(), 1);
    cloud_processor.LoadSingleModel(bottle, "bottle");
    cloud_processor.LoadTrainedDetector("bottle", detector_filename);

    for (int k = 0; k < img_dir.size(); k++)
    {
        string img_filename = img_files[k];
        string depth_filename = depth_files[k];
        string cloud_filename = cloud_files[k];
        Mat scene_rgb = imread(img_filename, 1);
        Mat scene_depth = imread(depth_filename, IMREAD_ANYDEPTH);
        vector<Mat> rgb_input; rgb_input.push_back(scene_rgb);
        vector<Mat> depth_input; depth_input.push_back(scene_depth);

        yolo::YOLODetector yolo_detector(modelConfiguration, modelWeights);
        yolo_detector.LoadClassNames(classesFile);
        yolo_detector.reloadImages(rgb_input, depth_input);
        vector<Mat> outs[1];
        yolo_detector.detect(outs, inpWidth, inpHeight);
        cout << "yolo out size: " << outs->size() << endl
            << "yolo out shape: " << outs[0][0].rows << " " << outs[0][0].cols << endl;
        
        yolo_detector.postprocess(outs, confThreshold, nmsThreshold);

        // yolo_detector.display();

        vector<Rect> bboxes_2d; vector<int> classIds; vector<int> indices;
        yolo_detector.TransferResults(0, bboxes_2d, classIds, indices);
        cout << "boxes size: " << bboxes_2d.size() << endl
            << "classIds size: " << classIds.size() << endl
            << "indices size: " << indices.size() << endl; //debug  
        // for (int i = 0; i < bboxes_2d.size(); i++)
        // {
        //     Rect box = bboxes_2d[i];
        //     cout << "box" << i << endl;
        //     cout << box.x << " " << box.x + box.width << endl
        //          << box.y << " " << box.y + box.height << endl;
        // }

        int64 tick1 = cv::getTickCount();
        PointCloud<PointXYZ>::Ptr scene_cloud (new PointCloud<PointXYZ>);
        io::loadPLYFile(cloud_filename, *scene_cloud);
        int64 tick2 = cv::getTickCount();
        cloud_processor.ReloadScenes(scene_cloud, scene_depth, bboxes_2d, classIds, indices);
        // ppf::CloudProcessor cloud_processor(scene_cloud, scene_depth, bboxes_2d, classIds, indices);
        

        // scene cropping
        vector<PointCloud<PointXYZ> > objects;

        objects = cloud_processor.SceneCropping(CameraIntr);
        int64 tick3 = cv::getTickCount();
        cout << "Time Consuming: " << (tick2-tick1)/cv::getTickFrequency() << " " << (tick3-tick1)/cv::getTickFrequency() << endl;

        // scene post-processing: downsample
        cloud_processor.Subsampling(leafsize);
        // scene post-processing: statistical outlier-removal
        cloud_processor.OutlierProcessing(50, outlierremoval_thresh);
        // normal_estimation
        vector<PointCloud<PointNormal> > objects_with_normals;
        objects_with_normals = cloud_processor.NormalEstimation(30);
        vector<PointCloud<PointNormal> > object_edges;
        object_edges = cloud_processor.EdgeExtraction(0.03); 
        // scene saving

        for (int i = 0; i < objects.size(); i++)
        {
            string filename = to_string(i) + ".ply";
            io::savePLYFileASCII(filename, objects_with_normals[i]);
        }

        // PPF detector load 3D model
        // string bottle_file = "../data/bottle_remesh_meter_normalized.ply";
        // // string trained_detector = "../data/detector_bottle.xml";
        // Mat bottle = ppf_match_3d::loadPLYSimple(bottle_file.c_str(), 1);
        // cloud_processor.LoadSingleModel(bottle, "bottle");

        // PPF detector load saved model
        // cloud_processor.LoadTrainedDetector("bottle", detector_filename);
        // PPF training detector
        // cloud_processor.TrainDetector(trained_detector, true);
        // PPF Matching
        Mat object_wn_mat, edges_mat;
        cloud_processor.PointCloudXYZNormalToMat(object_wn_mat, objects_with_normals[0].makeShared());
        cloud_processor.PointCloudXYZNormalToMat(edges_mat, object_edges[0].makeShared());
        Pose3D result_pose = cloud_processor.Matching_S2B("bottle", object_wn_mat, edges_mat);
        cout << "Result Pose: " << endl;
        result_pose.printPose(); // print pose
        Mat object_trans = transformPCPose(bottle, result_pose.pose);
        cout << "Transformed. " << endl;
        // writePLY(object_trans, argv[7]);
        // cout << "Written. " << endl;
    }


    return 0;
}
