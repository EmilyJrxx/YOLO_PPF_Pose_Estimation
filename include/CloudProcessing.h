// CloudProcessing.h
// Description: this header file contains functions that mainly 
// perform: Mask deprojection, Cloud Cropping, PPFMatching
// Input:
//      bounding boxes
//      (corresponding) classids - for selecting proper 3d model
//      3D Models
//      trained PPF model
// Output:
//      Transformation Matrices
//      (corresponding) confidence scores
// # include "YOLO_Detection_class.h"
# include "Camera.h"

# include <iostream>
# include <fstream>
// # include <ros/ros.h>
# include <opencv4/opencv2/surface_matching/ppf_helpers.hpp>
# include <opencv4/opencv2/surface_matching.hpp>

# include <pcl/features/normal_3d_omp.h>
# include <pcl/kdtree/kdtree_flann.h>
# include <pcl/filters/voxel_grid.h>
# include <pcl/filters/statistical_outlier_removal.h>
# include <pcl/filters/crop_hull.h>
# include <pcl/surface/convex_hull.h>

using namespace ppf_match_3d;
using namespace cv;
using namespace ppf_utils;
using namespace pcl;
using namespace std;

namespace ppf
{
    class CloudProcessor
    {
        private:
            PointCloud<PointXYZ> scene;
            PointCloud<PointNormal> scene_with_normal;
            vector<PointCloud<PointXYZ> > objects;
            vector<PointCloud<PointNormal> > objects_with_normals;
            vector<PointCloud<PointNormal> > objects_edges;
            vector<Mat>  final_objects;
            vector<Mat>  final_edges;

            Mat  depth;
            vector<Mat>  models;
            vector<double> confi;
            map<double, string> id_to_label;
            map<string, double> label_to_id;
            vector<bool> if_trained;

            double relativeSamplingStep; 
            double relativeDistanceStep;
        public: 
            // boost::shared_ptr<PPF3DDetector> detector;
            vector<PPF3DDetector> detectors;
            vector<Rect> boxes;
            vector<int>  classIds;
            vector<int>  indices;

            CloudProcessor(PointCloud<PointXYZ>::Ptr scene_input, Mat depth_input, vector<Rect>& boxes_input, 
                vector<int>& classIds_input, vector<int>& indices_input, double relativeSamplingStep_input=0.025,
                double relativeDistanceStep_input = 0.05)
            {
                scene = *scene_input;
                depth = depth_input;
                boxes = boxes_input;
                classIds = classIds_input;
                indices = indices_input;
                
                relativeSamplingStep = relativeSamplingStep_input;
                relativeDistanceStep = relativeDistanceStep_input;
                // for (int i = 0; i < if_trained.size(); i++){
                //     if_trained[i] = false;
                // }
            }
            ~CloudProcessor(){
                for (uint32_t i = 0; i < detectors.size(); i++){
                    detectors[i].~PPF3DDetector();
                }
            }
            void LoadTrainedDetector(const string name, const string TrainedDetectorFile);
            void ReloadScenes(PointCloud<PointXYZ>::Ptr scene, Mat depth_input, vector<Rect>& boxes_input, 
                vector<int>& classIds_input, vector<int>& indices_input);

            void MatToPointCloudXYZ(Mat& cv_cloud, PointCloud<PointXYZ>::Ptr pcl_cloud);
            void PointCloudXYZNormalToMat(Mat& cv_cloud, PointCloud<PointNormal>::Ptr pcl_cloud);

            void LoadModels(vector<Mat>& models, vector<string> labels);
            void LoadSingleModel(Mat model, string label);
            void TrainDetector(const string TrainedDetectorDir, bool saveflag, const double relativeSamplingSetp, 
                const double relativeDistanceStep); // train a new detector
            void Deprojection(Mat CameraIntr);
            vector<PointCloud<PointXYZ> > SceneCropping(Mat CameraIntr);
            vector<PointCloud<PointNormal> > NormalEstimation(int k); // k-nn neighbor number k
            vector<PointCloud<PointNormal> > EdgeExtraction(float curvThreshold); // extracting geometric edges
            vector<PointCloud<PointXYZ> > Subsampling(double leaf_size);
            vector<PointCloud<PointXYZ>> OutlierProcessing(int meanK, double Thresh);
            Pose3D Matching(const string ModelName, Mat scene, double relativeSceneSampleStep, 
                          double relativeSceneDistance);
            Pose3D Matching_S2B(const string name, Mat scene, Mat edge, double relativeSceneSampleStep, 
                                  double relativeSceneDistance);
    };
    void CloudProcessor::LoadTrainedDetector(const string name, const string TrainedDetectorFile)
    {
        int id = label_to_id[name];
        cout << "Deserializing..." << endl;
        int64 tick1 = cv::getTickCount(); 
        FileStorage fsload(TrainedDetectorFile, FileStorage::READ);
        detectors[id].read(fsload.root());
        fsload.release();
        int64 tick2 = cv::getTickCount();

        cout << "Deserialization complete in "
             << (double)(tick2 - tick1) / cv::getTickFrequency()
             << " sec" << endl;
        
        if_trained[id] = true;
    }
    void CloudProcessor::ReloadScenes(PointCloud<PointXYZ>::Ptr scene_input, Mat depth_input, vector<Rect>& boxes_input, 
                vector<int>& classIds_input, vector<int>& indices_input)
    {
        // clear old input
        scene.clear();
        boxes.clear();
        classIds.clear();
        indices.clear();

        scene_with_normal.clear();
        objects_edges.clear();
        objects.clear();
        objects_with_normals.clear();
        final_objects.clear();
        final_edges.clear();

        // load new input
        scene = *scene_input;
        depth = depth_input;
        boxes = boxes_input;
        classIds = classIds_input;
        indices = indices_input;        
    }
    void CloudProcessor::MatToPointCloudXYZ(Mat& cv_cloud, PointCloud<PointXYZ>::Ptr pcl_cloud){
        pcl_cloud->height = (uint32_t)cv_cloud.rows;
        pcl_cloud->width  = (uint32_t)cv_cloud.cols;
        pcl_cloud->is_dense = false;
        pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);
        uint32_t rows = cv_cloud.rows;
        uint32_t cols = cv_cloud.cols;
    #ifdef _OPENMP
        cout << "trans with OpenMP" << endl;
        #pragma omp parallel for
    #endif
        for( uint32_t i = 0; i < rows; i++)
        {
            pcl_cloud->points[i].x = cv_cloud.at<ushort>(i,0);
            pcl_cloud->points[i].y = cv_cloud.at<ushort>(i,1);
            pcl_cloud->points[i].z = cv_cloud.at<ushort>(i,2);
        }
    }
    void CloudProcessor::PointCloudXYZNormalToMat(Mat& cv_cloud, PointCloud<PointNormal>::Ptr pcl_cloud){
        uint32_t rows = pcl_cloud->size();
        uint32_t cols = 6;
        cout << rows << " " << cols << endl;
        cv_cloud = Mat(rows, cols, CV_32FC1);
        // cv_cloud.resize(rows, cols);
    #ifdef _OPENMP
        cout << "trans with OpenMP" << endl;
        #pragma omp parallel for
    #endif
        for (uint32_t i = 0; i < rows; i++)
        {
            float* data = cv_cloud.ptr<float>(i);
            data[0] = pcl_cloud->points[i].x;
            data[1] = pcl_cloud->points[i].y;
            data[2] = pcl_cloud->points[i].z;

            data[3] = pcl_cloud->points[i].normal_x;
            data[4] = pcl_cloud->points[i].normal_y;
            data[5] = pcl_cloud->points[i].normal_z; 
            double A = sqrt(data[3]*data[3] + data[4]*data[4] + data[5]*data[5]);
            if (A > 0.00001){
                data[3] /= static_cast<float>(A);
                data[4] /= static_cast<float>(A);
                data[5] /= static_cast<float>(A);            
            }
        }
    }
    void  CloudProcessor::LoadModels(vector<Mat>& models_input, vector<string> labels)
    {
        // load models with surface normals
        models = models_input;
        cout << "Models Loaded: " << models.size() << endl;
        for (int i = 0; i < models.size(); i++)
        {
            models.push_back(models_input[1]);
            label_to_id[labels[i]] = i;
            id_to_label[i] = labels[i];
            if_trained.push_back(false);
            
            // Init PPF detectors

            ppf_match_3d::PPF3DDetector detector(relativeSamplingStep, relativeDistanceStep);
            detectors.push_back(detector);
        }
    }
    void CloudProcessor::LoadSingleModel(Mat model_input, string label)
    {
        models.push_back(model_input);
        int id = models.size() - 1;
        if_trained.push_back(false);
        label_to_id[label] = id;
        id_to_label[id] = label;

        ppf_match_3d::PPF3DDetector detector(relativeSamplingStep, relativeDistanceStep);
        detectors.push_back(detector); 

        cout << "Loaded, having " << models.size() << " models now. " << endl; 
    }
    void CloudProcessor::TrainDetector(const string TrainedDetectorDir, bool saveflag = false, const double relativeSamplingSetp_train = 0.025, 
        const double relativeDistanceStep_train = 0.5)
        {
            if (models.size() == 0)
            {
                CV_Error(1, "No 3D model loaded");
                return;
            }
            uint32_t size = models.size();
            for (uint32_t i = 0; i < size; i++)
            {
                cout << "Training the "<< (i+1) <<" model in: ";
                ppf_match_3d::PPF3DDetector detector(relativeSamplingSetp_train, relativeDistanceStep_train);
                int64 tick1 = getTickCount();
                detector.trainModel(models[i]);
                int64 tick2 = getTickCount();
                double training_time = (double)(tick2 - tick1) / getTickFrequency();
                cout << training_time << "sec" << endl;
                detectors[i] = detector;

                if (saveflag == true)
                {
                    string FileName = TrainedDetectorDir + "detector_"
                                    + id_to_label[i] + ".xml";
                    ifstream detectorFile(FileName);
                    if (!detectorFile.good())
                    {
                        FileStorage fsOut(FileName, FileStorage::WRITE);
                        detector.write(fsOut);
                        fsOut.release();
                        cout << "Detector " << (i+1) << " saved: " << FileName << endl;
                    }
                    else
                    {
                        CV_Error(1, "SavingError: Error opening ");
                        return;
                    }
                }
            }
    }
    void CloudProcessor::Deprojection(Mat CameraIntr){}
    vector<PointCloud<PointXYZ> > CloudProcessor::SceneCropping(const Mat CameraIntr)
    {
        // Pre-check
        // Processing
        const double ppx = CameraIntr.at<double>(0, 2); // y(row) = 0, x(col) = 2 
        const double ppy = CameraIntr.at<double>(1, 2);
        const double fx  = CameraIntr.at<double>(0, 0);
        const double fy  = CameraIntr.at<double>(1, 1);
        cout << "Intr: " << fx << " " << fy << " "<< ppx << " " << ppy << endl;
        Camera camera(fx, fy, ppx, ppy);

        int depth_rows = depth.rows;
        int depth_cols = depth.cols;
        for (uint32_t i = 0; i < boxes.size(); i++){
            cout << "Processing on " << (i+1) << "th boundingbox." << endl;

            double left   = boxes[i].x - 30; if (left < 0) left = 0;
            double top    = boxes[i].y - 30; if (top < 0) top = 0;
            double right  = boxes[i].x + boxes[i].width + 30; if (right >= depth_cols) right = depth_cols - 1; 
            double bottom = boxes[i].y + boxes[i].height + 30; if (bottom >= depth_rows) bottom = depth_rows - 1;
            
            float depth_1 = depth.at<float>(top, left);
            float depth_2 = depth.at<float>(top, right);
            float depth_3 = depth.at<float>(bottom, left);
            float depth_4 = depth.at<float>(bottom, right);
            float depth_avg = (depth_1 + depth_2 + depth_3 + depth_4) / 4;
            PointXYZ left_top  = camera.back_projection_bbox(depth_avg, left, top); // y = top, x = left
            PointXYZ left_bot  = camera.back_projection_bbox(depth_avg, left, bottom);
            PointXYZ right_top = camera.back_projection_bbox(depth_avg, right, top);
            PointXYZ right_bot = camera.back_projection_bbox(depth_avg, right, bottom);

            // double z_c = (left_top.z + left_bot.z + right_top.z + right_bot.z) / 4;
            // z_c = z_c + 0.2; // add diameter of model
            left_top.z  += 0.15;
            left_bot.z  += 0.15;
            right_top.z += 0.15;
            right_bot.z += 0.15;

            // double z_f = z_c - 2 * 0.2; if (z_f < 0) z_f = 0;
            // PointXYZ left_top_front = left_top; left_top_front.z = z_f;
            // PointXYZ left_bot_front = left_bot; left_bot_front.z = z_f;
            // PointXYZ right_top_front = right_top; right_top_front.z = z_f;
            // PointXYZ right_bot_front = right_bot; right_bot_front.z = z_f;
            PointCloud<PointXYZ>::Ptr boundingbox_3d (new PointCloud<PointXYZ>);
            boundingbox_3d -> push_back(left_top);
            boundingbox_3d -> push_back(left_bot);
            boundingbox_3d -> push_back(right_top);
            boundingbox_3d -> push_back(right_bot);
            boundingbox_3d -> push_back(PointXYZ(0.0, 0.0, 0.0));
            // boundingbox_3d -> push_back(left_top_front);
            // boundingbox_3d -> push_back(left_bot_front);
            // boundingbox_3d -> push_back(right_top_front);
            // boundingbox_3d -> push_back(right_bot_front);
            
            ConvexHull<PointXYZ> hull;
            hull.setInputCloud(boundingbox_3d);
            hull.setDimension(3);
            vector<Vertices> polygons;
            PointCloud<PointXYZ>::Ptr surface_hull (new PointCloud<PointXYZ>);
            hull.reconstruct(*surface_hull, polygons);
            PointCloud<PointXYZ>::Ptr cropped (new PointCloud<PointXYZ>);
            CropHull<PointXYZ> bb_filter;  

            PointCloud<PointXYZ>::Ptr scene_tmp (new PointCloud<PointXYZ>);
            *scene_tmp = scene;
            bb_filter.setDim(3);
            bb_filter.setInputCloud(scene_tmp);
            bb_filter.setHullIndices(polygons);
            bb_filter.setHullCloud(surface_hull);
            bb_filter.filter(*cropped);

            printf("Cropping : %d / %d \n", scene_tmp->size(), cropped->size());
            objects.push_back(*cropped);
        }

        return objects;
    }
    vector<PointCloud<PointXYZ>> CloudProcessor::OutlierProcessing(int meanK = 50, double Thresh = 1.5)
    {
        StatisticalOutlierRemoval<PointXYZ> sor (new StatisticalOutlierRemoval<PointXYZ>);
        PointCloud<PointXYZ>::Ptr object_filtered (new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr object (new PointCloud<PointXYZ>);
        for (size_t i = 0; i < objects.size(); i++)
        {
            cout << "OutlierRemoval: Processing on " << (i+1) << "th object" << endl;
            *object = objects[i];
            cout << "Before Filtering: " << object->size() << endl; //debug
            sor.setInputCloud(object);
            sor.setMeanK(meanK);
            sor.setStddevMulThresh(Thresh);
            sor.filter(*object_filtered);
            cout << "After Filtering: " << object_filtered->size() << endl;
            objects[i] = *object_filtered;
        }
        return objects;
    }
    vector<PointCloud<PointXYZ> > CloudProcessor::Subsampling(double leafsize)
    {
        VoxelGrid<PointXYZ> sub;
        PointCloud<PointXYZ>::Ptr object_filtered (new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr object (new PointCloud<PointXYZ>);
        Eigen::Vector4f sub_leaf_size(leafsize, leafsize, leafsize, leafsize);
        for (size_t i = 0; i < objects.size(); i++)
        {
            cout << "SubSampling: Processing on " << (i+1) << "th object" << endl;
            *object = objects[i];
            cout << "Before Filtering: " << object->size() << endl; //debug
            sub.setInputCloud(object);
            sub.setLeafSize(sub_leaf_size);
            sub.filter(*object_filtered);
            cout << "After Filtering: " << object_filtered->size() << endl;
            objects[i] = *object_filtered;
        }
        return objects;
    }
    vector<PointCloud<PointNormal> > CloudProcessor::NormalEstimation(int k = 30)
    {
        NormalEstimationOMP<PointXYZ, Normal> ne;
        search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
        PointCloud<PointNormal>::Ptr object_filtered (new PointCloud<PointNormal>);
        PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
        PointCloud<PointXYZ>::Ptr object (new PointCloud<PointXYZ>);
        for (size_t i = 0; i < objects.size(); i++)
        {
            cout << "NormalEstimation: Processing on " << (i+1) << "th object" << endl;
            *object = objects[i];
            cout << "Input Size: " << object->size() << endl;
            ne.setInputCloud(object);
            ne.setNumberOfThreads(12);
            ne.setSearchMethod(tree);
            ne.setKSearch(k);
            ne.compute(*normals);
            concatenateFields(*object, *normals, *object_filtered);
            cout << "Cloud writing." << endl;
            objects_with_normals.push_back(*object_filtered);
        }
        cout << "Normal Estimation Done." << endl;
        return objects_with_normals;
    }
    vector<PointCloud<PointNormal> > CloudProcessor::EdgeExtraction(float curvThreshold)
    {
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
        for(size_t k = 0; k < objects.size(); k++)
        {
            PointCloud<PointNormal> object = objects_with_normals[k];
            PointCloud<PointNormal> edge;
            size_t rows = object.size();
            if(rows == 0)
            {
                cout << "surface normal has not been calculated.\n";
            }
            for(size_t i = 0; i < rows; i++)
            {
                PointNormal point = object.points[i];
                if(point.curvature > curvThreshold){
                    edge.push_back(point);
                }
            }
            objects_edges.push_back(edge);
            cout << "edges size: " << objects_edges[k].size() << endl;            
        }
        return objects_edges;
    }
    Pose3D CloudProcessor::Matching(const string name, Mat scene, double relativeSceneSampleStep = 0.0714, 
                                  double relativeSceneDistance = 0.05)
    {
        int id = label_to_id[name];
        PPF3DDetector detector = detectors[id];
        vector<Pose3DPtr> results;
        // Pre check
        if (!if_trained[id])
        {
            CV_Error(2, "Model [" + name + "] not trained yet.");
            exit(1);
        }
        cout << "Start Matching [" << name << "]" << endl;
        int64 tick1 = cv::getTickCount();
        detector.match(scene, results, relativeSceneSampleStep, relativeSceneDistance);
        int64 tick2 = cv::getTickCount();
        double matching_time = (double)(tick2-tick1) / cv::getTickFrequency();
        cout << endl << "PPF Elapsed Time " <<
            (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;
        
        // PostRefinement
        size_t results_size = results.size();
        if (results_size == 0)
        {
            cout << "No matching Poses found. Exiting." << endl;
            exit(0);
        }
        size_t N = 5;
        if (results_size < N) {
            cout << endl << "Reducing matching poses to be reported (as specified in code): "
                << N << " to the number of matches found: " << results_size << endl;
            N = results_size;
        }
        vector<Pose3DPtr> resultsSub(results.begin(), results.begin()+N);
        vector<double> resultsscores;

        // Create an instance of ICP
        ICP icp(100, 0.005f, 2.5f, 8);
        int64 t1 = cv::getTickCount();
        
        // Register for all selected poses
        cout << endl << "Performing ICP on " << N << " poses..." << endl;
        icp.registerModelToScene(models[id], scene, resultsSub);
        int64 t2 = cv::getTickCount();
        
        double icp_time = (double)(tick2-tick1) / cv::getTickFrequency();
        cout << endl << "ICP Elapsed Time " <<
            (t2-t1)/cv::getTickFrequency() << " sec" << endl;
        
        return *resultsSub[0]; // return transformation: model -> scene
        
        // TODO: Pose Validation
    }
    Pose3D CloudProcessor::Matching_S2B(const string name, Mat scene, Mat edge, double relativeSceneSampleStep = 0.05, 
                                  double relativeSceneDistance = 0.05)
    {
        int id = label_to_id[name];
        PPF3DDetector detector = detectors[id];
        vector<Pose3DPtr> results;
        // Pre check
        if (!if_trained[id])
        {
            CV_Error(2, "Model [" + name + "] not trained yet.");
            exit(1);
        }
        cout << "Start Matching [" << name << "]" << endl;
        int64 tick1 = cv::getTickCount();
        detector.match_S2B(scene, edge, results, relativeSceneSampleStep, relativeSceneDistance);
        int64 tick2 = cv::getTickCount();
        double matching_time = (double)(tick2-tick1) / cv::getTickFrequency();
        cout << endl << "PPF Elapsed Time " <<
            (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;
        
        // PostRefinement
        size_t results_size = results.size();
        if (results_size == 0)
        {
            cout << "No matching Poses found. Exiting." << endl;
            exit(0);
        }
        size_t N = 5;
        if (results_size < N) {
            cout << endl << "Reducing matching poses to be reported (as specified in code): "
                << N << " to the number of matches found: " << results_size << endl;
            N = results_size;
        }
        vector<Pose3DPtr> resultsSub(results.begin(), results.begin()+N);
        vector<double> resultsscores;

        // Create an instance of ICP
        ICP icp(100, 0.005f, 2.5f, 8);
        int64 t1 = cv::getTickCount();
        
        // Register for all selected poses
        cout << endl << "Performing ICP on " << N << " poses..." << endl;
        icp.registerModelToScene(models[id], scene, resultsSub);
        int64 t2 = cv::getTickCount();
        
        double icp_time = (double)(tick2-tick1) / cv::getTickFrequency();
        cout << endl << "ICP Elapsed Time " <<
            (t2-t1)/cv::getTickFrequency() << " sec" << endl;
        
        return *resultsSub[0]; // return transformation: model -> scene
        
        // TODO: Pose Validation
    }
}