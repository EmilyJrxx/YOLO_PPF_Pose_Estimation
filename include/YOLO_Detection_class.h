// YOLODetection.h
// Description: this header file contains functions for YOLODetection,
// which are mainly: Detection, Outputing Boundingboxes
// Input:
//      rgb images,
//      depth images
// Output:
//      bounding boxes
//      (corresponding) confidences
//      (corresponding) classids
// # include <ros/ros.h>

# include <iostream>
# include <fstream>
# include <opencv4/opencv2/dnn.hpp>
# include <opencv4/opencv2/imgproc.hpp>
# include <opencv4/opencv2/highgui.hpp>

// # include <pcl/features/normal_3d_omp.h>
// # include <pcl/kdtree/kdtree_flann.h>
// # include <pcl/surface/convex_hull.h>
// # include <pcl/filters/crop_hull.h>

using namespace std;
using namespace cv;
using namespace dnn;

namespace yolo
{
    class YOLODetector
    {
        private:
            vector<Mat> rgb_imgs;
            vector<Mat> depth_imgs;
            vector<vector<float> > confidences;
            vector<vector<Rect> > boxes;
            vector<vector<int> > classIds;
            vector<vector<int> > indices;

            vector<string> class_labels;
            bool if_detected;
            bool if_postprocessed;
            Net net;

        public:
            YOLODetector(const string modelConfiguration, const string modelWeights){
                net = readNetFromDarknet(modelConfiguration, modelWeights);
                net.setPreferableBackend(DNN_BACKEND_OPENCV);
                net.setPreferableTarget(DNN_TARGET_CPU);
                cout << "YOLO network configured. " << endl; 
                boxes.resize(1);
                classIds.resize(1);
                indices.resize(1); 
                confidences.resize(1);
                if_detected = false;
                if_postprocessed = false;
            }
            YOLODetector(const string modelConfiguration, const string modelWeights, 
                vector<Mat> rgb_input, vector<Mat> depth_input){
                net = readNetFromDarknet(modelConfiguration, modelWeights);
                net.setPreferableBackend(DNN_BACKEND_OPENCV);
                net.setPreferableTarget(DNN_TARGET_CPU);
                cout << "YOLO network configured. " << endl; 
                rgb_imgs = rgb_input;
                depth_imgs = depth_input;
                boxes.clear(); 
                classIds.clear(); 
                indices.clear(); 
                confidences.clear();
                boxes.resize(rgb_input.size());
                classIds.resize(rgb_input.size());
                indices.resize(rgb_input.size()); 
                confidences.resize(rgb_input.size());
                if_detected = false;
                if_postprocessed = false;
                

                cout << "RGB and depth images inputed" << endl;
            }
            void reloadImages(vector<Mat> rgb_input, vector<Mat> depth_input){
                TODO: //每次重新加载图片时需要清除上一次的检测结果。
                rgb_imgs = rgb_input;
                depth_imgs = depth_input;
                boxes.clear(); 
                classIds.clear(); 
                indices.clear(); 
                confidences.clear();
                boxes.resize(rgb_input.size());
                classIds.resize(rgb_input.size());
                indices.resize(rgb_input.size()); 
                confidences.resize(rgb_input.size());

                if_detected = false;
                if_postprocessed = false; // 加了两个flag要注意同步的问题.
                cout << "RGB and depth images reloaded" << endl;
                // cout << "After reloading " << classIds.size() << " " << classIds[0].size() << endl;                                
            }
            ~YOLODetector(){
                net.~Net();
            }
            void LoadClassNames(const string classesFile);
            vector<string> getOutPutNames(const Net& net);
            int detect(vector<Mat> *outs, const int inpWidth, const int inpHeight);
            int postprocess(const vector<Mat> *outs, const double confThreshold,
                const double nmsThreshold); // draw bounding box and display, and get final output
            int display();
            void drawPred(const vector<string> classes_label, int classId, float conf, int left, int top, int right,
                 int bottom, Mat& frame);
            void TransferResults(int index, vector<Rect>& boxes_out, vector<int>& classIds_out, vector<int>& indices_out);
    };
    void YOLODetector::LoadClassNames(const string classesFile)
    {
        ifstream ifs(classesFile.c_str());
        string line;
        while (getline(ifs, line)) class_labels.push_back(line);
    }
    vector<string> YOLODetector::getOutPutNames(const Net& net)
    {
        static vector<String> names;
        if (names.empty())
        {
            //Get the indices of the output layers, i.e. the layers with unconnected outputs
            vector<int> outLayers = net.getUnconnectedOutLayers();
            
            //get the names of all the layers in the network
            vector<String> layersNames = net.getLayerNames();
            
            // Get the names of the output layers in names
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
        }
        return names;                
    }
    int YOLODetector::detect(vector<Mat> *outs, const int inpWidth, const int inpHeight){
        // Input: RGB & depth images
        // Output: outs
        // Pre-check
        if (rgb_imgs.size() != depth_imgs.size())
        {
            printf("number of RGB images doesn't equal number of Depth images");
            return 1;
        }
        // Detection
        uint32_t size = rgb_imgs.size();
        for (int k = 0; k < size; k++)
        {
            Mat rgb   = rgb_imgs[k];
            Mat depth = depth_imgs[k];
            Mat blob, frame;
            blobFromImage(rgb, blob, 1/255.0, Size(inpWidth, inpHeight),
                Scalar(0, 0, 0), true, false);
            
            // Core Procedure
            net.setInput(blob);
            net.forward(outs[k], getOutPutNames(net));
            
            // Efficiency Estimation
            vector<double> layerTime;
            double freq = getTickFrequency() / 1000; // seconds
            double t = net.getPerfProfile(layerTime) / freq;
            string timelabel = format("Inference time for a frame: %.2f ms", t);
            putText(rgb, timelabel, Point(0, 15), FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 0, 255));
        } 

        // cout << "After Detecting " << classIds.size() << " " << classIds[0].size() << endl;    
        if_detected = true;
        return 0;
    }
    int YOLODetector::postprocess(const vector<Mat> *outs, const double confThreshold, const double nmsThreshold){
        // Input: outs (frames outputed from net model)
        // Output: classIds, confidences, boundingboxes
        if (if_detected == false){
            printf("PostProcessError: Not detected yet");
            return 2;
        }
        if (outs[0].size() == 0)
        {
            printf("PostProcessError: None object detected");
            return 1;
        }
        cout << "Postprocess: pre-check." << endl; // debug
        for (uint32_t k = 0; k < rgb_imgs.size(); k++)
        {
            Mat frame = rgb_imgs[k];
            cout << "Processing on " << k << endl;
            for (size_t i = 0; i < outs[k].size(); i++)
            {
                // Scan through all the bounding boxes output from the network and 
                // keep only the ones with high confidence scores. Assign the box's
                // class label as the class with the highest score for the box. 
                vector<Mat> out = outs[k];
                float* data = (float*)out[i].data;
                cout << "Processing on " << i << "th output. " << endl;
                cout << "out size: " << out[i].rows << " " << out[i].cols << endl;
                for (int j = 0; j < out[i].rows; ++j, data+= out[i].cols) 
                {
                    Mat scores = out[i].row(j).colRange(5, out[i].cols);
                    Point classIdPoint;
                    double confidence;                
                    // Get the value and location of the maximum score
                    minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                    // cout << "minMaxLoc done." << endl;         
                    if (confidence > confThreshold)
                    {
                        cout << "Higher than confThresh found! " << j << endl; 
                        int centerX = (int)(data[0] * frame.cols);
                        int centerY = (int)(data[1] * frame.rows);
                        int width = (int)(data[2] * frame.cols);
                        int height = (int)(data[3] * frame.rows);
                        int left = centerX - width / 2;
                        int top = centerY - height / 2;
                        cout << "Size done." << endl;
                        cout << classIds[k].size() << endl;
                        cout << "Loc: " << classIdPoint.x << classIdPoint.y << endl;
                        classIds[k].push_back(classIdPoint.x);
                        cout << "CLassId done." << endl;
                        confidences[k].push_back((float)confidence);
                        cout << "Confi done." << endl;
                        boxes[k].push_back(Rect(left, top, width, height));
                        cout << "box done. " << endl;
                    }            
                }      
            }
            cout << (k+1) << " results size: " << classIds[k].size() << endl;
            cout << "Processing NMS" << endl;
            dnn::NMSBoxes(boxes[k], confidences[k], confThreshold, nmsThreshold, indices[k]);            
        }

        if_postprocessed = 1;
        return 0;
    }
    int YOLODetector::display(){
        // Pre-Check
        if (if_postprocessed == false){
            printf("DisplayError: Not postprocessed yet");
            return 2;
        }
        for (uint32_t k = 0; k < rgb_imgs.size(); k++)
        {
            vector<int> indi = indices[k];
            vector<Rect> bo = boxes[k];
            Mat frame = rgb_imgs[k];
            // Mat frame = depth_imgs[k];
            for (size_t i = 0; i < indi.size(); i++){
                int idx = indi[i];
                Rect box = bo[idx];
                drawPred(class_labels, classIds[k][idx], confidences[k][idx], box.x, box.y,
                    box.x + box.width, box.y + box.height, frame);
                cout << "Bounding box drawn. " << endl;
            }
            imshow("Detected Object", frame);            
            resizeWindow("Detected Object", 320, 240);
        } 
        waitKey(0);
    }
    void YOLODetector::drawPred(const vector<string> classes_label, int classId, float conf, int left,
         int top, int right, int bottom, Mat& frame)
    {
        rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

        std::string label = format("%.2f", conf);
        if (!classes_label.empty()) {
            CV_Assert(classId < (int)classes_label.size());
            label = classes_label[classId] + ": " + label;
        }

        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        top = max(top, labelSize.height);
        rectangle(frame, Point(left, top - labelSize.height),
            Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
        putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
        cout << "Boundingbox: " << label << endl;
    }   
    void YOLODetector::TransferResults(int index, vector<Rect>& boxes_out, vector<int>& classIds_out, vector<int>& indices_out)
    {
        boxes_out = boxes[index];
        classIds_out = classIds[index];
        indices_out =  indices[index];
    }
}