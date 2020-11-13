#include <iostream>
#include <string>
#include <queue>
#include <iterator>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"

using namespace std;
using namespace cv;
using namespace sen;

constexpr float CONFIDENCE_THRESHOLD = 0;
constexpr float NMS_THRESHOLD = 0.4;
constexpr int NUM_CLASSES = 80;

// colors for bounding boxes
const cv::Scalar colors[] = {
    {0, 255, 255},
    {255, 255, 0},
    {0, 255, 0},
    {255, 0, 0}
};
const auto NUM_COLORS = sizeof(colors)/sizeof(colors[0]);

const string yolo_cfg_file = "/home/xxwang/Packages/darknet-master/cfg/yolov4.cfg";
const string yolo_weights_file = "/home/xxwang/Packages/darknet-master/yolov4.weights";
const string classFile = "/home/xxwang/Packages/darknet-master/cfg/coco.names";

int main(int argc, char **argv)
{
    // YOLO configuration
    std::vector<std::string> class_names;
    {
        std::ifstream class_file(classFile);
        if (!class_file)
        {
            std::cerr << "failed to open classes.txt\n";
            return 0;
        }

        std::string line;
        while (std::getline(class_file, line))
            class_names.push_back(line);
    }
    auto net = cv::dnn::readNetFromDarknet(yolo_cfg_file, yolo_weights_file);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    // net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    // net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    auto output_names = net.getUnconnectedOutLayersNames();

    // K4a configuration
    const uint32_t deviceNum = k4a::device::get_installed_count();
    if (deviceNum == 0){
        cerr << "No Azure Kinect Device Detected. " << endl;
        exit(1);
    }
    /// Configuration
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL; // init
    config.camera_fps   = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode   = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.synchronized_images_only = true; // only synchronized images allowed

    /// Starting Device
    cout << "Opening K4A device ... " << endl;
    k4a::device dev = k4a::device::open(K4A_DEVICE_DEFAULT);
    dev.start_cameras(& config);
    cout << "K4A device Started." << endl;

    cv::Mat frame, blob;
    std::vector<cv::Mat> detections;
    k4a::capture capture;
    while(cv::waitKey(1) < 1)
    {
        if (dev.get_capture(&capture, std::chrono::milliseconds(0)))
        {
            k4a::image rgb_img = capture.get_color_image();
            uint8_t *color_buffer = rgb_img.get_buffer();
            uint16_t color_h = rgb_img.get_height_pixels();
            uint16_t color_w = rgb_img.get_width_pixels();
            frame = Mat(color_h, color_w, CV_8UC4, color_buffer);
            // source >> frame;
            if (frame.empty())
            {
                cv::waitKey();
                break;
            }
            cv::cvtColor(frame, frame, COLOR_BGRA2BGR);
            auto total_start = std::chrono::steady_clock::now();
            cv::dnn::blobFromImage(frame, blob, 0.00392, cv::Size(608, 608), cv::Scalar(), true, false, CV_32F);
            net.setInput(blob);

            auto dnn_start = std::chrono::steady_clock::now();
            net.forward(detections, output_names);
            auto dnn_end = std::chrono::steady_clock::now();

            std::vector<int> indices[NUM_CLASSES];
            std::vector<cv::Rect> boxes[NUM_CLASSES];
            std::vector<float> scores[NUM_CLASSES];

            for (auto& output : detections)
            {
                const auto num_boxes = output.rows;
                for (int i = 0; i < num_boxes; i++)
                {
                    auto x = output.at<float>(i, 0) * frame.cols;
                    auto y = output.at<float>(i, 1) * frame.rows;
                    auto width = output.at<float>(i, 2) * frame.cols;
                    auto height = output.at<float>(i, 3) * frame.rows;
                    cv::Rect rect(x - width/2, y - height/2, width, height);

                    for (int c = 0; c < NUM_CLASSES; c++)
                    {
                        auto confidence = *output.ptr<float>(i, 5 + c);
                        if (confidence >= CONFIDENCE_THRESHOLD)
                        {
                            boxes[c].push_back(rect);
                            scores[c].push_back(confidence);
                        }
                    }
                }
            }

            for (int c = 0; c < NUM_CLASSES; c++)
                cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);
            
            for (int c= 0; c < NUM_CLASSES; c++)
            {
                for (size_t i = 0; i < indices[c].size(); ++i)
                {
                    const auto color = colors[c % NUM_COLORS];

                    auto idx = indices[c][i];
                    const auto& rect = boxes[c][idx];
                    cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color, 3);

                    std::ostringstream label_ss;
                    label_ss << class_names[c] << ": " << std::fixed << std::setprecision(2) << scores[c][idx];
                    auto label = label_ss.str();
                    
                    int baseline;
                    auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                    cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
                    cv::putText(frame, label.c_str(), cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
                }
            }
        
            auto total_end = std::chrono::steady_clock::now();

            float inference_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(dnn_end - dnn_start).count();
            float total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
            std::ostringstream stats_ss;
            stats_ss << std::fixed << std::setprecision(2);
            stats_ss << "Inference FPS: " << inference_fps << ", Total FPS: " << total_fps;
            auto stats = stats_ss.str();
                
            int baseline;
            auto stats_bg_sz = cv::getTextSize(stats.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
            cv::rectangle(frame, cv::Point(0, 0), cv::Point(stats_bg_sz.width, stats_bg_sz.height + 10), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(frame, stats.c_str(), cv::Point(0, stats_bg_sz.height + 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));

            cv::namedWindow("output");
            cv::imshow("output", frame);    
        }
        if (waitKey(1) == 'q')
        {
            dev.close();
            break;
        }
    }

    return 0;
}