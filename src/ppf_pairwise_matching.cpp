#include <iostream>
#include <vector>
#include <opencv2/core/mat.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching.hpp>
#include <string>

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

static void help(const string& errorMessage)
{

}
int main(int argc, char** argv)
{
    // Macro Configuration
#if (defined __x86_64__ || defined _M_X64)
    std::cout << "Running on 64 bits" << endl;
#else
    std::cout << "Running on 32 bits" << endl;
#endif  
#ifdef _OPENMP
    std::cout << "Running with OpenMP" << endl;
#else
    std::cout << "Running without OpenMP and without TBB" << endl;
#endif

    // Input Configuration and Parameters
    if (argc < 3)
    {
        help("Not enough input arguments");
        exit(1);
    }
    const string input_dir  = (string)argv[1];
    const string output_dir = (string)argv[2];

    vector<cv::String> filenames;
    cv::glob(input_dir, filenames);

    // Pairwise Registration
    string target_name = filenames[0];
    Mat target = loadPLYSimple(target_name.c_str(), 1);
    vector<cv::Matx44d> global_pose;
    global_pose.push_back(cv::Matx44d::eye()); // pose of the base view
    Mat final = target; // final initialized as the base view

    for (int i = 1; i < filenames.size(); i++)
    {
        std::cout << "[Registration]: " << filenames[i] << " to " << filenames[i-1] << endl;
        Mat source = loadPLYSimple(filenames[i].c_str(), 1); // with normals
        // Model training
        std::cout << "Training... " << endl;
        int64 tick1 = cv::getTickCount();
        ppf_match_3d::PPF3DDetector detector(0.025, 0.05);
        detector.trainModel(target);
        int64 tick2 = cv::getTickCount();
        std::cout << endl << "Training complete in "
             << (double)(tick2-tick1)/ cv::getTickFrequency()
             << " sec" << endl;
        
        // Matching
        std::cout << "Matching... " << endl;
        vector<Pose3DPtr> results;        
        tick1 = cv::getTickCount();
        detector.match(source, results, 10.0/40.0, 0.05);
        tick2 = cv::getTickCount();
        std::cout << endl << "Matching complete in "
             << (double)(tick2-tick1)/ cv::getTickFrequency()
             << " sec" << endl;


        // ICP Refinement
        size_t N = 5;
        if (results.size() < N)
        {
            std::cout << endl << "Reducing matching poses to be reported (as specified in code): "
                << N << " to the number of matches found: " << results.size() << endl;
            N = results.size();
        }
        vector<Pose3DPtr> resultsSub(results.begin(), results.begin()+N);
        cout << "Processing ICP on " << N << " results" << endl;
        ICP icp(100, 0.005f, 2.5f, 8);
        tick1 = cv::getTickCount();
        icp.registerModelToScene(target, source, resultsSub);
        tick2 = cv::getTickCount();
        std::cout << endl << "ICP Refinement complete in "
             << (double)(tick2-tick1)/ cv::getTickFrequency()
             << " sec" << endl;
        
        // Pose Exporting
        vector<float> residuals;
        for (size_t j = 0; j < 5; j++)
        {
            residuals.push_back(resultsSub[j]->residual);
        }
        vector<float>::iterator min = min_element(begin(residuals), end(residuals));
        int min_index = (int)distance(begin(residuals), min);
        cout << "min residual index: " << min_index << endl;
        Pose3DPtr result = resultsSub[min_index];
        if (result->residual < 0.01){
            std::cout << "[Accepted]" << endl;
        }
        else{
            std::cout << "[Warning]: residual of " << i 
                        << "th cloud: " << result->residual << endl;
        }        
        cout << "Global Pose Computing" << endl;
        cv::Matx44d pose_i = global_pose[i-1] * result->pose.inv();
        global_pose.push_back(pose_i);
        cout << "Writing" << endl;
        string output_name = output_dir + "trans_00" + to_string(i+1) + ".ply";
        writePLY(transformPCPose(source, pose_i), output_name.c_str());

        // for (size_t j = 0; j < 5; j++)
        // {
        //     cout << i << endl;
        //     Pose3DPtr result = resultsSub[j];
        //     if (j == 0)
        //     {
        //         // cout << "results writing." << endl;
        //         if (result->residual < 0.01){
        //             std::cout << "[Accepted]" << endl;
        //         }
        //         else{
        //             std::cout << "[Warning]: residual of " << i 
        //                       << "th cloud: " << result->residual << endl;
        //         }
        //         // result->printPose();
        //         cout << "Global Pose Computing" << endl;
        //         cv::Matx44d pose_i = global_pose[i-1] * result->pose.inv();
        //         global_pose.push_back(pose_i);
        //         cout << "Writing" << endl;
        //         string output_name = output_dir + "trans_00" + to_string(i+1) + ".ply";
        //         writePLY(transformPCPose(source, pose_i), output_name.c_str());
        //     }
        // }
        target = source;
    }

    return 0;
}


