// Input:
//     modelfile   - .ply
//     scenefile   - .ply
// Output:
//     pose_result - in
//     aggregated_pointcloud   - "para6700PCTrans.ply"
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching.hpp>
#include <string>
#include <algorithm>

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

static void help(const string& errorMessage)
{
    cout << "Program init error : "<< errorMessage << endl;
    cout << "\nUsage : ppf_matching [input model file] [input scene file] [output trans file]"<< endl;
    cout << "\nPlease start again with new parameters"<< endl;
}
double matching_score(Pose3DPtr result, Pose3DPtr ground_truth){
    Matx44d rpose = result->pose;
    Matx44d gtpose = result->pose;
    // translation;
    Vec3d r_trans(rpose(0,3), rpose(1,3), rpose(2,3));
    Vec3d gt_trans(gtpose(0,3), gtpose(1,3), gtpose(2,3));
    double trans = norm(r_trans - gt_trans);
    // rotation;
    double angle = result->angle - ground_truth->angle;
    
    return trans + angle/(2*M_PI);
    // if (trans < 0.5 && angle < angle_thresh){

    // }

}
double getdiameter(Mat model){}
int main(int argc, char** argv)
{
    // welcome message
    cout << "****************************************************" << endl;
    cout << "* Surface Matching demonstration : demonstrates the use of surface matching"
             " using point pair features." << endl;
    cout << "* The sample loads a model and a scene, where the model lies in a different"
             " pose than the training.\n* It then trains the model and searches for it in the"
             " input scene. The detected poses are further refined by ICP\n* and printed to the "
             " standard output." << endl;
    cout << "****************************************************" << endl;
    
    if (argc < 4)
    {
        help("Not enough input arguments");
        exit(1);
    }
    
#if (defined __x86_64__ || defined _M_X64)
    cout << "Running on 64 bits" << endl;
#else
    cout << "Running on 32 bits" << endl;
#endif
    
#ifdef _OPENMP
    cout << "Running with OpenMP" << endl;
#else
    cout << "Running without OpenMP and without TBB" << endl;
#endif
    
    string modelFileName = (string)argv[1];
    // string modelEdgeFile = (string)argv[5];
    string sceneFileName = (string)argv[2];
    bool detector_saveflag = atoi(argv[4]);
    Mat pc_edges, pcTest_edges;
    if (argc >= 6)
    {
        string modelEdgeFileName = (string)argv[5];
        pc_edges = loadPLYSimple(modelEdgeFileName.c_str(), 1);
    }
    if (argc == 7)
    {
        string sceneEdgeFileName = (string)argv[6];
        pcTest_edges = loadPLYSimple(sceneEdgeFileName.c_str(), 1);
    }
    // string groundtruthFileName = (string)argv[4];
    int64 debug1 = cv::getTickCount();
    Mat pc = loadPLYSimple(modelFileName.c_str(), 1);
    // Mat pc_edge = loadPLYSimple(modelEdgeFile.c_str(), 1);
    // double diameter = getdiameter(pc);
    int64 debug2 = cv::getTickCount();
    cout << endl << "loading Model complete in "
         << (double)(debug2-debug1)/ cv::getTickFrequency()
         << "sec" << endl;
    cout << endl << "model channel: " << pc.channels() << endl
         << "model dim:" << pc.cols << endl
         << "model vertices num: " << pc.rows << endl;
    // pc = ppf_match_3d::samplePCUniform(pc, 2);

    // Now train the model
    cout << "Training..." << endl;
    int64 tick1 = cv::getTickCount();
    ppf_match_3d::PPF3DDetector detector(0.025, 0.05);
    if (argc >= 6)
    {
        cout << "training with edges" << endl;
        detector.trainModel_S2B(pc, pc_edges);
    }
    else{
        cout << "training without edges" << endl;
        detector.trainModel(pc);
    }
    int64 tick2 = cv::getTickCount();
    double training_time = (double)(tick2-tick1) / cv::getTickFrequency();
    cout << endl << "Training complete in "
         << (double)(tick2-tick1)/ cv::getTickFrequency()
         << " sec" << endl << "Loading model..." << endl;
         
    // Read the scene
    Mat pcTest = loadPLYSimple(sceneFileName.c_str(), 1);
    // Matx44d ground_truth_mat = LoadData(groundtruthFileName, 4, 4);
    // Pose3DPtr ground_truth (new Pose3D);
    // ground_truth->updatePose(ground_truth_mat);
    // cout << ground_truth->pose << endl;
    // Match the model to the scene and get the pose

    // for single scene
    cout << endl << "Starting matching..." << endl;
    vector<Pose3DPtr> results;
    tick1 = cv::getTickCount();
    if (argc == 7)
    {
        cout << "matching with edges" << endl;
        detector.match_S2B(pcTest, pcTest_edges, results, 0.25, 0.05);
    }
    else{
        cout << "matching without edges" << endl;
        detector.match(pcTest, results, 0.25, 0.05);
    }
    
    tick2 = cv::getTickCount();
    double matching_time = (double)(tick2-tick1) / cv::getTickFrequency();
    cout << endl << "PPF Elapsed Time " <<
         (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;

    //check results size from match call above
    size_t results_size = results.size();
    cout << "Number of matching poses: " << results_size;
    if (results_size == 0) {
        cout << endl << "No matching poses found. Exiting." << endl;
        exit(0);
    }

    // Get only first N results - but adjust to results size if num of results are less than that specified by N
    size_t N = 10;
    if (results_size < N) {
        cout << endl << "Reducing matching poses to be reported (as specified in code): "
             << N << " to the number of matches found: " << results_size << endl;
        N = results_size;
    }
    vector<Pose3DPtr> resultsSub(results.begin(),results.begin()+N);
    vector<double> resultsscores;

    // Create an instance of ICP
    ICP icp(100, 0.005f, 2.5f, 8);
    int64 t1 = cv::getTickCount();
    
    // Register for all selected poses
    cout << endl << "Performing ICP on " << N << " poses..." << endl;
    icp.registerModelToScene(pc, pcTest, resultsSub);
    int64 t2 = cv::getTickCount();
    
    double icp_time = (double)(tick2-tick1) / cv::getTickFrequency();
    cout << endl << "ICP Elapsed Time " <<
         (t2-t1)/cv::getTickFrequency() << " sec" << endl;
         
    cout << "Poses: " << endl;

    // debug first five poses
    // for (size_t i=0; i<resultsSub.size(); i++){
    //     double score = matching_score(resultsSub[i], ground_truth);
    //     resultsscores.push_back(abs(score));
    //     cout << "result i: " << score << endl;
    // }
    // vector<double>::iterator min = min_element(begin(resultsscores), end(resultsscores));
    // int min_index = (int)distance(begin(resultsscores), min);
    // if (min_index != 0){
    //     cout << "ICP gives the false result" << endl;
    // }
    vector<float> residuals;
    for (size_t i=0; i<resultsSub.size(); i++)
    {
        Pose3DPtr result = resultsSub[i];
        cout << "Pose Result " << i << endl;
        result->printPose();
        // if (i == 0)
        // {
        //     Mat pct = transformPCPose(pcTest, result->pose.inv());
        //     writePLY(pct, argv[3]);
        // }
        residuals.push_back(result->residual);
    }
    vector<float>::iterator min = min_element(begin(residuals), end(residuals));
    int min_index = (int)distance(begin(residuals), min);
    Pose3DPtr result = resultsSub[min_index];
    result->printPose();
    Mat pct = transformPCPose(pcTest, result->pose.inv());
    writePLY(pct, argv[3]);
    
#ifdef _OPENMP
    cout << "Running with OpenMP" << endl;
#else
    cout << "Running without OpenMP and without TBB" << endl;
#endif
    cout << "training time: " << training_time << endl
         << "matching time: " << matching_time << endl
         << "icp time: " << icp_time << endl;

    if (detector_saveflag)
    {   
        const char* DETECTOR_NAME = "detector_bluemoon_bottle.xml";
        ifstream detectorFile(DETECTOR_NAME);
        cout << "Serializing..." << endl;
    
        tick1 = cv::getTickCount();
        FileStorage fsOut(DETECTOR_NAME, FileStorage::WRITE);
        detector.write(fsOut);
        fsOut.release();
        tick2 = cv::getTickCount();

        cout << "Serialization complete in "
            << (double)(tick2 - tick1) / cv::getTickFrequency()
            << " sec" << endl;
    }
    return 0;

    FileNode fn;
    fn.size();
    FileNode item = fn[1];
}
    
