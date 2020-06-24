#ifndef MAIN
#define MAIN

#include "pfc_initializer.h"
#include <string>
#include "pfc_initializer_constants.h"
#include <fstream>

using namespace std;

vector<string> runForPoseAndType(int pose_id, string img_type);
vector<vector<string>> runOnAllData();
void writeDataListToCSV(vector<vector<string>> dataList);

int main(int argc, char** argv)
{
    vector<string> results = runForPoseAndType(stoi(argv[1]), argv[2]);
    for(int i = 0; i < results.size(); i++){
        cout << results.at(i) << ", ";
    }   
    cout << endl;
    // vector<vector<string>> data_list = {results};
    
    // vector<vector<string>> data_list = runOnAllData();
    // writeDataListToCSV(data_list);
}

vector<string> runForPoseAndType(int pose_id, string img_type)
{
    string left_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_l_c_" + img_type + ".png";
    string right_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_r_c_" + img_type + ".png";

    pfc::match_params params = {
        pfc::min_rot.at(pose_id), 
        pfc::max_rot.at(pose_id),
        1,
        pfc::min_scl.at(pose_id),
        pfc::max_scl.at(pose_id),
        1
    };
    // pfc::match_params params = {
    //     0, 
    //     360,
    //     1,
    //     98,
    //     200,
    //     1
    // };

    PfcInitializer pfc(left_img_path, right_img_path, pose_id, params);


    //TODO: Put this stuff in a run() function on PfcInitialzier
    //You should be able to just create the object and do pfc.run
    //Start timer
    double t = (double)cv::getTickCount();
    
    pfc.computeNeedlePose();

    //Stop timer
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Time: " << t << " s" << endl;

    pfc.scorePoseEstimation();
    pfc.displayResults();

    return pfc.getResultsVector();

}

vector<vector<string>> runOnAllData()
{
    vector<vector<string> > dataList;

    for(int pose_id = 0; pose_id < pfc::num_poses; pose_id++){
        for(int j = 0; j < pfc::num_img_types; j++){
            string img_type = pfc::img_types.at(j);
            cout << "Running for: " << pose_id << ", " << img_type << endl;
            vector<string> data = runForPoseAndType(pose_id, img_type);
            dataList.push_back(data);
        }
    }
    return dataList;
}

void writeDataListToCSV(vector<vector<string>> dataList)
{
    ofstream data_file;
    data_file.open("../pfcinit_performance_data.csv");

    for(int i = 0; i < dataList.size(); i++){
        for(int j = 0; j < dataList.at(i).size(); j++)
        {
            string data_point = dataList.at(i).at(j);
            if(j < dataList.at(i).size()-1)
                data_file << data_point << ", ";
            else 
                data_file << data_point;
        }
        data_file << "\n";
    }
}

#endif
