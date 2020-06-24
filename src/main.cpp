#ifndef MAIN
#define MAIN

#include <string>
#include <fstream>
#include "pfc_initializer_constants.h"
#include "pfc_initializer.h"
#include "pose_helper.h"

using namespace std;

vector<string> runForPoseAndType(int pose_id, string img_type, bool cheat, bool print);
vector<vector<string>> runOnAllData(bool cheat, bool print);
void writeDataListToCSV(vector<vector<string>> dataList);

int main(int argc, char** argv)
{
    if(argc == 5)
    {
        string cheat_str = argv[3];
        string print_str = argv[4];
        
        bool print = true;
        bool cheat = true;

        if(cheat_str == "false")
            cheat = false;
        else if(cheat_str != "true")
        {
            cout << "use (run on one img type and pose): ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)>" << endl;
            cout << "use (run on all img types and poses): ./main <cheat(true/false)> <print(true/false)>" << endl;
            return 0; 
        }

        if(print_str == "false")
            print = false;
        else if(print_str != "true")
        {
            cout << "use (run on one img type and pose): ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)>" << endl;
            cout << "use (run on all img types and poses): ./main <cheat(true/false)> <print(true/false)>" << endl;
            return 0; 
        }

        vector<string> results = runForPoseAndType(stoi(argv[1]), argv[2], cheat, print);
    }
    else if (argc == 3)
    {
        string cheat_str = argv[1];
        string print_str = argv[2];
        
        bool print = true;
        bool cheat = true;

        if(cheat_str == "false")
            cheat = false;
        else if(cheat_str != "true")
        {
            cout << "use (run on one img type and pose): ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)>" << endl;
            cout << "use (run on all img types and poses): ./main <cheat(true/false)> <print(true/false)>" << endl;
            return 0; 
        }

        if(print_str == "false")
            print = false;
        else if(print_str != "true")
        {
            cout << "use (run on one img type and pose): ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)>" << endl;
            cout << "use (run on all img types and poses): ./main <cheat(true/false)> <print(true/false)>" << endl;
            return 0; 
        }

        vector<vector<string>> data_list = runOnAllData(cheat, print);
        writeDataListToCSV(data_list);
    }
    else
    {
        cout << "use (run on one img type and pose): ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)>" << endl;
        cout << "use (run on all img types and poses): ./main <cheat(true/false)> <print(true/false)>" << endl;
        return 0;
    }
}

vector<string> runForPoseAndType(int pose_id, string img_type, bool cheat, bool print)
{
    string left_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_l_c_" + img_type + ".png";
    string right_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_r_c_" + img_type + ".png";

    pfc::match_params params = {
        0, 
        360,
        1,
        98,
        200,
        1
    };

    if(cheat)
    {
        params = {
            pfc::min_rot.at(pose_id), 
            pfc::max_rot.at(pose_id),
            1,
            pfc::min_scl.at(pose_id),
            pfc::max_scl.at(pose_id),
            1
        };
    }


    PfcInitializer pfc(left_img_path, right_img_path, params);
    pfc.run(print);

    // Get results back as vector
    vector<string> results = pfc.getResultsAsVector();
    // Add pose score to results vector
    vector<double> score = scorePoseEstimation(pfc.pose, pose_id, print);
    results.push_back(to_string(score.at(0)));
    results.push_back(to_string(score.at(1)));

    return results;
}

vector<vector<string>> runOnAllData(bool cheat, bool print)
{
    vector<vector<string> > dataList;

    for(int pose_id = 0; pose_id < pfc::num_poses; pose_id++){
        for(int j = 0; j < pfc::num_img_types; j++){
            string img_type = pfc::img_types.at(j);
            cout << "Running for: " << pose_id << ", " << img_type << endl;
            vector<string> data = runForPoseAndType(pose_id, img_type, cheat, print);
            dataList.push_back(data);
        }
    }
    return dataList;
}

void writeDataListToCSV(vector<vector<string>> dataList)
{
    ofstream data_file;
    data_file.open("../result_data/pfcinit_performance_data.csv");
    
    if (data_file.fail()){
        cout << "couldn't open file" << endl;
    }

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
