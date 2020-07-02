#include <string>
#include <fstream>
#include "pfc_initializer_constants.h"
#include "pfc_initializer.h"
#include "pose_helper.h"

using namespace std;

vector<string> runForPoseAndType(int pose_id, string img_type, bool cheat, bool print, bool thread);
vector<vector<string>> runOnAllData(bool cheat, bool print, bool thread);
void writeDataListToCSV(vector<vector<string>> dataList);
bool processArgBool(string str);

string use_msg_one = "use (run on one img type and pose): ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)> <thread(t/f)>" ;
string use_msg_all = "use (run on all img types and poses): ./main <cheat(true/false)> <print(true/false)> <thread(t/f)>";

bool processArgBool(string str)
{
    if(str == "false")
        return false;
    else if(str != "true")
    {
        cout << use_msg_one << endl;
        cout <<  use_msg_all<< endl;
    }
    return true;
}

int main(int argc, char** argv)
{ 
    if(argc == 6)
    {
        string cheat_str = argv[3];
        string print_str = argv[4];
        string thread_str = argv[5];
        
        bool print = processArgBool(print_str);
        bool cheat = processArgBool(cheat_str);
        bool thread = processArgBool(thread_str);

        vector<string> results = runForPoseAndType(stoi(argv[1]), argv[2], cheat, print, thread);
    }
    else if (argc == 4)
    {
        string cheat_str = argv[1];
        string print_str = argv[2];
        string thread_str = argv[3];
        
        bool print = processArgBool(print_str);
        bool cheat = processArgBool(cheat_str);
        bool thread = processArgBool(thread_str);

        vector<vector<string>> data_list = runOnAllData(cheat, print, thread);
        writeDataListToCSV(data_list);
    }
    else
    {
        cout << use_msg_one << endl;
        cout << use_msg_all << endl;
        return 0;
    }
}

vector<string> runForPoseAndType(int pose_id, string img_type, bool cheat, bool print, bool thread)
{
    string left_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_l_c_" + img_type + ".png";
    string right_img_path = "../imgs/raw/" + std::to_string(pose_id) + "_r_c_" + img_type + ".png";

    pfc::match_params params = {
        0, 
        360,
        1,
        98,
        120,
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
    pfc.run(print, thread);

    // Get results back as vector
    vector<string> results = pfc.getResultsAsVector();
    // Add pose score to results vector
    vector<double> score = scorePoseEstimation(pfc.pose, pose_id, print);
    results.push_back(to_string(score.at(0)));
    results.push_back(to_string(score.at(1)));

    return results;
}

vector<vector<string>> runOnAllData(bool cheat, bool print, bool thread)
{
    vector<vector<string> > dataList;

    for(int pose_id = 0; pose_id < pfc::num_poses; pose_id++){
        for(int j = 0; j < pfc::num_img_types; j++){
            string img_type = pfc::img_types.at(j);
            cout << "Running for: " << pose_id << ", " << img_type << endl;
            vector<string> data = runForPoseAndType(pose_id, img_type, cheat, print, thread);
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