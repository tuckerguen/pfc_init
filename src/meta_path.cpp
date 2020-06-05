#include <iostream>
#include <unistd.h>
#include <fstream>
#include <sstream>

using namespace std;


int main(){
    string meta_path = "../imgs/raw_l_a.png";
    meta_path.erase(0, 7);
    meta_path.erase(meta_path.length() - 3, 3);
    meta_path = "../imgs/meta" + meta_path + "meta";
    cout << "meta-path: " << meta_path << endl;

    string data;
    ifstream meta_file;
    meta_file.open(meta_path.c_str());
    getline(meta_file, data);
    cout << "meta-data: " << data << endl;
    
    istringstream ss(data);
    int tx, ty, tw, th;
    ss >> tx >> ty >> tw >> th;
    cout << tx << ", "<< ty << ", "<< tw << ", "<< th << endl; 
    return 0;
}