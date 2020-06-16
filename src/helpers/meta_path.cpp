#include <iostream>
#include <dirent.h>
#include <stdio.h>
#include <fstream>
#include <string.h>

using namespace std;

int main(){
    struct dirent *entry = nullptr;
    DIR *dp = nullptr;

    dp = opendir("/home/tucker/research/pfc_init/imgs/");
    if (dp != nullptr) {
        while ((entry = readdir(dp))){
            string fname = entry->d_name;
            if(fname != "."&& fname != ".."&& fname != "template.png"&& fname != "vessel"&& fname != "meta"){
                printf ("%s\n", entry->d_name);
                string meta_path = entry->d_name;
                meta_path.erase(meta_path.length() - 3, 3);
                meta_path = "../imgs/meta/" + meta_path + "meta";
                cout << meta_path << endl;

                ofstream file;
                file.open (meta_path);
                file.close();
            }
            
        }
            
    }

    closedir(dp);
    return 0;
}