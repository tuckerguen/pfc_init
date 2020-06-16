// Run batch of initializations on a variety of images
// aggregate and report time and performance
#include <iostream>
#include <unistd.h>
#include <stdlib.h>  
#include <dirent.h>
#include <stdio.h>

#include "pfc_init.hpp"

using namespace std;
// using namespace cv;

bool isLeftImage(string name);

int main(int argc, char** argv){
    //Use: ./batch_init <num_iterations> <0=same image, 1=variety of images>  <left_img_path> <right_img_path>
    int num_iterations = 10;
    bool run_same_image = true;
    string left_image_path = "../imgs/raw_l_a.png";
    string right_image_path = "../imgs/raw_r_a.png";

    if(argc == 5){
        num_iterations = atoi(argv[1]);
        if(num_iterations <= 0){
            cout << "num iterations must be greater than 0" << endl;
            return 0;
        }
        run_same_image = !strcmp(argv[2], "0") ? true : false;
        left_image_path = argv[3];
    }
    else if(argc != 1){
        cout << "Use: ./batch_init <num_iterations> <0=same image, 1=variety of images> <left_img_path> <right_img_path> OR ./batch_init" << endl;
        return 0;
    }

    double total_time = 0;

    if(run_same_image){
        for(int i = 0; i < num_iterations; ++i){
            double time = PFCInit(left_image_path, right_image_path, false);
            total_time += time;
            cout << "completed iteration " << i << " in: " << time << " seconds" << endl;
        }
    }
    else {
        struct dirent *file;
        DIR *dir;
        dir = opendir("../imgs/");
        if(dir != NULL){
            while((file = readdir(dir))){
                cout << file->d_name << endl;
                if(isLeftImage(file->d_name)){
                    string left_img = file->d_name;
                    int id_index = left_img.find("_l_");
                    string right_img = left_img;
                    right_img.replace(id_index, 3, "_r_");
                    cout << right_img << endl;
                }

            }
        }
    }

    double average_time = total_time / (double) num_iterations;
    cout << "avg time: " << average_time << endl;
}

bool isLeftImage(string name){
    if(name.size() > 2){
        for(int i = 0; i < name.size() - 2; i++){
            string c = name.substr(i, 3);
            if(c.compare("_l_"))
                return true;
        }
    }
    cout << name << " is not properly formatted" << endl;
    return false;
}
