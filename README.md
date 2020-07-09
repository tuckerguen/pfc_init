# Setup
1. Clone repo
2. From top level directory:
``` {bash}   
> mkdir build
> cd build
> cmake ../src
> make
```

# Run program
From build directory:
To run on a specific pose and image type:
```{bash}
> ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)> <multi_thread(true/false)>
```
pose_id = [0-9]  
img_type = ["fatty", "marked", "red", "tan"]  
cheat = true to use pre-defined rotation and scale range parameters to speed up execution time  
print = true to print and show all results  
multi_thread = true to use multiple threads to accelerate matching function

To run on all poses and image types:
```{bash}
> ./main <cheat(true/false)> <print(true/false)> <multi_thread(true/false)>
```
*Note: will write all results to the result_data/pfcinit_performance_data_{current date time}.csv file*  

Ex:
```
> ./main 2 marked true true true
> ./main true true false
```
1. Run on pose id 2, image type marked, with cheating, printing, and multithreading
2. Run on all poses with cheating, printing, and on a single thread