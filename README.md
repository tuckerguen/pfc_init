# Setup
1. Clone repo
2. From top level directory:
``` {bash}   
> mkdir build
> cd build
> cmake ../src
```

# Run program
From build directory:
To run on a specific pose and image type:
```{bash}
> ./main <pose_id> <img_type> <cheat(true/false)> <print(true/false)>
```
pose_id = [0-9]
img_type = ["fatty", "marked", "red", "tan"]
cheat = true to use pre-defined rotation and scale range parameters to speed up execution time
print = true to print and show all results

To run on all poses and image types:
```{bash}
> ./main <cheat(true/false)> <print(true/false)>
```
*Note: will write all results to the result_data/pfcinit_performance_data.csv file*

Ex:
```
> ./main 2 marked true true
> ./main true true
```