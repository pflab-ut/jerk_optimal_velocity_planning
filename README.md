# Jerk Constraint Velocity Planning for an Autonomous Vehicle: Linear Programming Approach

## Paper

## Dependencies. 
This code has been tested on OSX, but the author is working to test the code on Ubuntu18.04/20.04. External dependencies are 
- Gurobi 9.0
- NLOPT
- Eigen3
- CMake3.10 or higher

## Getting Started
#### 1. Build the Code
```
mkdir build
cd build
cmake ..
make 
```

#### 2. Running Code
- Proposed Methodology
```
./filter_position_opt
```

- Nonconvex Optimization
```
./nonconvex_opt
```

- Pseudo Jerk Optimization
```
./pseudo_opt
```

## Result
- Scenario1
![Scenrio1](https://github.com/pflab-ut/jerk_optimal_velocity_planning/blob/master/media/scenario1.png)
