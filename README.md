# Jerk Constraint Velocity Planning for an Autonomous Vehicle: Linear Programming Approach

## Paper

## Dependencies. 
This code has been tested on OSX, but the author is working to test the code on Ubuntu18.04/20.04. External dependencies are 
- Gurobi 9.0
- NLOPT
- Eigen3
- CMake3.3 or higher

## Getting Started
#### 1. Build the Code
```
mkdir build
cd build
cmake ..
make 
```

#### 2. Running Code
- Running the main code
```
./filter_position_opt
```

- Running the visualization code
```asm
python3 visualize_experiments.py
```

- Running the st graph code
```asm
python3 visualize_st.py
```

## Result
- Scenario Normal
<img src="https://github.com/pflab-ut/jerk_optimal_velocity_planning/blob/master/media/scenario1.png" width=50%>

- Scenario Stop
<img src="https://github.com/pflab-ut/jerk_optimal_velocity_planning/blob/master/media/stop_scenario.png" width=50%>
