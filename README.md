# Jerk Constraint Velocity Planning for an Autonomous Vehicle: Linear Programming Approach

## Paper
Our work is going to be presented at ICRA 2022. You can find the paper from [here](https://arxiv.org/abs/2202.10029). 

## Dependencies. 
This code has been tested on OSX, but the author is working to test the code on Ubuntu18.04/20.04. External dependencies are 
- Gurobi 10.0
- NLOPT
- Eigen3
- OSQP
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

## On Board Experiment Result
- Stopping Scenario
<img src="https://github.com/pflab-ut/jerk_optimal_velocity_planning/blob/master/media/stopping.png" width=70%>

- Following
<img src="https://github.com/pflab-ut/jerk_optimal_velocity_planning/blob/master/media/vehicle_following.png" width=70%>

- Cut-In
<img src="https://github.com/pflab-ut/jerk_optimal_velocity_planning/blob/master/media/cut_in.png" width=70%>

## Author
Yutaka Shimizu: yutaka.shimizu@pf.is.s.u-tokyo.ac.jp  
If you have any questions about the code or paper, feel free to contact the author either through Issues or email.

