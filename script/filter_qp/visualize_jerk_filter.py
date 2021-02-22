import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import matplotlib.patches as pat
from scipy import stats
import platform
import os

if __name__ == '__main__':
    os.chdir('../../')
    path = os.getcwd()
    ref_vel_data = pd.read_csv(path+"/result/filter_qp/reference_velocity.csv")
    obs_filtered_data = pd.read_csv(path+"/result/filter_qp/obs_filtered.csv")

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix' # math fontの設定
    plt.rcParams['xtick.direction'] = 'in' # x axis in
    plt.rcParams['ytick.direction'] = 'in' # y axis in
    plt.rcParams['axes.linewidth'] = 1.0 # axis line width
    plt.rcParams['axes.grid'] = True # make grid

    linewidth_ = 2.5
    fig = plt.figure(figsize=(14, 10))
    plt.plot(obs_filtered_data['position'], obs_filtered_data['original_velocity'], label="Maximum Velocity", color="black", linewidth=linewidth_)
    plt.plot(ref_vel_data["position"], ref_vel_data["original_velocity"],label="Obstacle Avoidance Velocity", color="purple", linewidth=linewidth_)
    plt.plot(ref_vel_data["position"], ref_vel_data["filtered_velocity"],label="Jerk Filtered Velocity", color="orange", linewidth=linewidth_)
    plt.xlabel("s [m]", fontsize=50)
    plt.ylabel("velocity [m/s]", fontsize=50)
    plt.tick_params(labelsize=50)
    plt.legend(fontsize=35)
    plt.savefig('/Users/yutaka/Desktop/experiment_filter.eps', bbox_inches="tight", pad_inches=0.05)
    plt.show()