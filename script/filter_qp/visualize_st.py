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
    data = pd.read_csv(path+"/result/filter_qp/st_graph.csv")
    obs_filtered_data = pd.read_csv(path+"/result/filter_qp/obs_filtered.csv")

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix' # math fontの設定
    plt.rcParams['xtick.direction'] = 'in' # x axis in
    plt.rcParams['ytick.direction'] = 'in' # y axis in
    plt.rcParams['axes.linewidth'] = 1.0 # axis line width

    fig = plt.figure(figsize=(24, 8))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)

    ax1.plot(data['obs_time'], data['obs_s'], label="obstacle", color="red")
    ax1.plot(data['original_t'], data['position'], label="original", color="blue")
    ax1.plot(data['filtered_t'], data['position'], label="filtered", color="green")
    ax1.set_xlabel("t [s]", fontsize=35)
    ax1.set_ylabel("s [m]", fontsize=35)
    ax1.tick_params(labelsize=28)
    ax1.legend(fontsize=35)

    ax2.plot(obs_filtered_data['position'], obs_filtered_data['original_velocity'], label="original velocity", color="blue")
    ax2.plot(obs_filtered_data['position'], obs_filtered_data['filtered_velocity'], label="filtered velocity", color="green")
    ax2.set_xlabel("s [m]", fontsize=35)
    ax2.set_ylabel("velocity [m/s]", fontsize=35)
    ax2.tick_params(labelsize=28)
    ax2.legend(fontsize=38)

    plt.savefig('/Users/yutaka/Desktop/experiment_obstacle_avoidance.eps', bbox_inches="tight", pad_inches=0.05)
    plt.show()