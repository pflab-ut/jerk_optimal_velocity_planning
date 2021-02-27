import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import matplotlib.patches as pat
from scipy import stats
import platform
import os

if __name__ == '__main__':
    os.chdir('../')
    path = os.getcwd()
    data = pd.read_csv(path+"/result/optimization_result.csv")

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix' # math fontの設定
    plt.rcParams['xtick.direction'] = 'in' # x axis in
    plt.rcParams['ytick.direction'] = 'in' # y axis in
    plt.rcParams['axes.linewidth'] = 1.0 # axis line width
    plt.rcParams['axes.grid'] = True # make grid

    linewidth_ = 8.5
    fontsize_ = 86
    legend_size_ = 65
    markersize_ = 20

    fig = plt.figure(figsize=(30, 60))
    ax1 = fig.add_subplot(311)
    ax2 = fig.add_subplot(312)
    ax3 = fig.add_subplot(313)

    ax1.plot(data['position'], data['lp_velocity'], label="LP", color="blue", linewidth=linewidth_)
    ax1.plot(data['position'], data['nc_velocity'], label="Non-Convex", color="red", linewidth=linewidth_,)
    ax1.plot(data['position'], data['qp_velocity'], label="Pseudo-QP", color="green", linewidth=linewidth_,)
    #ax1.plot(obs_filtered_data['position'], obs_filtered_data['original_velocity'], label="Maximum Velocity", color="black", linewidth=linewidth_)
    ax1.plot(data["position"], data["obs_filtered_velocity"], color="purple", label="Obstacle Avoidance Velocity", linewidth=linewidth_, linestyle="dashed")
    ax1.plot(data["position"], data["jerk_filtered_velocity"], color="orange", label="Jerk Filtered Velocity", linewidth=linewidth_, linestyle="dashed")
    ax1.set_xlabel("s [m]", fontsize=fontsize_)
    ax1.set_ylabel("velocity [m/s]", fontsize=fontsize_)
    ax1.tick_params(labelsize=fontsize_)
    #ax1.legend(fontsize=legend_size_)
    ax1.legend(loc="lower center", bbox_to_anchor=(.5, 1.0), ncol=2, fontsize=legend_size_)

    ax2.plot(data['position'], data['lp_acceleration'], color="blue",linewidth=linewidth_)
    ax2.plot(data['position'], data['nc_acceleration'], color="red", linewidth=linewidth_,)
    ax2.plot(data['position'], data['qp_acceleration'], color="green", linewidth=linewidth_,)
    ax2.set_xlabel("s [m]", fontsize=fontsize_)
    ax2.set_ylabel("acceleration [m/s^2]", fontsize=fontsize_)
    ax2.tick_params(labelsize=fontsize_)
    #ax2.legend(fontsize=legend_size_)

    ax3.plot(data['position'], data['lp_jerk'], color="blue", linewidth=linewidth_)
    ax3.plot(data['position'], data['nc_jerk'], color="red", linewidth=linewidth_,)
    ax3.plot(data['position'], data['qp_jerk'], color="green", linewidth=linewidth_,)
    ax3.plot([0, 30], [0.8, 0.8], color="black", linewidth=linewidth_, linestyle="dashed")
    ax3.plot([0, 30], [-0.8, -0.8], color="black", linewidth=linewidth_, linestyle="dashed")
    ax3.set_xlabel("s [m]", fontsize=fontsize_)
    ax3.set_ylabel("jerk [m/s^3]", fontsize=fontsize_)
    ax3.tick_params(labelsize=fontsize_)
    ax3.legend(fontsize=legend_size_)

    userhome = os.path.expanduser('~')
    desktop_path = userhome + '/Desktop/'
    plt.savefig(desktop_path + 'experiment_results.eps', bbox_inches="tight", pad_inches=0.05)
