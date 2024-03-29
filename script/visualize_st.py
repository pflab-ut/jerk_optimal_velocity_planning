import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import matplotlib.patches as pat
from scipy import stats
import platform
import os
import matplotlib

if __name__ == '__main__':
    os.chdir('../')
    path = os.getcwd()
    obs_data = pd.read_csv(path+"/result/obs.csv")
    opt_data = pd.read_csv(path+"/result/optimization_result.csv")

    linewidth_ = 5.5

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix' # math fontの設定
    plt.rcParams['xtick.direction'] = 'in' # x axis in
    plt.rcParams['ytick.direction'] = 'in' # y axis in
    plt.rcParams['axes.linewidth'] = 1.0 # axis line width
    plt.rcParams['axes.grid'] = True # make grid
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42

    fig = plt.figure(figsize=(20, 10))
    ax1 = fig.add_subplot(111)

    ax1.plot(obs_data['obs_time'], obs_data['obs_s'], label="Obstacle", color="black", linewidth=linewidth_, linestyle="dashed")
    ax1.plot(opt_data['max_time'], opt_data['max_position'], label="Original", color="black", linewidth=linewidth_)
    ax1.plot(opt_data['obs_filtered_time'], opt_data['obs_position'], label="Avoidance", color="purple", linewidth=linewidth_)
    #ax1.plot(opt_data['jerk_filtered_time'], opt_data['position'], label="Jerk Filter", color="orange", linewidth=linewidth_)
    ax1.plot(opt_data['lp_time'], opt_data['lp_position'], label="LP", color="blue", linewidth=linewidth_)
    ax1.set_xlabel("t [s]", fontsize=50)
    ax1.set_ylabel("s [m]", fontsize=50)
    ax1.tick_params(labelsize=40)
    ax1.legend(fontsize=40)

    userhome = os.path.expanduser('~')
    desktop_path = userhome + '/Desktop/'
    plt.savefig(desktop_path + 'experiment_obstacle_avoidance.eps', bbox_inches="tight", pad_inches=0.05)