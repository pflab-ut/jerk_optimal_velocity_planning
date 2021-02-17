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
    obs_filtered_data = pd.read_csv(path+"/result/filter_qp/obs_filtered.csv")

    fig = plt.figure(figsize=(14, 10))

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix' # math fontの設定
    plt.rcParams['xtick.direction'] = 'in' # x axis in
    plt.rcParams['ytick.direction'] = 'in' # y axis in
    plt.rcParams['axes.linewidth'] = 1.0 # axis line width

    plt.ylim([-0.1, 5.5])
    plt.plot(obs_filtered_data['position'], obs_filtered_data['original_velocity'], label="original_vel", color="red")
    plt.plot(obs_filtered_data['position'], obs_filtered_data['filtered_velocity'], label="filtered_vel", color="blue")
    plt.xlabel("s [m]", fontsize=25)
    plt.ylabel("velocity [m/s]", fontsize=25)
    plt.tick_params(labelsize=28)
    plt.legend(fontsize=28)
    plt.savefig('/Users/yutaka/Desktop/obstacle_avoidance.eps', bbox_inches="tight", pad_inches=0.05)
    plt.show()