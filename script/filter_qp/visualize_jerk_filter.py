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

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix' # math fontの設定
    plt.rcParams['xtick.direction'] = 'in' # x axis in
    plt.rcParams['ytick.direction'] = 'in' # y axis in
    plt.rcParams['axes.linewidth'] = 1.0 # axis line width

    fig = plt.figure(figsize=(14, 10))
    plt.plot(ref_vel_data["position"], ref_vel_data["original_velocity"],label="original velocity", color="black", linewidth=2.5)
    plt.plot(ref_vel_data["position"], ref_vel_data["filtered_velocity"],label="filtered velocity", linewidth=2.5)
    plt.xlabel("s [m]", fontsize=45)
    plt.ylabel("velocity [m/s]", fontsize=45)
    plt.tick_params(labelsize=45)
    plt.legend(fontsize=45)
    plt.savefig('/Users/yutaka/Desktop/experiment_jerk_filter.eps', bbox_inches="tight", pad_inches=0.05)
    plt.show()