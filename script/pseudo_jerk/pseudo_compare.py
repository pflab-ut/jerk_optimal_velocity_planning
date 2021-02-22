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
    pseudo_data = pd.read_csv(path+"/result/pseudo_jerk/qp_result.csv")

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix' # math fontの設定
    plt.rcParams['xtick.direction'] = 'in' # x axis in
    plt.rcParams['ytick.direction'] = 'in' # y axis in
    plt.rcParams['axes.linewidth'] = 1.0 # axis line width

    fig = plt.figure(figsize=(14, 10))
    plt.plot(pseudo_data['qp_position'], pseudo_data['qp_velocity'], label="optimized velocity", color="b")
    plt.plot(pseudo_data['qp_position'], pseudo_data['qp_acceleration'], label="optimized acceleration", color="red")
    plt.plot(pseudo_data['qp_position'], pseudo_data['qp_jerk'], label="optimized pseudo jerk", color="purple")
    plt.plot(pseudo_data['qp_position'], pseudo_data['ref_velocity'], label="maximum velocity", color="black")
    plt.xlabel("s [m]", fontsize=45)
    plt.ylabel("velocity [m/s]", fontsize=45)
    plt.tick_params(labelsize=38)
    plt.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, fontsize=30)
    plt.savefig('/Users/yutaka/Desktop/experiment_gurobi_pseudo.eps', bbox_inches="tight", pad_inches=0.05)
    #plt.show()