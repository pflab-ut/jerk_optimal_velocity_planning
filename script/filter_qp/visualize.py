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
    qp_data = pd.read_csv(path+"/result/filter_qp/qp_result.csv")

    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix' # math fontの設定
    plt.rcParams['xtick.direction'] = 'in' # x axis in
    plt.rcParams['ytick.direction'] = 'in' # y axis in
    plt.rcParams['axes.linewidth'] = 1.0 # axis line width

    fig = plt.figure(figsize=(28, 20))
    plt.plot(qp_data['qp_position'], qp_data['qp_velocity'], label="optimized velocity", color="b", linewidth=2.5)
    plt.plot(qp_data['qp_position'], qp_data['qp_acceleration'], label="optimized acceleration", color="red",linewidth=2.5)
    plt.plot(qp_data['qp_position'], qp_data['qp_jerk'], label="optimized jerk", color="purple", linewidth=2.5)
    plt.plot(ref_vel_data["position"], ref_vel_data["original_velocity"],label="maximum velocity", color="black", linewidth=2.5)
    plt.plot(ref_vel_data["position"], ref_vel_data["filtered_velocity"],label="filtered velocity", linewidth=2.5)
    #plt.plot(ref_vel_data["position"], ref_vel_data["filtered_acc"],label="", linewidth=2.5)
    plt.xlabel("s [m]", fontsize=76)
    plt.ylabel("velocity [m/s]", fontsize=76)
    plt.tick_params(labelsize=76)
    plt.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=2, fontsize=62)
    plt.savefig('/Users/yutaka/Desktop/experiment_gurobi_lp.eps', bbox_inches="tight", pad_inches=0.05)
    #plt.show()
