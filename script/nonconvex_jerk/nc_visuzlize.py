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
    ref_vel_data = pd.read_csv(path+"/result/nonconvex_jerk/reference_velocity.csv")
    nc_data = pd.read_csv(path+"/result/nonconvex_jerk/nc_result.csv")

    fig = plt.figure(figsize=(14, 10))
    plt.ylim([-2.5, 3.5])
    plt.plot(nc_data['qp_position'], nc_data['qp_velocity'], label="nc_vel", color="b")
    plt.plot(nc_data['qp_position'], nc_data['qp_acceleration'], label="nc_acc", color="red")
    plt.plot(nc_data['qp_position'], nc_data['qp_jerk'], label="nc_jerk", color="purple")
    plt.plot(ref_vel_data["position"], ref_vel_data["original_velocity"],label="original_vel", color="black")
    plt.plot(ref_vel_data["position"], ref_vel_data["filtered_velocity"],label="filtered_vel")
    plt.plot(ref_vel_data["position"], ref_vel_data["filtered_acc"],label="filtered_acc")
    plt.xlabel("s [m]", fontsize=15)
    plt.ylabel("velocity [m/s]", fontsize=15)
    plt.tick_params(labelsize=18)
    plt.legend(fontsize=18)
    plt.show()