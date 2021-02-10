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

    fig = plt.figure(figsize=(14, 10))
    #plt.ylim([-6, 6.0])
    plt.plot(pseudo_data['qp_position'], pseudo_data['qp_velocity'], label="qp_vel", color="b")
    plt.plot(pseudo_data['qp_position'], pseudo_data['qp_acceleration'], label="qp_acce", color="red")
    plt.plot(pseudo_data['qp_position'], pseudo_data['qp_jerk'], label="qp_jerk", color="purple")
    plt.plot(pseudo_data['qp_position'], pseudo_data['ref_velocity'], label="ref_vel", color="black")
    plt.xlabel("s [m]", fontsize=15)
    plt.ylabel("velocity [m/s]", fontsize=15)
    plt.tick_params(labelsize=18)
    plt.legend(fontsize=18)
    plt.show()