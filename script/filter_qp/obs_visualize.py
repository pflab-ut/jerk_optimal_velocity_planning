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
    obs_filtered_data = pd.read_csv(path+"/result/filter_qp/obs_filtered.csv")

    fig = plt.figure(figsize=(14, 10))
    plt.ylim([-2.5, 5.5])
    plt.plot(obs_filtered_data['position'], obs_filtered_data['original_velocity'], label="original_vel", color="red")
    plt.plot(obs_filtered_data['position'], obs_filtered_data['filtered_velocity'], label="filtered_vel", color="blue")
    plt.xlabel("s [m]", fontsize=15)
    plt.ylabel("velocity [m/s]", fontsize=15)
    plt.tick_params(labelsize=18)
    plt.legend(fontsize=18)
    plt.show()