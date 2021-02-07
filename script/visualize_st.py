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
    data = pd.read_csv(path+"/result/st_graph.csv")

    fig = plt.figure(figsize=(14, 10))
    plt.plot(data['obs_time'], data['obs_s'], label="obstacle", color="red")
    plt.plot(data['original_t'], data['position'], label="original", color="blue")
    plt.plot(data['filtered_t'], data['position'], label="filtered", color="purple")
    plt.xlabel("t [s]", fontsize=15)
    plt.ylabel("s [m]", fontsize=15)
    plt.tick_params(labelsize=18)
    plt.legend(fontsize=18)
    plt.show()