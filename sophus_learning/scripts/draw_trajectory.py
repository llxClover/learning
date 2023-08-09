import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as Axes3D
import numpy as np
from pathlib import Path

groundtruth_file = Path(__file__).parent / "../data/groundtruth.txt"
estimated_file = Path(__file__).parent / "../data/estimated.txt"


def ReadTrajectory(gt_filename, esti_filename):
    gt_data = np.genfromtxt(
        gt_filename,
        dtype=[float, float, float, float, float, float, float, float])
    esti_data = np.genfromtxt(
        esti_filename,
        dtype=[float, float, float, float, float, float, float, float])
    return gt_data, esti_data


def DrawTrajectory(gt_data, esti_data):
    x_gt_arrary=[]
    y_gt_arrary=[]
    z_gt_arrary=[]
    for i in range(gt_data.size):
        x_gt_arrary.append(gt_data[i][1])
        y_gt_arrary.append(gt_data[i][2])
        z_gt_arrary.append(gt_data[i][3])
    
    x_esti_arrary=[]
    y_esti_arrary=[]
    z_esti_arrary=[]
    for i in range(esti_data.size):
        x_esti_arrary.append(esti_data[i][1])
        y_esti_arrary.append(esti_data[i][2])
        z_esti_arrary.append(esti_data[i][3])   
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x_gt_arrary,y_gt_arrary,z_gt_arrary,'r-')
    ax.plot(x_esti_arrary,y_esti_arrary,z_esti_arrary,'g-')
    plt.show()


gt_data, esti_data = ReadTrajectory(groundtruth_file, estimated_file)
DrawTrajectory(gt_data, esti_data)