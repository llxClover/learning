import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as Axes3D
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

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


def DrawCoordinateArrow(ax, x, y, z, qx=0, qy=0, qz=0, qw=1, length_arrow=0.15):
    q = [qx, qy, qz, qw]
    r = R.from_quat(q)
    Rm = r.as_dcm()

    p_o = [x, y, z]

    p_x = [length_arrow, 0, 0]
    p_y = [0, length_arrow, 0]
    p_z = [0, 0, length_arrow]

    p_x = [Rm[0][0]*p_x[0]+Rm[0][1]*p_x[1]+Rm[0][2]*p_x[2] + p_o[0],
           Rm[1][0]*p_x[0]+Rm[1][1]*p_x[1]+Rm[1][2]*p_x[2] + p_o[1],
           Rm[2][0]*p_x[0]+Rm[2][1]*p_x[1]+Rm[2][2]*p_x[2] + p_o[2]]
    ax.plot([p_o[0], p_x[0]], [p_o[1], p_x[1]], [p_o[2], p_x[2]], color='red')
    # print("|X| = ", ((p_o[0]-p_x[0]) ** 2 +
    #       (p_o[1]-p_x[1]) ** 2 + (p_o[2]-p_x[2]) ** 2)**0.5)
    p_y = [Rm[0][0]*p_y[0]+Rm[0][1]*p_y[1]+Rm[0][2]*p_y[2] + p_o[0],
           Rm[1][0]*p_y[0]+Rm[1][1]*p_y[1]+Rm[1][2]*p_y[2] + p_o[1],
           Rm[2][0]*p_y[0]+Rm[2][1]*p_y[1]+Rm[2][2]*p_y[2] + p_o[2]]
    ax.plot([p_o[0], p_y[0]], [p_o[1], p_y[1]],
            [p_o[2], p_y[2]], color='green')
    # print("|Y| = ", ((p_o[0]-p_y[0]) ** 2 +
    #       (p_o[1]-p_y[1]) ** 2 + (p_o[2]-p_y[2]) ** 2)**0.5)
    p_z = [Rm[0][0]*p_z[0]+Rm[0][1]*p_z[1]+Rm[0][2]*p_z[2] + p_o[0],
           Rm[1][0]*p_z[0]+Rm[1][1]*p_z[1]+Rm[1][2]*p_z[2] + p_o[1],
           Rm[2][0]*p_z[0]+Rm[2][1]*p_z[1]+Rm[2][2]*p_z[2] + p_o[2]]
    ax.plot([p_o[0], p_z[0]], [p_o[1], p_z[1]], [p_o[2], p_z[2]], color='blue')
    # print("|Z| = ", ((p_o[0]-p_z[0]) ** 2 +
    #       (p_o[1]-p_z[1]) ** 2 + (p_o[2]-p_z[2]) ** 2)**0.5)
    # if -1e-10 <= (p_x[0]-p_o[0])*(p_y[0]-p_o[0])+(p_x[1]-p_o[1])*(p_y[1]-p_o[1])+(p_x[2]-p_o[2])*(p_y[2]-p_o[2]) <= 1e-10:
    #     print("X Y 正交")
    # else:
    #     print("X Y 不正交 = ", (p_x[0]-p_o[0])*(p_y[0]-p_o[0])+(p_x[1] -
    #           p_o[1])*(p_y[1]-p_o[1])+(p_x[2]-p_o[2])*(p_y[2]-p_o[2]))
    # if -1e-10 <= (p_z[0]-p_o[0])*(p_y[0]-p_o[0])+(p_z[1]-p_o[1])*(p_y[1]-p_o[1])+(p_z[2]-p_o[2])*(p_y[2]-p_o[2]) <= 1e-10:
    #     print("Z Y 正交")
    # else:
    #     print("Z Y 不正交 = ", (p_z[0]-p_o[0])*(p_y[0]-p_o[0])+(p_z[1] -
    #           p_o[1])*(p_y[1]-p_o[1])+(p_z[2]-p_o[2])*(p_y[2]-p_o[2]))
    # if -1e-10 <= (p_x[0]-p_o[0])*(p_z[0]-p_o[0])+(p_x[1]-p_o[1])*(p_z[1]-p_o[1])+(p_x[2]-p_o[2])*(p_z[2]-p_o[2]) <= 1e-10:
    #     print("X Z 正交")
    # else:
    #     print("X Z 不正交 = ", (p_x[0]-p_o[0])*(p_z[0]-p_o[0])+(p_x[1] -
    #           p_o[1])*(p_z[1]-p_o[1])+(p_x[2]-p_o[2])*(p_z[2]-p_o[2]))


def DrawTrajectory(gt_data, esti_data):
    x_gt_arrary = []
    y_gt_arrary = []
    z_gt_arrary = []
    for i in range(gt_data.size):
        x_gt_arrary.append(gt_data[i][1])
        y_gt_arrary.append(gt_data[i][2])
        z_gt_arrary.append(gt_data[i][3])

    x_esti_arrary = []
    y_esti_arrary = []
    z_esti_arrary = []
    for i in range(esti_data.size):
        x_esti_arrary.append(esti_data[i][1])
        y_esti_arrary.append(esti_data[i][2])
        z_esti_arrary.append(esti_data[i][3])

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    a1 = ax.plot(x_gt_arrary, y_gt_arrary, z_gt_arrary,
                 'r-', label='groundtruth')
    a2 = ax.plot(x_esti_arrary, y_esti_arrary,
                 z_esti_arrary, 'c-', label='estimated')
    # 绘制姿态
    for gt in gt_data:
        DrawCoordinateArrow(ax=ax, x=gt[1], y=gt[2], z=gt[3],
                            qx=gt[4], qy=gt[5], qz=gt[6], qw=gt[7], length_arrow=0.01)

    for esti in esti_data:
        DrawCoordinateArrow(ax=ax, x=esti[1], y=esti[2], z=esti[3],
                            qx=esti[4], qy=esti[5], qz=esti[6], qw=esti[7], length_arrow=0.05)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels)
    # ax.legend()
    ax.set_title('Trajectory Error Analyze', fontsize=15, color='k')
    plt.show()


gt_data, esti_data = ReadTrajectory(groundtruth_file, estimated_file)
DrawTrajectory(gt_data, esti_data)

# ------------test-----------------
# fig = plt.figure()
# # ax = fig.add_subplot(111, projection='3d')
# ax = fig.gca(projection='3d')

# DrawCoordinateArrow(ax=ax, x=-0.338476338118489872, y=0.0359025381510416075, z=-0.382748686588541775, qx=0.773160588428214002, qy=0.209890773601576153,
#                     qz=-0.587109258956292734, qw=-0.116065867963249775, length_arrow=0.5)
# ax.grid()
# ax.set_xlabel('X')
# ax.set_xlim3d(-3, 3)
# ax.set_ylabel('Y')
# ax.set_ylim3d(-3, 3)
# ax.set_zlabel('Z')
# ax.set_zlim3d(-3, 3)
# plt.show()
# ---------------------------------------
