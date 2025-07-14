import numpy as np
import os
import csv
# import path_planer
import matplotlib.pyplot as plt
# import open3d as o3d

# from path_planer import arm_path_planning


def tail_viewer(paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for i in range(len(paths)):
        # 画点
        ax.scatter(paths[i][:,0], paths[i][:,1], paths[i][:,2], c='b', marker='.',linewidth=0.5, label='Points')
        
        ax.set_aspect('equal')
        # ax.axis('equal')
        # 按顺序连线
        ax.plot3D(paths[i][:,0], paths[i][:,1], paths[i][:,2], c='r', linestyle='-', linewidth=0.5, label='Connected Line')

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # 显示图例
    # ax.legend()

    # 显示图像
    plt.show()

if __name__ == "__main__":
    path_1 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/1.csv',delimiter=',')
    path_2 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/2.csv',delimiter=',')
    path_3 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/3.csv',delimiter=',')
    path_4 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/4.csv',delimiter=',')
    # tail_viewer(path_3[:,0],path_3[:,1],path_3[:,2])
    paths = [path_1,path_2,path_3,path_4]
    tail_viewer(paths)
    # 
    # tail_viewer(path_4[:,0],path_4[:,1],path_4[:,2])
    
