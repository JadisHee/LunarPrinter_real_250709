import numpy as np
import os
import csv
# import path_planer
import matplotlib.pyplot as plt
# import open3d as o3d

# from path_planer import arm_path_planning


def tail_viewer(paths_1,paths_2,paths_3,paths_4):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for path in paths_1:
        # 画点
        ax.scatter(path[:,0], path[:,1], path[:,2], c='b', marker='.',linewidth=0.5, label='Points')
        
        ax.set_aspect('equal')
        # ax.axis('equal')
        # 按顺序连线
        ax.plot3D(path[:,0], path[:,1], path[:,2], c='r', linestyle='-', linewidth=0.5, label='Connected Line')
    
    for path in paths_2:
        # 画点
        ax.scatter(path[:,0], path[:,1], path[:,2], c='b', marker='.',linewidth=0.5, label='Points')
        
        ax.set_aspect('equal')
        # ax.axis('equal')
        # 按顺序连线
        ax.plot3D(path[:,0], path[:,1], path[:,2], c='k', linestyle='-', linewidth=0.5, label='Connected Line')

    for path in paths_3:
        # 画点
        ax.scatter(path[:,0], path[:,1], path[:,2], c='b', marker='.',linewidth=0.5, label='Points')
        
        ax.set_aspect('equal')
        # ax.axis('equal')
        # 按顺序连线
        ax.plot3D(path[:,0], path[:,1], path[:,2], c='y', linestyle='-', linewidth=0.5, label='Connected Line')

    for path in paths_4:
        # 画点
        ax.scatter(path[:,0], path[:,1], path[:,2], c='b', marker='.',linewidth=0.5, label='Points')
        
        ax.set_aspect('equal')
        # ax.axis('equal')
        # 按顺序连线
        ax.plot3D(path[:,0], path[:,1], path[:,2], c='c', linestyle='-', linewidth=0.5, label='Connected Line')

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # 显示图例
    # ax.legend()

    # 显示图像
    plt.show()

if __name__ == "__main__":
    path_1_1 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/1.csv',delimiter=',')
    path_2_1 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/2.csv',delimiter=',')
    path_3_1 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/3.csv',delimiter=',')
    path_4_1 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/4.csv',delimiter=',')

    path_1_2 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/2/1.csv',delimiter=',')
    path_2_2 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/2/2.csv',delimiter=',')
    path_3_2 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/2/3.csv',delimiter=',')
    path_4_2 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/2/4.csv',delimiter=',')

    path_1_3 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/1/1.csv',delimiter=',')
    path_2_3 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/1/2.csv',delimiter=',')
    path_3_3 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/1/3.csv',delimiter=',')
    path_4_3 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/1/4.csv',delimiter=',')
    # tail_viewer(path_3[:,0],path_3[:,1],path_3[:,2])
    paths = [
        path_1_1,path_2_1,path_3_1,path_4_1,
        path_1_2,path_2_2,path_3_2,path_4_2,
        path_1_3,path_2_3,path_3_3,path_4_3,
             ]
    paths_1 = [
        path_1_1,path_1_2,path_1_3
    ]
    paths_2 = [
        path_2_1,path_2_2,path_2_3
    ]
    paths_3 = [
        path_3_1,path_3_2,path_3_3
    ]
    paths_4 = [
        path_4_1,path_4_2,path_4_3
    ]
    tail_viewer(paths_1,paths_2,paths_3,paths_4)
    # 
    # tail_viewer(path_4[:,0],path_4[:,1],path_4[:,2])
    
