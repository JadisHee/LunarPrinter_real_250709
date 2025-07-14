import numpy as np
import pandas as pd
import math
import os

def read_points(file_path):
    return np.loadtxt(file_path, delimiter=',')

def get_angle(p):
    # 返回极角（以原点为中心，逆时针）
    return math.atan2(p[1], p[0])

def quadrant(p):
    x, y = p[0], p[1]
    if x < 0 and y > 0:
        return 1  # 第二象限 → 第1组
    elif x > 0 and y > 0:
        return 2  # 第一象限 → 第2组
    elif x > 0 and y < 0:
        return 3  # 第四象限 → 第3组
    elif x < 0 and y < 0:
        return 4  # 第三象限 → 第4组
    else:
        return 0  # 在坐标轴上，忽略

def classify_and_sort(points):
    groups = {1: [], 2: [], 3: [], 4: []}

    for p in points:
        q = quadrant(p)
        if q in groups:
            groups[q].append(p)

    # 转为 numpy 数组并按逆时针角度排序
    for k in groups:
        group_np = np.array(groups[k])
        if len(group_np) > 0:
            angles = np.arctan2(group_np[:, 1], group_np[:, 0])
            sort_idx = np.argsort(-angles)
            groups[k] = group_np[sort_idx]
        else:
            groups[k] = np.empty((0, 3))

    return groups

def connect_groups(groups):
    # 按照你给出的连接顺序添加起始点
    if all(len(groups[i]) > 0 for i in [1, 2, 3, 4]):
        groups[1] = np.vstack([groups[1], groups[2][0]])
        groups[2] = np.vstack([groups[2], groups[3][0]])
        groups[3] = np.vstack([groups[3], groups[4][0]])
        groups[4] = np.vstack([groups[4], groups[1][0]])
    return groups

def save_groups(groups, output_dir="."):
    for i in range(1, 5):
        file_path = os.path.join(output_dir, f"{i}.csv")
        np.savetxt(file_path, groups[i], fmt="%.6f", delimiter=",")

def main(input_csv,output_dir):
    # input_csv = "2_path_csv/2_pretreat_data/lunar_house_250623/0/0.csv"  # 输入你的点云csv文件路径
    # output_dir = "2_path_csv/2_pretreat_data/lunar_house_250623/"         # 可自定义输出目录

    points = read_points(input_csv)
    groups = classify_and_sort(points)
    groups = connect_groups(groups)
    os.makedirs(output_dir, exist_ok=True)
    save_groups(groups, output_dir)

if __name__ == "__main__":

    # 设置目标文件夹路径
    folder_path = '2_path_csv/2_pretreat_data/lunar_house_250623'  # 替换为你的文件夹路径

    # 获取所有子文件夹
    subfolders = [f for f in os.listdir(folder_path)
                if os.path.isdir(os.path.join(folder_path, f))]

    folder_count = len(subfolders)

    for i in range(folder_count):
        root_dir = folder_path + '/' + str(i+1)
        files = [f for f in os.listdir(root_dir) if f.endswith('.csv')]
        files_count = len(files)
        for j in range(files_count):
            input_csv = root_dir + '/' + str(j+1) + '.csv'
            output_dir = root_dir + '/' + str(j+1) +'/'
            main(input_csv,output_dir)
