import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import transform_tools

code_bias = transform_tools.code_bias_transform()
tools = transform_tools.Tools()


def tail_viewer(ax, path):
    
    T0 = np.eye(4)
    draw_axes(ax, T0[:3, 3], T0[:3, :3], length=1000, colors=('r', 'g', 'b'),system_label='arm_base')

    
    # 画点
    ax.scatter(path[:,0], path[:,1], path[:,2], c='b', marker='.',linewidth=0.5, label='Points')
    
    ax.set_aspect('equal')
    # ax.axis('equal')
    # 按顺序连线
    ax.plot3D(path[:,0], path[:,1], path[:,2], c='r', linestyle='-', linewidth=0.5, label='Connected Line')


def draw_axes(ax, origin, rotation_matrix, length=1.0, colors=('r', 'g', 'b'),system_label=''):
    '''
    * func: 绘制3D坐标轴
    * params:
        ax: 3D坐标轴对象
        origin: 坐标轴原点
        rotation_matrix: 旋转矩阵
        length: 坐标轴长度
        colors: 坐标轴颜色
    '''
    axes = np.eye(3)  # 基础坐标轴
    transformed_axes = rotation_matrix @ axes  # 应用旋转矩阵
    for i in range(3):
        ax.quiver(*origin, *transformed_axes[:, i], length=length, color=colors[i], arrow_length_ratio=0.1)
    if system_label:
        ax.text(*origin, system_label, color='k', fontsize=12, ha='center', va='center')


# 创建一个新的图形对象
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 设置坐标轴范围
# ax.set_xlim([0, 2])
# ax.set_ylim([-2, 0])
# ax.set_zlim([0, 2])

# ax.set_xlim([-4, 4])
# ax.set_ylim([-4, 4])
# ax.set_zlim([0, 8])


# world
T0 = np.eye(4)
draw_axes(ax, T0[:3, 3], T0[:3, :3], length=3000, colors=('r', 'g', 'b'),system_label='world')

# 第一个打印点
V1 = [-2485.5,2522.82,0,0,0,-0.8447*np.pi/180]
T1 = tools.pos_vec_to_pos_mat(V1)
np.set_printoptions(suppress=True, precision=6)
# print(T1)
draw_axes(ax, T1[:3, 3], T1[:3, :3], length=1000, colors=('r', 'g', 'b'),system_label='tag_1')

# 第二个打印点
V2 = [2515.68,2482.54,0,0,0,-0.2052*np.pi/180]
T2 = tools.pos_vec_to_pos_mat(V2)
np.set_printoptions(suppress=True, precision=6)
# print(T2)
draw_axes(ax, T2[:3, 3], T2[:3, :3], length=1000, colors=('r', 'g', 'b'),system_label='tag_2')

# 第三个打印点
V3 = [2481.45,-2517.3,0,0,0,-0.7189*np.pi/180]
T3 = tools.pos_vec_to_pos_mat(V3)
np.set_printoptions(suppress=True, precision=6)
# print(T3)
draw_axes(ax, T3[:3, 3], T3[:3, :3], length=1000, colors=('r', 'g', 'b'),system_label='tag_3')

# 第四个打印点
V4 = [-2517.9,-2480.7,0,0,0,-1.1093*np.pi/180]
T4 = tools.pos_vec_to_pos_mat(V4)
np.set_printoptions(suppress=True, precision=6)
# print(T4)
draw_axes(ax, T4[:3, 3], T4[:3, :3], length=1000, colors=('r', 'g', 'b'),system_label='tag_4')

path_1 = np.loadtxt('2_path_csv/2_pretreat_data/lunar_house_250623/1/3/1.csv',delimiter=',')
tail_viewer(ax,path_1)


# T5 = np.array([
#     [0.723702, -0.690113, 0, -3571.318057],
#     [0.690113, 0.723702 ,0,12.078518],
#     [0,0,1,1996.53],
#     [0,0,0,1]
# ])


code_data_1 = [0,0,45]
# 第一个打印点-->车体
T_code_bias_1 = code_bias.CodeDateToTransMat(code_data_1)
# world --> cam
T5 = np.dot(T1,T_code_bias_1)
draw_axes(ax, T5[:3, 3], T5[:3, :3], length=500, colors=('r', 'g', 'b'),system_label='cam')

# cam_arm_base
T6 = np.array([
    [0.99996,0.009011,0,-30.351818],
    [-0.009011,0.99996,0,-46.291431],
    [0,0,1,1996.53],
    [0,0,0,1]
])
T7 = T5@T6
draw_axes(ax, T7[:3, 3], T7[:3, :3], length=500, colors=('r', 'g', 'b'),system_label='arm_base')

# # 车体 --> 机械臂底座
# vh = [0,0,2.006,0,0,0]
# T_h = tools.pos_vec_to_pos_mat(vh)
# # world --> 机械臂底座
# T6 = np.dot(T5,T_h)
# draw_axes(ax, T6[:3, 3], T6[:3, :3], length=1, colors=('r', 'g', 'b'),system_label='arm_base')


# # arm_base-->arm_tool
# v_tool = [-855.385647/1000,-2493.244792/1000,-1206.5/1000,-3.141000,0.000000,1.661000]
# T_tool = tools.pos_vec_to_pos_mat(v_tool)
# # world --> arm_tool
# T7 = np.dot(T6,T_tool)
# draw_axes(ax, T7[:3, 3], T7[:3, :3], length=1, colors=('r', 'g', 'b'),system_label='arm_tool')



# code_data_1 = [0.0013,0.0021,45.1]
# # 第一个打印点-->车体
# T_code_bias_1 = code_bias.CodeDateToTransMat(code_data_1)
# # world --> 车体
# T8 = np.dot(T4,T_code_bias_1)
# draw_axes(ax, T8[:3, 3], T8[:3, :3], length=1, colors=('r', 'g', 'b'),system_label='agv')

# # 车体 --> 机械臂底座
# vh = [0,0,2.006,0,0,0]
# T_h = tools.pos_vec_to_pos_mat(vh)
# # world --> 机械臂底座
# T9 = np.dot(T8,T_h)
# draw_axes(ax, T9[:3, 3], T9[:3, :3], length=1, colors=('r', 'g', 'b'),system_label='arm_base')


# # arm_base-->arm_tool
# v_tool = [1042.255208/1000,-2680.114353/1000,-1206.5/1000,-3.141000,0.000000,1.661000]
# T_tool = tools.pos_vec_to_pos_mat(v_tool)
# # world --> arm_tool
# T10 = np.dot(T9,T_tool)
# draw_axes(ax, T10[:3, 3], T10[:3, :3], length=1, colors=('r', 'g', 'b'),system_label='arm_tool')

# DucoScrew_Base --> DucoAntenna_Base
# V1 = [1.0666099999999998, -0.30418, 0.276, -3.140370923113397, -0.02181661564992912, 3.1353094682826135]
# T1 = np.array([
#     [ 0.9947,    0.0007,    0.1033,  688.4492],
#     [-0.0015,    1.0000,    0.0073,  -20.6477],
#     [-0.1033,   -0.0074,    0.9947,   71.6639],
#     [      0,         0,         0,    1.0000]
# ])
# draw_axes(ax, T1[:3, 3], T1[:3, :3], length=300, colors=('r', 'g', 'b'),system_label='Base2')

ax.set_aspect('equal')
# 设置标签和标题
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('transformation relationship diagram')

# 显示图形
plt.pause(5)

# 运行结束但保持窗口打开
input("Press [enter] to close the plot")

# print("lalala")