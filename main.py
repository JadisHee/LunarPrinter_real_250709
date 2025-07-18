from load_params import load_params
import numpy as np
import transform_tools
import matplotlib.pyplot as plt
import plc_connections

tools = transform_tools.Tools()
params = load_params('ctrl_params.xml')

def tag_pose_to_mat(tag_pos):
    '''
    * func: 二维码x,y,r数据转为变换矩阵
    * params:
        tag_pos: 通过二维码相机读取上来的数据, list[x,y,r], 单位:mm,deg
    * returns: 
        tag_mat: 变换矩阵
    '''
    tag_trans_tools = transform_tools.code_bias_transform()
    tag_mat = tag_trans_tools.CodeDateToTransMat(tag_pos)
    return tag_mat

def path_from_world_to_arm(tag_data):
    '''
    * func: 将全局坐标系下的路径变换至机械臂基坐标系下
    * params:
        tag_data: 当前实时二维码数据, list[id,x,y,r], 单位:mm,deg
    * returns:
        
    '''
    
    # 计算当前所在二维码与二维码相机之间的变换
    tag_id = tag_data[0]
    
    
    
    current_tag_pos = [tag_data[1],tag_data[2],tag_data[3]]
    # tag --> cam
    trans_mat_current_tag = tag_pose_to_mat(current_tag_pos)

    
    

    if tag_id == 421:
        which_cut = 1
    elif tag_id == 417:
        which_cut = 2
    elif tag_id == 335:
        which_cut = 3
    elif tag_id == 394:
        which_cut = 4

    trans_mat_bias_file = '0_trans_mat/trans_mat_' + str(which_cut) + '_bias.csv'
    trans_mat_bias = np.loadtxt(trans_mat_bias_file, delimiter=',')

    # 载入当前二维码对应的原始路径
    source_path_file = params.printer_attribute.path_file_doc + '/' + str(params.printer_attribute.witch_floor) + '/' + str(params.printer_attribute.which_part) + '/' + str(which_cut) + '.csv'
    source_path = np.loadtxt(source_path_file, delimiter=',')
    tool_path = source_path
    
    # 载入世界坐标系至当前二维码的固定变换
    for i in range(len(params.agv_attribute.tags_attribute)):
        if params.agv_attribute.tags_attribute[i].id == tag_id:
            # world --> tag
            trans_mat_world_tag = params.agv_attribute.tags_attribute[i].trans_mat

    # cam --> arm_base
    trans_mat_cam_arm = params.printer_attribute.trans_mat_cam_arm

    # world --> arm_base
    trans_mat_world_arm =  trans_mat_world_tag @ trans_mat_current_tag @ trans_mat_cam_arm
    # @ trans_mat_bias
    np.set_printoptions(suppress=True, precision=6)
    print(trans_mat_world_arm)

    # 计算变换后的路径点
    for j in range(len(source_path)):
        # 路径点增广
        source_waypoint = np.array([
            [source_path[j,0]],
            [source_path[j,1]],
            [source_path[j,2]],
            [1]
        ])
        target_waypoint = np.linalg.inv(trans_mat_world_arm) @ source_waypoint
        # 赋值
        tool_path[j,0] = target_waypoint[0,0]
        tool_path[j,1] = target_waypoint[1,0]
        tool_path[j,2] = target_waypoint[2,0]

    # 路径变换
    flange_path = tool_path
    for k in range(len(tool_path)):
        tool_waypoint = np.array([
            [tool_path[k,0]],
            [tool_path[k,1]],
            [tool_path[k,2]],
            [1]
        ])
        flange_waypoint = params.printer_attribute.tool_trans_mat @ tool_waypoint
        # 赋值
        flange_path[k,0] = flange_waypoint[0,0]
        flange_path[k,1] = flange_waypoint[1,0]
        flange_path[k,2] = flange_waypoint[2,0]

    # 给路径加入姿态
    flange_tail_path = np.zeros([len(flange_path),6])
    for q in range(len(flange_path)):
        flange_tail_path[q,0] = flange_path[q,0]
        flange_tail_path[q,1] = flange_path[q,1]
        flange_tail_path[q,2] = flange_path[q,2]
        flange_tail_path[q,3] = params.printer_attribute.tool_posture[0]
        flange_tail_path[q,4] = params.printer_attribute.tool_posture[1]
        flange_tail_path[q,5] = params.printer_attribute.tool_posture[2]


    return flange_tail_path


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

def tail_viewer(paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    T0 = np.eye(4)
    draw_axes(ax, T0[:3, 3], T0[:3, :3], length=1000, colors=('r', 'g', 'b'),system_label='arm_base')




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

def get_tag():
    ip = params.agv_attribute.ip
    port = params.agv_attribute.port
    ams_id = params.agv_attribute.ams_id
    plc = plc_connections.plc_vars(ams_id=ams_id,ip=ip,port=port)
    plc.connection_open()
    plc.read_tag_variables()

    if plc.bTagValid == True:
        id = plc.nTagid
        x = plc.rxValue
        y = plc.ryValue
        r = plc.rAngle

        plc.connection_close()
        return [id,x,y,r]
    else:
        plc.connection_close()
        return 0

if __name__ == '__main__':

    # tag_data = get_tag()
    # if tag_data != 0:
    tag_data = [421,-0.1,-0.3,44.7999]
    path = path_from_world_to_arm(tag_data)
    print(path)
    tail_viewer([path])