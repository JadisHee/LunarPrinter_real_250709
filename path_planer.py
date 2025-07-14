import re
# import pyads
import numpy as np

from transform_tools import Tools


# from CalcTools import Tools
from scipy.spatial.transform import Rotation


class gcode_analysis:
    def __init__(self):
        pass

    def parse_gcode(self,gcode_file_path):
        '''
            输入:
                gcode_file_path: 导出gcode文件的路径
                    string
            输出: 
                x,y,z: 坐标     s:打印速度
                    np.array([
                        [x1,y1,z1,s1],
                        [x2,y2,z2,s2],
                        ...,
                        [xn,yn,zn,sn]
                    ])
        '''
        # 定义容器
        points = []

        # 定义Z轴
        current_z = 0.0

        # 读取gcode
        with open(gcode_file_path, 'r') as file:
            # 按行读取
            for line in file:
                if line.startswith('G1'):
                    x_match = re.search(r'X([-+]?\d*\.?\d+)', line)
                    y_match = re.search(r'Y([-+]?\d*\.?\d+)', line)
                    z_match = re.search(r'Z([-+]?\d*\.?\d+)', line)
                    e_match = re.search(r'E([-+]?\d*\.?\d+)', line)

                    x = float(x_match.group(1)) if x_match else None
                    y = float(y_match.group(1)) if y_match else None
                    z = float(z_match.group(1)) if z_match else current_z  # 如果没有Z值，沿用上一个Z
                    e = 1 if e_match else 0


                    if x is not None and y is not None:
                        points.append([x, y, z, e])  # 仅添加完整坐标

                    current_z = z  # 更新 Z 值

        return np.array(points)

                

class arm_path_planning:
    

    def __init__ (self, ori_path_points, pre_pos_vector, tool_euler=None, plc_ip=None, plc_port=None):
        '''
        输入: 
            ori_path_points: gcode解析点云
                np.array([
                    [x1,y1,z1,s1],
                    [x2,y2,z2,s2],
                    ...,
                    [xn,yn,zn,sn]
                ])
            pos_vector: 打印目标位置相对于机械臂基座的位姿
                list[x,y,z,rx,ry,rz]
            tool_euler: 打印头的欧拉角姿态 rad
                list[rx,ry,rz]
            plc_ip: AGV中plc的ip地址
                string
            plc_port: AGV中plc的端口号
                int
        '''
        self.tools = Tools()

        # pcl 的ip和端口
        self.plc_ip = plc_ip
        self.plc_port = plc_port

        # 打印路径点
        self.pre_path_points = ori_path_points[:,:3]
        # 挤出速度控制
        self.extrusion_speed = ori_path_points[:,3]

        # 打印目标位置相对于机械臂基座的位姿
        self.pre_pos_mat = self.tools.PosVecToPosMat(pre_pos_vector)

        # 打印头的欧拉角
        self.tool_posture = tool_euler

        # self.agv_pose = tools.PosVecToPosMat(agv_position)

        pass


    '''
        获取agv当前的状态
            输入：
                plc_ip: plc的ip地址
                    string
                plc_port: plc的端口
                    int
            返回：
                agv_info: agv当前的状态数据
                    list[agv_state,code_id,code_pose]
                        agv_state: 当前agv的状态
                            0: 修正完成
                            1: 正在运动
                        code_id: 当前二维码的码值
                            int
                        code_pose: 当前agv在二维码上的姿态
                            list[x,y,0,0,0,rz]
    '''
    # def get_agv_info(self):

    #     # 连接plc
    #     plc = pyads.Connection(self.plc_ip,self.plc_port)
    #     try:
    #         # 打开连接
    #         plc.open()
    #         if plc.is_open:
    #             print("plc 连接成功！")
                
    #             # 获取数据
    #             agv_state = plc.read_by_name("MAIN.agvSTATE",123, pyads.PLCTYPE_INT)
    #             id = plc.read_by_name("MAIN.codeID",123, pyads.PLCTYPE_INT)
    #             x = plc.read_by_name("MAIN.codeX",123, pyads.PLCTYPE_INT)
    #             y = plc.read_by_name("MAIN.codeY",123, pyads.PLCTYPE_INT)
    #             r = plc.read_by_name("MAIN.codeR",123, pyads.PLCTYPE_INT)

    #             agv_pose = [x,y,0,0,0,r]
                
    #             # 关闭连接
    #             plc.close()
    #             print("plc 连接已关闭！")
    #             return [agv_state,id,agv_pose]
                
    #     except Exception as e:
    #         print(f"与plc的通讯错误：{e}")

    #     finally:
    #         plc.close()
    #         print("plc 连接已关闭！")


    
    def tf_points(self,trans,points):
        '''
        执行点云变换
            输入：
                trans: 变换矩阵
                    np.array([...])
                points: 待变换点云
                    np.array([
                        [x0,y0,z0],
                        [x1,y1,z1],
                        ...
                        [xn,yn,zn]
                    ])
            返回:
                points_t
        '''
        ones = np.ones((points.shape[0],1))
        points_temp_1 = np.hstack((points,ones))

        points_temp_2 = (trans @ points_temp_1.T).T

        points_t = points_temp_2[:,:3]

        return points_t


    def self_path(self):


        # 将点云变换至地面
        self_path = self.tf_points(self.pre_pos_mat,self.pre_path_points)
        

        return self_path


    def global_path(self,code_bias):
        '''
        计算当前停车位置下基于全局的打印路径
        '''
        # 获取当前AGV在全局下的偏差
        codeID = code_bias[0]
        bias = code_bias[0:]
        current_agv_info = self.get_agv_info()
        for i in range(len(codeID)):
            if current_agv_info[1] == codeID[i]:
                current_bias_vec = bias[i,:]
                break
            else:
                pass
        current_code_err = self.tools.PosVecToPosMat(current_agv_info[2])
        current_code_bias_tf_mat = self.tools.PosVecToPosMat(current_bias_vec)

        global_bias = np.dot(current_code_err,current_code_bias_tf_mat)

        # 将点云变换至地面
        temp_path_1 = self.tf_points(self.pre_pos_mat,self.pre_path_points)
        
        # 将点云变换至全局
        global_path = self.tf_points(global_bias,temp_path_1)

        return global_path
