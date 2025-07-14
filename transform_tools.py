import numpy as np

class Tools:
    def __init__(self):
        pass

    def rot_mat_x(self,rx):
        '''
        * params: 
            rx: 绕x轴的旋转角,rad
        '''
        return np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
    
    def rot_mat_y(self,ry):
        '''
        * params: 
            ry: 绕y轴的旋转角,rad
        '''
        return np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
    
    def rot_mat_z(self,rz):
        '''
        * params: 
            rz: 绕z轴的旋转角,rad
        '''
        return np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
    
    def euler_to_rot_mat(self,rx,ry,rz,mode):
        '''
        * params: 
            rx: 绕x轴的旋转角,rad
            ry: 绕y轴的旋转角,rad
            rz: 绕z轴的旋转角,rad
            mode: 变换顺序 0: xyz 1: zyx
        * return:
            rot_mat: 旋转矩阵
        '''    
        
        mat_rx = self.rot_mat_x(rx)
        mat_ry = self.rot_mat_y(ry)
        mat_rz = self.rot_mat_z(rz)

        if mode == 0:
            rot_mat = mat_rx @ mat_ry @ mat_rz
        elif mode == 1:
            rot_mat = mat_rz @ mat_ry @ mat_rx 

        return rot_mat
    
    def pos_vec_to_pos_mat(self, pos_vec, mode=0):
        '''
        * params:
            pos_vec: 当前位姿，list[x,y,z,rx,ry,rz]，（m,rad）
            mode: 变换顺序 0: xyz 1: zyx, 默认为0
        * return:
            pos_mat: 变换矩阵
        '''
        rot_mat = self.euler_to_rot_mat(pos_vec[3],pos_vec[4],pos_vec[5],mode)

        pos_mat = np.array([
            [rot_mat[0,0], rot_mat[0,1], rot_mat[0,2], pos_vec[0]],
            [rot_mat[1,0], rot_mat[1,1], rot_mat[1,2], pos_vec[1]],
            [rot_mat[2,0], rot_mat[2,1], rot_mat[2,2], pos_vec[2]],
            [0,0,0,1]            
        ])

        return pos_mat
    
    def rot_mat_to_euler(self,rot_mat,mode):
        '''
        * params: 
            rot_mat: 旋转矩阵，np.array
            mode: 构造顺序，0: xyz, 1: zyx
        * return:
            rot_vec: 欧拉角，list[rx,ry,rz]，rad
        '''
        if not (rot_mat.shape == (3,3)):
                raise ValueError('输入为3x3的矩阵')
        
        if mode == 0:
            if np.isclose(rot_mat[0,2], -1.0):
                pitch = np.pi / 2
                roll = np.arctan2(rot_mat[1,0], rot_mat[1,1])
                yaw = 0

            elif np.isclose(rot_mat[0,2], 1.0):
                pitch = -np.pi / 2
                roll = np.arctan2(-rot_mat[1,0], -rot_mat[1,1])
                yaw = 0

            else:
                pitch = -np.arcsin(rot_mat[0,2])
                cos_pitch = np.cos(pitch)
                roll = np.arctan2( rot_mat[1,2] / cos_pitch, rot_mat[2,2] / cos_pitch)
                yaw = np.arctan2( rot_mat[0,1] / cos_pitch, rot_mat[0,0] / cos_pitch)
        elif mode == 1:
            if np.isclose(rot_mat[2, 0], -1.0):
                pitch = np.pi / 2
                yaw = np.arctan2(rot_mat[0, 1], rot_mat[0, 2])
                roll = 0
            elif np.isclose(rot_mat[2, 0], 1.0):
                pitch = -np.pi / 2
                yaw = np.arctan2(-rot_mat[0, 1], -rot_mat[0, 2])
                roll = 0
            else:
                pitch = -np.arcsin(rot_mat[2, 0])
                cos_pitch = np.cos(pitch)
                roll = np.arctan2(rot_mat[2, 1] / cos_pitch, rot_mat[2, 2] / cos_pitch)
                yaw = np.arctan2(rot_mat[1, 0] / cos_pitch, rot_mat[0, 0] / cos_pitch)

        return [roll, pitch, yaw]


    def pos_mat_to_pos_vec(self, pos_mat, mode):
        '''
        * params:
            pos_mat: 变换矩阵，np.array
            mode: 构造顺序，0: xyz, 1: zyx
        * return:
            pos_vec: 六维位姿, list[x,y,z,rx,ry,rz]，（m,rad）
        '''
        rot_mat = pos_mat[:3,:3]
        rot_vec = self.rot_mat_to_euler(rot_mat,mode)

        pos_vec = [pos_mat[0,3], pos_mat[1,3], pos_mat[2,3], rot_vec[0], rot_vec[1], rot_vec[2]]
        return pos_vec
    

class code_bias_transform:
    def __init__(self):
        self.tools = Tools()
        pass

    def CodeDateToTransMat(self,code_data):
        '''
        * func: 将二维码的数据转为当前二维码坐标系下的变换矩阵
        * params:
            code_data: [x,y,r], 单位:mm,deg
        * returns:
            pos_mat: 变换矩阵
        '''
        if code_data[2] > 180:
            rz_deg = code_data[2]-360
        else:
            rz_deg = code_data[2]
        
        rz_rad = np.deg2rad(rz_deg)

        pos_vec = [code_data[0],code_data[1],0,0,0,rz_rad]

        pos_mat = self.tools.pos_vec_to_pos_mat(pos_vec,0)

        return pos_mat




