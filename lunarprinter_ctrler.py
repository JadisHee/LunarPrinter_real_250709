import time

import socket
import struct
import json
import binascii

import numpy as np
import keyboard
import nrc_interface as nrc
from plc_connections import plc_vars as plc

import transform_tools
from load_params import load_params


class plc_ctrl:
    def __init__(self,ams_id,ip,port):
        '''
        * func: plc控制
        * params:
            ams_id: plc的ams_id, str
            ip: plc的ip, str
            port: plc的端口, int
        '''
        self.plc_var = plc(ams_id,ip,port)

        # 连接plc
        self.plc_var.connection_open()
        pass

    def get_tag_data(self):
        self.plc_var.read_tag_variables()
        if self.plc_var.bTagValid == True:
            id = self.plc_var.nTagid
            x = self.plc_var.rxValue
            y = self.plc_var.ryValue
            r = self.plc_var.rAngle

            # self.plc_var.connection_close()
            return [id,x,y,r]
        else:
            # self.plc_var.connection_close()
            return 0


    def agv_nav(self,path_num):
        '''
        * func: 让agv按照给定路径进行导航
        * params:
            path_num: 路径编号, int
        '''

        # 获取当前agv所在二维码的数据
        self.plc_var.read_tag_variables()



        # 按路径编号执行
        if path_num == 1:
            if self.plc_var.bTagValid == True and self.plc_var.nTagid == 328:
                print('即将从328导航至421...')
                time.sleep(1)
                self.plc_var.write_nav_target_command(path_num)
                self.plc_var.write_nav_start_command()
            else:
                print('agv不在指令路径起点, 请将agv手动开至328')
                return 0
        elif path_num == 2:
            if self.plc_var.bTagValid == True and self.plc_var.nTagid == 421:
                print('即将从421导航至417')
                time.sleep(1)
                self.plc_var.write_nav_target_command(path_num)
                self.plc_var.write_nav_start_command()
            else:
                print('agv不在指令路径起点, 请将agv手动开至421')
                return 0
        elif path_num == 3:
            if self.plc_var.bTagValid == True and self.plc_var.nTagid == 417:
                print('即将从417导航至335')
                time.sleep(1)
                self.plc_var.write_nav_target_command(path_num)
                self.plc_var.write_nav_start_command()
            else:
                print('agv不在指令路径起点, 请将agv手动开至417')
                return 0
        elif path_num == 4:
            if self.plc_var.bTagValid == True and self.plc_var.nTagid == 335:
                print('即将从335导航至336')
                time.sleep(1)
                self.plc_var.write_nav_target_command(path_num)
                self.plc_var.write_nav_start_command()
            else:
                print('agv不在指令路径起点, 请将agv手动开至335')
                return 0
    
        print('agv正在导航...')    
        while True:
            self.plc_var.read_agvStatus_variables()
            if self.plc_var.finishAutoGuide == True:
                print('agv当前导航完成! ! !')
                break
            else:
                time.sleep(0.5)

    def extruder_start(self,rOverride):
        '''
        * function: 挤出头开始挤出
        * params:
            rOverride: 挤出速度率, int, [0,100]
        '''

        #######################################################################################################
        # 若不需要挤出头转动，注释掉以下区域代码
        #######################################################################################################

        # 挤出电机额定转速
        max_v = 10
        # 写入额定转速
        self.plc_var.write_extruder_variables_rJogLowlVelocity(max_v)
        # 获取系统的手、自动状态
        self.plc_var.read_sys_status()
        sys_status = self.plc_var.sys_status

        # 在手、自动不同的模式下旋转指令的位置不同，因此需要进行判断
        if sys_status == 40:
            self.plc_var.write_extruder_variables_bJogForM(True)
        elif sys_status == 80:
            self.plc_var.write_extruder_variables_bJogForA(True)
        
        # 写入挤出速度比例
        self.plc_var.write_extruder_variables_rOverride(rOverride)
        self.plc_var.read_extruder_variables()
        #######################################################################################################
        #######################################################################################################

        # 开启泵机变频器
        self.plc_var.write_runing_start_cmd()
        print('挤出头已开启')
        time.sleep(5)
        # self.plc_var.connection_close()

    def extruder_close(self):
        '''
        * function: 关闭挤出头
        '''
        # 获取系统手、自动状态
        self.plc_var.read_sys_status()
        sys_status = self.plc_var.sys_status

        # 将转速比例置为0，停止挤出
        self.plc_var.write_extruder_variables_rOverride(0)
        if sys_status == 40:
            self.plc_var.write_extruder_variables_bJogForM(False)
        elif sys_status == 80:
            self.plc_var.write_extruder_variables_bJogForA(False)
        
        # 关闭泵机变频器
        self.plc_var.write_runing_end_cmd()
        

    def read_hz(self):
        '''
        * function: 读取距离传感器的实际距离
        '''
        real_hz = self.plc_var.read_distance_sensor_data()
        return real_hz

class arm_ctrl:
    
    def __init__(self,ip,port):
        self.ip = ip
        self.port = str(port)
        # self.plc = plc_ctrl()
        pass

    def get_actual_pose(self,socketFd):
        
        pos = nrc.DoubleVector(6)
        nrc.get_current_position(socketFd,0,pos)
        
        position = [pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]]
        print(position)


    def servo_ready(self, socketFd):
        '''
        * func: 伺服就绪
        * param: 
            socketFd: int, 控制器id
        * return:
            state: int, 伺服状态
        '''
        status = 0
        status = nrc.get_servo_state(socketFd, status)

        jug = status[1]

        if jug == 0:
            nrc.set_servo_state(socketFd,1)
        elif jug == 2:
            nrc.clear_error(socketFd)
            nrc.set_servo_state(socketFd,1)
        
        status = 0
        status = nrc.get_servo_state(socketFd,status)
        jug = status[1]
        return jug

    def wait_for_running_over(self, socketFd):
        '''
        * func: 通过阻塞的形式等待运动完成
        * param:
            socketFd: int, 控制器id
        '''
        running_state = 0
        running_state = nrc.get_robot_running_state(socketFd,running_state)
        jug = running_state[1]
        print('当前运动状态：',jug)
        while jug == 2:
            time.sleep(0.2)
            running_state = 0
            running_state = nrc.get_robot_running_state(socketFd,running_state)
            jug = running_state[1]

    def power_ctr(self, socketFd, servo_state, mode):
        '''
        * func: 机械臂使能
        * param:
                socketFd: int, 控制器id
        * param:
                mode: bool, 上、下使能
        * return: 
                1: 上、下使能完成
                0: 上下电失败
        '''
        if mode == True:
            if servo_state != 1:
                return 0
            else:    
                nrc.set_servo_poweron(socketFd)
                return 1
        elif mode == False:
            nrc.set_servo_poweroff(socketFd)
            return 1
        else:
            return 0

    def moveP_test(self, socketFd, pos,vec,acc):
        '''
        * func: 让机械臂执行点动
        * params:
            socketFd: 对应连接的机械臂
            pos: 目标姿态, list[x,y,z,rx,ry,rz], 单位:mm、rad

        '''
        # 设置当前模式为示教模式
        nrc.set_current_mode(socketFd,0)

        # 设置全局速度
        nrc.set_speed(socketFd, 100)
        print('正在获取机械臂伺服状态...')
        time.sleep(1)

        servo_state = self.servo_ready(socketFd)
        print('当前伺服状态为: ',servo_state)

        print('机械臂即将使能...')
        self.power_ctr(socketFd,servo_state,True)

        target_pos = nrc.DoubleVector()
        for a in pos:
            target_pos.append(a)
        print('目标点为: ',[target_pos[0],target_pos[1],target_pos[2],target_pos[3],target_pos[4],target_pos[5]])
        
        temp_cmd = nrc.MoveCmd()
        temp_cmd.coord = 1
        temp_cmd.targetPosType = nrc.PosType_data
        temp_cmd.targetPosValue = target_pos
        temp_cmd.velocity = vec
        temp_cmd.acc = acc
        temp_cmd.dec = acc
        move_return = nrc.robot_movel(socketFd, temp_cmd)

        print('move_return: ', move_return)
        self.wait_for_running_over(socketFd)

        print('机械臂即将断开使能...')
        self.power_ctr(socketFd,servo_state,False)

    def moves_test(self, socketFd,p_list,vec,acc):
        '''
        * func: 让机械臂执行路径连续运动
        * params:
            sockedFd: 对应连接的机械臂
            p_list: np.array([p0,p1,p2,...,pn]), n<=19, 单位:mm、rad
        '''
        # 设置队列状态
        nrc.queue_motion_set_status(socketFd,True)
        time.sleep(1)
        pos = nrc.DoubleVector()
        
        moveCmd = nrc.MoveCmd()
        moveCmd.targetPosType = nrc.PosType_data

        moveCmd.coord = 1
        moveCmd.velocity = vec
        moveCmd.acc = acc
        moveCmd.dec = acc
        moveCmd.pl = 5

        for i in range(len(p_list)):

            for a in p_list[i]:
                pos.append(a)
            moveCmd.targetPosValue = pos
            print('即将写入: ',[pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]])
            if i == 0:
                queue_return = nrc.queue_motion_push_back_moveL(socketFd, moveCmd)
            else:
                queue_return = nrc.queue_motion_push_back_moveL(socketFd, moveCmd)
            print('queue_motion_push_back_moveS return: ', queue_return)
            pos.clear()

        send_return = nrc.queue_motion_send_to_controller(socketFd,len(p_list))
        print('\nqueue_motion_send_to_controller return: ', send_return)

        self.wait_for_running_over(socketFd)

        nrc.set_current_mode(socketFd,0)

class arm_status:
    def __init__(self,arm_ip):
        '''
        * func: 初始化获取机械臂状态的库
        * params:
            arm_ip: 机械臂的ip
        '''
        self.ip = arm_ip
        self.port = 7000
        pass

    def build_query_packet(self):
        '''
        * func: 构建以json格式为数据的tlv格式字节流
        * returns: 
            packet: 查询指令字节报文
        '''
        # 报文命令字，查询状态
        cmd = 0x9512

        # 查询数据的json数据包
        json_data = {
            "channel":1,
            "robot":1,
            "mode":1,
            "interval":10,
            "queryType":[
                "realPosACS","realPosMCS","axisVel","axisAcc"
            ]
        }

        # 将数据包转义为byte
        json_bytes = json.dumps(json_data,separators=(',',':')).encode('utf-8')
        
        # 数据包长度
        length = len(json_bytes)
        
        # tlv：type,length,value
        # 组合为字节报文
        tlv_body = struct.pack('>HH',length,cmd) + json_bytes
        
        # 计算crc32校验
        crc = binascii.crc32(tlv_body) & 0xFFFFFFFF
        
        # 组合完整的查询状态字节报文
        packet = b'\x4E\x66' + tlv_body + struct.pack('>I', crc)

        return packet

    def parse_response(self, data):
        '''
        * func: 解析回传数据
        * params: 
            data: 控制器回传tlv字节流
        * returns:
            real_pos_acs: 实时各轴关节角
            real_pos_mcs: 实时末端法兰位于基坐标系下的位姿
            axisVel: 实时关节速度
            axisAcc: 实时关节加速度
        '''
        if not data.startswith(b'\x4E\x66'):
            print('非法包头')
            return
        try:
            # 提取报文中数据部分
            tlv_data = data[2:~4]
            
            # 解析为字符串
            json_str = tlv_data[4:].decode('utf-8')
            
            # 利用json进行解析
            json_data = json.loads(json_str)
            
            # 提取出回传状态数据
            reply = json_data.get("replyData", {})
            
            # 再次提取出各数据
            real_pos_acs = reply.get("realPosACS")
            real_pos_mcs = reply.get("realPosMCS")
            axisVel = reply.get("axisVel")
            axisAcc = reply.get("axisAcc")

            return real_pos_acs, real_pos_mcs, axisVel, axisAcc

        except Exception as e:
            print("解析错误: ", e)

    def open_boardcast(self):
        '''
        * func: 打开7000端口的状态广播
        '''
        with socket.create_connection((self.ip,self.port)) as sock:
            packet = self.build_query_packet()
            sock.sendall(packet)
    
    def get_arm_status(self):
        '''
        * func: 获取机械臂运动状态
        * returns:
            real_pos_acs: 实时关节角
            real_pos_msc: 末端空间坐标
            axis_vel: 关节速度
            axis_acc: 关节加速度
        '''
        
        sock = socket.create_connection((self.ip,self.port))

        recv_data = sock.recv(4096)

        real_pos_acs, real_pos_msc, axis_vel, axis_acc = self.parse_response(recv_data)

        return real_pos_acs, real_pos_msc, axis_vel, axis_acc


class printer_ctrl:
    def __init__(self,ctrl_params_file):
        self.tools = transform_tools.Tools()
        self.params = load_params(ctrl_params_file)

        self.plc = plc_ctrl(
            ams_id=self.params.agv_attribute.ams_id,
            ip=self.params.agv_attribute.ip,
            port=self.params.agv_attribute.port 
        )

        self.arm = arm_ctrl(
            ip=self.params.printer_attribute.ip,
            port=self.params.printer_attribute.port
        )

        self.pos_0 = [0,-2500,-500,-3.141,0,1.661]
        pass

    def tag_pose_to_mat(self,tag_pos):
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


    def path_from_world_to_arm(self,tag_data):
        # 计算当前所在二维码与二维码相机之间的变换
        tag_id = tag_data[0]
        current_tag_pos = [tag_data[1],tag_data[2],tag_data[3]]

        # tag --> cam
        trans_mat_current_tag = self.tag_pose_to_mat(current_tag_pos)

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
        source_path_file = self.params.printer_attribute.path_file_doc + '/' + str(self.params.printer_attribute.witch_floor) + '/' + str(self.params.printer_attribute.which_part) + '/' + str(which_cut) + '.csv'
        source_path = np.loadtxt(source_path_file, delimiter=',')
        tool_path = source_path
        # 载入世界坐标系至当前二维码的固定变换
        for i in range(len(self.params.agv_attribute.tags_attribute)):
            if self.params.agv_attribute.tags_attribute[i].id == tag_id:
                # world --> tag
                trans_mat_world_tag = self.params.agv_attribute.tags_attribute[i].trans_mat

        # cam --> arm_base
        trans_mat_cam_arm = self.params.printer_attribute.trans_mat_cam_arm

        # world --> arm_base
        trans_mat_world_arm =  trans_mat_world_tag @ trans_mat_current_tag @ trans_mat_cam_arm @ trans_mat_bias
        np.set_printoptions(suppress=True, precision=6)
        # print(trans_mat_world_arm)

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
            flange_waypoint = self.params.printer_attribute.tool_trans_mat @ tool_waypoint
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
            flange_tail_path[q,3] = self.params.printer_attribute.tool_posture[0]
            flange_tail_path[q,4] = self.params.printer_attribute.tool_posture[1]
            flange_tail_path[q,5] = self.params.printer_attribute.tool_posture[2]


        return flange_tail_path


    def sand_test(self):
        try:
            print('正在获取当前二维码信息')
            tag_data = self.plc.get_tag_data()

            if tag_data != 0:
                print('获取二维码成功，当前二维码数据为: ', tag_data)
                pos_list = self.path_from_world_to_arm(tag_data)

                for i in range(len(pos_list)):
                    pos_list[i,2] = pos_list[i,2] + 200
                socketFd = nrc.connect_robot(self.arm.ip,self.arm.port)
                print('初始化机械臂id: ', socketFd)
                if socketFd <= 0:
                    print('连接失败，程序即将退出')
                    time.sleep(3)
                    return 0
                else:
                    print('机械臂连接成功')
                
                # 获取打印路径的起始点
                p_start = [pos_list[0,0], pos_list[0,1], pos_list[0,2], pos_list[0,3], pos_list[0,4], pos_list[0,5]]
                print("按空格键前往起点...",p_start)
                keyboard.wait('space')
                # print("正在运行连续路径...")
                print('正在前往此段打印起始点...')

                # 点动至路径起始点
                self.arm.moveP_test(socketFd,p_start,vec=self.params.printer_attribute.linear_speed,acc=self.params.printer_attribute.linear_acc)
                nrc.disconnect_robot(socketFd)

                print("按空格键继续...")
                keyboard.wait('space')
                print("正在运行连续路径...")

                # 重新连接机械臂
                socketFd = nrc.connect_robot(self.arm.ip,self.arm.port)
                print('初始化机械臂id: ', socketFd)
                if socketFd <= 0:
                    print('连接失败，程序即将退出')
                    time.sleep(3)
                    return 0
                else:
                    print('机械臂连接成功')
                
                # 机械臂开始连续运动
                print('即将开始连续运动')
                self.arm.moves_test(socketFd,pos_list,vec=self.params.printer_attribute.linear_speed,acc=self.params.printer_attribute.linear_acc)
                print('连续运动结束')
                # 断开连接
                nrc.disconnect_robot(socketFd)

                # 此段结束后的抬升点
                p_end = [ 
                    pos_list[len(pos_list)-1,0], 
                    pos_list[len(pos_list)-1,1], 
                    pos_list[len(pos_list)-1,2] + 100, 
                    pos_list[len(pos_list)-1,3], 
                    pos_list[len(pos_list)-1,4], 
                    pos_list[len(pos_list)-1,5]
                    ]
                
                print("按空格键略微抬升")
                keyboard.wait('space')
                print("正在抬升...")
                # 
                socketFd = nrc.connect_robot(self.arm.ip,self.arm.port)
                print('初始化机械臂id: ', socketFd)
                if socketFd <= 0:
                    print('连接失败，程序即将退出')
                    time.sleep(3)
                    return 0
                else:
                    print('机械臂连接成功')
                self.arm.moveP_test(socketFd,p_end,vec=self.params.printer_attribute.linear_speed,acc=self.params.printer_attribute.linear_acc)
                nrc.disconnect_robot(socketFd)
                time.sleep(1)
                print('此段路径行走完毕')

                return 1
            else:
                return 0

        except Exception as e:
            print('发送错误：',e)
            return 0




class print_ctrl:
    def __init__(self,arm_ip,arm_port,plc_ams_id,plc_ip,plc_port):
        self.plc = plc_ctrl(plc_ams_id,plc_ip,plc_port)
        
        self.arm = arm_ctrl(arm_ip,arm_port)

        self.pos_0 = [0,-2500,-500,-3.141,0,1.661]
        pass

    def hz_confirm(self,pos_list):
        
        real_hz_dist = []
        d_hz_list = []
        for i in range(len(pos_list)):
            pos_ = pos_list[i]
            # y轴补偿
            pos_[1] = pos_[1]-118
            pos_[2] = pos_[2]+200
            socketFd = nrc.connect_robot(self.arm.ip,self.arm.port)
            print('初始化机械臂id: ', socketFd)
            if socketFd <= 0:
                print('连接失败，程序即将退出')
                time.sleep(3)
                return 0
            else:
                print('机械臂连接成功')

            self.arm.moveP_test(socketFd,pos_)
            nrc.disconnect_robot(socketFd)
            if i == 0:
                print("等待机械臂稳定后,按空格键继续...")
                keyboard.wait('space')
            else:
                time.sleep(2)
            # print("正在运行连续路径...")
            # 高度补偿
            real_hz = self.plc.read_hz() - 353 
            # real_hz = self.plc.read_hz()
            real_hz_dist.append(real_hz)

            d_hz = real_hz - 200 - 25
            
            if d_hz >=15:
                # 高度补偿
                time.sleep(2)
                real_hz = self.plc.read_hz() - 353 
                # real_hz = self.plc.read_hz()
                real_hz_dist.append(real_hz)

                d_hz = real_hz - 200

            d_hz_list.append(d_hz)

            print(real_hz,d_hz)

        # print("按空格键回到起始点...")
        # keyboard.wait('space')
        # print("正在回到起始点...")
        # print('初始化机械臂id: ', socketFd)
        # if socketFd <= 0:
        #     print('连接失败，程序即将退出')
        #     time.sleep(3)
        #     return 0
        # else:
        #     print('机械臂连接成功')

        # self.arm.moveP_test(socketFd,self.pos_0)
        # nrc.disconnect_robot(socketFd)
        return real_hz_dist, d_hz_list
    

    def one_tail_print(self,pos_list):
        try:
            socketFd = nrc.connect_robot(self.arm.ip,self.arm.port)
            print('初始化机械臂id: ', socketFd)
            if socketFd <= 0:
                print('连接失败，程序即将退出')
                time.sleep(3)
                return 0
            else:
                print('机械臂连接成功')
            
            # 获取打印路径的起始点
            p_start = [pos_list[0,0], pos_list[0,1], pos_list[0,2], pos_list[0,3], pos_list[0,4], pos_list[0,5]]
            print('正在前往此段打印起始点')

            # 点动至路径起始点
            self.arm.moveP_test(socketFd,p_start)
            nrc.disconnect_robot(socketFd)

            print("按空格键继续...")
            keyboard.wait('space')
            print("正在运行连续路径...")

            # 重新连接机械臂
            socketFd = nrc.connect_robot(self.arm.ip,self.arm.port)
            print('初始化机械臂id: ', socketFd)
            if socketFd <= 0:
                print('连接失败，程序即将退出')
                time.sleep(3)
                return 0
            else:
                print('机械臂连接成功')
            # 开启挤出头
            print('即将开启挤出头')
            self.plc.extruder_start(15)
            
            # 机械臂开始连续运动
            print('即将开始连续运动')
            self.arm.moves_test(socketFd,pos_list)
            # 关闭挤出头
            self.plc.extruder_close()
            # 断开连接
            nrc.disconnect_robot(socketFd)


            print("按空格键回升...")
            keyboard.wait('space')
            print("正在回升...")
            time.sleep(1)
            # 此段结束后的抬升点
            p_end = [ 
                pos_list[len(pos_list)-1,0], 
                pos_list[len(pos_list)-1,1], 
                pos_list[len(pos_list)-1,2] + 100, 
                pos_list[len(pos_list)-1,3], 
                pos_list[len(pos_list)-1,4], 
                pos_list[len(pos_list)-1,5]
                ]
            
            # 
            socketFd = nrc.connect_robot(self.arm.ip,self.arm.port)
            print('初始化机械臂id: ', socketFd)
            if socketFd <= 0:
                print('连接失败，程序即将退出')
                time.sleep(3)
                return 0
            else:
                print('机械臂连接成功')
            self.arm.moveP_test(socketFd,p_end)
            nrc.disconnect_robot(socketFd)
            time.sleep(1)
            print('此段路径行走完毕')

            # print("按空格键回到起始点...")
            # keyboard.wait('space')
            # print("正在回到起始点...")
            # print('初始化机械臂id: ', socketFd)
            # if socketFd <= 0:
            #     print('连接失败，程序即将退出')
            #     time.sleep(3)
            #     return 0
            # else:
            #     print('机械臂连接成功')

            # self.arm.moveP_test(socketFd,self.pos_0)
            # nrc.disconnect_robot(socketFd)
            
            return 1


        except Exception as e:
            return 0
        
    def first_floor_dhz(self,file_name):
        
        pos_list = np.loadtxt(file_name + '.csv',delimiter=',')


        _,dh = self.hz_confirm(pos_list)
        # np.savetxt()

        np.savetxt(file_name + '_dh.csv', dh, delimiter=',',fmt='%.6f')

    def tail_print(self,file_name,dh):
        pos_list = np.loadtxt(file_name + '.csv',delimiter=',')
        d_hz = np.loadtxt(file_name + '_dh.csv',delimiter=',')

        for i in range(len(d_hz)):
            pos_list[i,2] = pos_list[i,2] - d_hz[i] + dh

        self.one_tail_print(pos_list)