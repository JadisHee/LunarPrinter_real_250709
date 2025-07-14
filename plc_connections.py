
import ctypes
# 显式加载 DLL
ctypes.CDLL(r"C:\TwinCAT\Common64\TcAdsDll.dll")
import pyads


class plc_vars():
    def __init__(self,ams_id,ip,port):
        '''
        :func: 获取plc的变量
        '''
        self.plc = pyads.Connection(ams_id,port,ip)
        # self.plc.set_timeout(5000)
        self.INDEX_GROUP = 0x4020
        pass


    def connection_open(self):
        '''
        :func: 打开连接
        '''
        try:
            self.plc.open()
            print("已连接到 PLC")
            return 1
        except Exception as e:
            print(f"通信失败: {e}")
            return 0


    def connection_close(self):
        '''
        :func: 断开连接
        '''
        self.plc.close()
        print("与PLC连接已断开")


    def read_sys_status(self):
        '''
        :func: 读取系统手、自动状态
        '''
        self.sys_status = self.plc.read(self.INDEX_GROUP,140,pyads.PLCTYPE_DWORD)
        pass

    def write_runing_start_cmd(self):
        '''
        * func: 写入PLC灌浆机变频器开始运行指令
        '''
        self.plc.write(self.INDEX_GROUP,3002,1,pyads.PLCTYPE_BYTE)

    def write_runing_end_cmd(self):
        '''
        * func: 写入PLC灌浆机变频器停止运行指令
        '''
        self.plc.write(self.INDEX_GROUP,3002,5,pyads.PLCTYPE_BYTE)


    def read_extruder_variables(self):
        '''
        :func: 读取PLC中挤出头的相关控制变量
        '''
        # 设置运行速度
        self.rJogLowlVelocity = self.plc.read(self.INDEX_GROUP,11716,pyads.PLCTYPE_REAL)
        # 速度动态缩放系数
        self.rOverride = self.plc.read(self.INDEX_GROUP,11786,pyads.PLCTYPE_REAL)
        # 电机手动模式下正转指令
        self.bJogForM = self.plc.read(self.INDEX_GROUP,11846,pyads.PLCTYPE_BOOL)
        # 电机自动模式下正转指令
        self.bJogForA = self.plc.read(self.INDEX_GROUP,11850,pyads.PLCTYPE_BOOL)
        # 读取电机实时速度
        self.rActVelocity = self.plc.read(self.INDEX_GROUP,11648,pyads.PLCTYPE_REAL)
        # 驱动器复位
        self.bReset = self.plc.read(self.INDEX_GROUP,11844,pyads.PLCTYPE_BOOL)
        # 设备是否有报警信号
        self.bError = self.plc.read(self.INDEX_GROUP,11905,pyads.PLCTYPE_BOOL)
        pass
    
    def write_extruder_variables_bJogForM(self, bJogForM):
        '''
        :func: 向PLC中写入手动模式下是否正转的指令
        :param: 
            bJogForM: bool,是否正转
        '''
        self.plc.write(self.INDEX_GROUP,11847,bJogForM,pyads.PLCTYPE_BOOL)
        
        pass

    def write_extruder_variables_bJogForA(self, bJogForA):
        '''
        :func: 向PLC中写入自动模式下是否正转的指令
        :param: 
            bJogForA: bool,是否正转
        '''
        self.plc.write(self.INDEX_GROUP,11860,bJogForA,pyads.PLCTYPE_BOOL)
        pass

    def write_extruder_variables_rJogLowlVelocity(self,rJogLowlVelocity):
        '''
        :func: 向PLC中写入挤出头桨叶转速
        :param: 
            rJogLowlVelocity: float, 目标桨叶转速
        '''
        # plc_is_open = 
        print('plc是否已连接: ',self.plc.is_open)
        self.plc.write(self.INDEX_GROUP,11716,rJogLowlVelocity,pyads.PLCTYPE_REAL)
        self.plc.set_timeout(5000)
        pass


    def write_extruder_variables_rOverride(self,rOverride):
        '''
        :func: 向PLC中写入速度缩放系数
        :param: 
            rOverride: float, 速度动态缩放系数
        '''
        self.plc.write(self.INDEX_GROUP,11786,rOverride,pyads.PLCTYPE_REAL)
        pass


    def read_tag_variables(self):
        '''
        :func: 读取PLC中二维码的相关变量
        '''
        # 是否识别到二维码标签
        self.bTagValid = self.plc.read(self.INDEX_GROUP,2500,pyads.PLCTYPE_BOOL)
        # 二维码id
        self.nTagid = self.plc.read(self.INDEX_GROUP,2540,pyads.PLCTYPE_UDINT)
        # x轴偏差
        self.rxValue = self.plc.read(self.INDEX_GROUP,2544,pyads.PLCTYPE_REAL)
        # y轴偏差
        self.ryValue = self.plc.read(self.INDEX_GROUP,2548,pyads.PLCTYPE_REAL)
        # 偏航角偏差
        self.rAngle = self.plc.read(self.INDEX_GROUP,2552,pyads.PLCTYPE_REAL)
        pass


    def read_agvStatus_variables(self):
        '''
        :func: 读取PLC中AGV状态的相关变量
        '''
        # AGV是否停止
        self.agvStop = self.plc.read(self.INDEX_GROUP,222,pyads.PLCTYPE_BOOL)
        # AGV自动运行速度
        self.agvAutoVelocity = self.plc.read(self.INDEX_GROUP,1904,pyads.PLCTYPE_REAL)
        # AGV实际运行速度
        self.agvTrueVelocity = self.plc.read(self.INDEX_GROUP,1000,pyads.PLCTYPE_REAL)
        # 开始行车导航
        self.startAutoGuide = self.plc.read(self.INDEX_GROUP,2000,pyads.PLCTYPE_BOOL)
        # 停止行车导航
        self.stopAutoGuide = self.plc.read(self.INDEX_GROUP,2001,pyads.PLCTYPE_BOOL)
        # 车体是否正在进行自动导航
        self.ifAutoGuiding = self.plc.read(self.INDEX_GROUP,2100,pyads.PLCTYPE_BOOL)
        # 车体自动导航是否正常结束
        self.finishAutoGuide = self.plc.read(self.INDEX_GROUP,2101,pyads.PLCTYPE_BOOL)
        pass

    def write_code_nav_variables(self,codeid,x,y,angle):
        '''
        * function:
        * params:
            codeid: 目标二维码
            x: 目标二维码上x轴的偏移量
            y: 目标二维码上y轴的偏移量
            angle: 目标二维码上的偏航角
        '''
        # 写入目标二维码id
        self.plc.write(self.INDEX_GROUP,11786,codeid,pyads.PLCTYPE_UINT)
        # 写入目标二维码上x轴偏移量
        self.plc.write(self.INDEX_GROUP,11786,x,pyads.PLCTYPE_REAL)
        # 写入目标二维码上y轴偏移量
        self.plc.write(self.INDEX_GROUP,11786,y,pyads.PLCTYPE_REAL)
        # 写入目标二维码上的偏航角
        self.plc.write(self.INDEX_GROUP,11786,angle,pyads.PLCTYPE_REAL)

    def read_nav_finished_command(self):
        '''
        * function: 读取当前此段导航结束指令
        '''
        self.nav_finished = self.plc.read(self.INDEX_GROUP,2101,pyads.PLCTYPE_BOOL)

    def write_nav_target_command(self,i):
        '''
        * function: 执行自动导航指令
        * i: 需要执行的导航路径
        '''
        self.plc.write(self.INDEX_GROUP,24100,i,pyads.PLCTYPE_INT)

    def write_nav_start_command(self):
        '''
        * funciton: 导航开始
        '''
        self.plc.write(self.INDEX_GROUP,2000,True,pyads.PLCTYPE_BOOL)

    def read_distance_sensor_data(self):
        '''
        * function: 读取距离传感器
        '''
        distance_data = self.plc.read(self.INDEX_GROUP,24102,pyads.PLCTYPE_INT)
        # distance_data = self.plc.read_by_name('rHigDistance',pyads.PLCTYPE_REAL)
        return distance_data