import numpy as np
import xml.etree.ElementTree as ET

class load_params:
    def __init__(self,param_file):
        
        tree = ET.parse(param_file)

        param = tree.getroot()

        agv_tree = param.find('agv')
        self.agv_attribute = agv_attribute(agv_tree)

        printer_tree = param.find('printer')
        self.printer_attribute = printer_attribute(printer_tree)

        pass


class agv_attribute():
    def __init__(self, agv_tree):
        self.ip = agv_tree.find('ip').text
        self.ams_id = agv_tree.find('ams_id').text
        self.port = int(agv_tree.find('port').text)

        self.auto_nav = int(agv_tree.find('auto_nav').text)
        self.nav_path = int(agv_tree.find('nav_path').text)
        
        tags_tree = agv_tree.findall('tag')
        
        self.tags_attribute = []
        for tag_tree in tags_tree:
            self.tags_attribute.append(tag_attribute(tag_tree))

        pass

class tag_attribute():
    def __init__(self, tag_tree):
        self.id = int(tag_tree.get('id'))
        mat_file = tag_tree.find('trans_mat_file').text
        self.trans_mat = np.loadtxt(mat_file,delimiter=',')
        pass

class printer_attribute():
    def __init__(self, printer_tree):
        self.ip = printer_tree.find('ip').text
        self.port = int(printer_tree.find('port').text)

        tool_mat_file = printer_tree.find('tool_mat_file').text
        self.tool_trans_mat = np.loadtxt(tool_mat_file,delimiter=',')
        
        trans_mat_cam_arm_file = printer_tree.find('trans_mat_cam_arm_file').text
        self.trans_mat_cam_arm = np.loadtxt(trans_mat_cam_arm_file,delimiter=',')

        tool_posture_file = printer_tree.find('tool_posture_file').text
        self.tool_posture = np.loadtxt(tool_posture_file,delimiter=',')

        self.path_file_doc = printer_tree.find('path_file_doc').text
        self.witch_floor = int(printer_tree.find('which_floor').text)
        self.which_part = int(printer_tree.find('which_part').text)
        self.linear_speed = float(printer_tree.find('linear_speed').text)
        self.linear_acc = float(printer_tree.find('linear_acc').text)
        self.rOverride = int(printer_tree.find('rOverride').text)

        self.first_floor_height = float(printer_tree.find('first_floor_height').text)
        self.floor_height = float(printer_tree.find('floor_height').text)

        self.sand_test = int(printer_tree.find('sand_test').text)
        self.sand_hight = float(printer_tree.find('sand_hight').text)
        pass