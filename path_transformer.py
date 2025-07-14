import numpy as np
from transform_tools import Tools

class path_from_world_to_armbase:
    def __init__(self, path, tag_data, tool_pose):
        
        self.tools = Tools
        
        
        self.source_path = path
        self.tag_data = tag_data
        self.tag_id = self.tag_data[0]
        # self.tag_pose = tag_data[0:]
        self.tool_pose = tool_pose
        

        pass

    def get_tag_cam_mat(self):
        
        x = self.tag_data[1]
        y = self.tag_data[2]
        rz = self.tag_data[3]

        tag_vec = [x,y,0,0,0,rz]

        tag_mat = self.tools.pos_vec_to_pos_mat(tag_vec)

        return tag_mat
    
    def get_world_tag_mat(self):
        
        # tag_pose_mat = self.tag_trans.CodeDateToTransMat()