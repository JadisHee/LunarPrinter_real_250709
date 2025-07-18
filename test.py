import transform_tools
import numpy as np

code_bias = transform_tools.code_bias_transform()
tools = transform_tools.Tools()
if __name__ == "__main__":
    
    # # tools = Tools()
    # V1 = [4.3,0.325,0,0,0,-0.1083*np.pi/180]
    # # tag_1 --> cam
    # T1 = tools.pos_vec_to_pos_mat(V1)
    # np.set_printoptions(suppress=True, precision=6)
    # # print(T1)

    # V2 = [-2483.8,2503.76,1996.53,0,0,-1.4693*np.pi/180]
    # # world --> arm_base
    # T2 = tools.pos_vec_to_pos_mat(V2)
    # np.set_printoptions(suppress=True, precision=6)
    # # print(T2)

    # # world --> tag_1
    # T3 = np.array([
    #     [ 0.999891, 0.014742, 0, -2485.4],
    #     [-0.014742, 0.999891, 0, 2522.82],
    #     [0,0,1,0],
    #     [0,0,0,1]
    # ])

    # # world --> tag_1
    # T1 = np.array([
    #     [ 0.999891, 0.014742, 0, -2485.4],
    #     [-0.014742, 0.999891, 0, 2522.82],
    #     [0,0,1,0],
    #     [0,0,0,1]
    # ])

    # # tag_1 --> cam
    # V2 = [4.24,0.97333,0,0,0,0.2*np.pi/180]
    # T2 = tools.pos_vec_to_pos_mat(V2)
    # np.set_printoptions(suppress=True, precision=6)
    # print(T2)

    # # cam --> arm_base
    # T3 = np.array([
    #     [  0.99996, 0.009011, 0, -2.382593],
    #     [-0.009011,  0.99996, 0, -19.363886],
    #     [0,0,1,1996.53],
    #     [0,0,0,1]
    # ])

    # # world --> arm_base
    # T4 = T1 @ T2 @ T3
    # np.set_printoptions(suppress=True, precision=6)
    # print(T4)
    # V4 = [-205.118,-20.502,0]
    # T4 = code_bias.CodeDateToTransMat(V4)
    # np.set_printoptions(suppress=True, precision=6)
    # print(T4)

    # V5 = [0,0,3.648]
    # T5 = code_bias.CodeDateToTransMat(V5)
    # np.set_printoptions(suppress=True, precision=6)
    # print(T5)

    V6 = [0,0,1.2387]
    T6 = code_bias.CodeDateToTransMat(V6)
    np.set_printoptions(suppress=True, precision=6)
    print(T6)

    # T7 = T5 @ T4
    # np.set_printoptions(suppress=True, precision=6)
    # print(T7)
    # A = np.array([
    #     [-109.259254],
    #     [-1109.295044],
    #     [0],
    #     [1]
    # ])
    # # A_ = np.linalg.inv(T4) @ A
    # A_ = T4 @ A
    # np.set_printoptions(suppress=True, precision=6)
    # print(A_)
    # T4 =np.linalg.inv(T1) @ np.linalg.inv(T3) @ T2
    
    # print(T4)