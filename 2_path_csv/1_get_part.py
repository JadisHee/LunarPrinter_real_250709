import os

def slice_part(input_file,output_dir):

    # 输入文件路径
    # input_file = '2_path_csv/1_source_path/lunar_house_250623/printpath0.csv'

    # 保存目录
    # output_dir = '2_path_csv/2_pretreat_data/lunar_house_250623/'
    os.makedirs(output_dir, exist_ok=True)

    # 初始化
    path_id = -1
    path_data = []

    # 读取并分类
    with open(input_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('path'):
                if path_data:
                    # 保存前一个 path 的数据
                    output_path = os.path.join(output_dir, f'{path_id + 1}.csv')
                    with open(output_path, 'w') as out_f:
                        out_f.writelines(path_data)
                    path_data = []
                path_id += 1
            elif line:  # 只保存非空数据行
                path_data.append(line + '\n')

    # 保存最后一个 path 的数据
    if path_data:
        output_path = os.path.join(output_dir, f'{path_id + 1}.csv')
        with open(output_path, 'w') as out_f:
            out_f.writelines(path_data)

    # print("数据已成功拆分保存为 1.csv, 2.csv, 3.csv ...")

if __name__ == '__main__':

    root_dir = '2_path_csv/1_source_path/lunar_house_250623'
    files = [f for f in os.listdir(root_dir) if f.endswith('.csv')]
    files_count = len(files)

    for i in range(files_count):
        input_file = root_dir + '/printpath' + str(i) + '.csv'
        output_dir = '2_path_csv/2_pretreat_data/lunar_house_250623/' + str(i+1) + '/'
        slice_part(input_file,output_dir)
