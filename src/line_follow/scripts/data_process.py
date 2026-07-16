import csv

def flip_x_coordinates(input_csv, output_csv, image_width=640):
    """
    处理CSV文件，翻转点的X坐标
    
    参数:
    input_csv: 输入CSV文件路径
    output_csv: 输出CSV文件路径
    image_width: 图像宽度(默认640)
    """
    flipped_points = []
    
    with open(input_csv, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            try:
                # 确保每行有两个数字
                if len(row) < 2:
                    continue
                    
                # 解析坐标并翻转X
                x = float(row[0])
                y = float(row[1])
                flipped_x = image_width - x
                
                # 保留原始数值精度
                flipped_points.append((flipped_x, y))
            except ValueError:
                # 跳过无法解析的行
                continue
                
    # 写入翻转后的点
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(flipped_points)
        
    print(f"处理完成！翻转了 {len(flipped_points)} 个点")
    print(f"原始坐标示例: ({x}, {y}) → 新坐标: ({flipped_x}, {y})")

# 使用示例
input_file = "original_points.csv"
output_file = "flipped_points.csv"
flip_x_coordinates(input_file, output_file)