#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# 视觉数据集采集工具：按键盘 0 截取带时间戳的图片 (含硬排空与画面翻转)

import cv2
import time
import os

# 存放截图的目录（如果不存在会自动创建）
SAVE_DIR = "/home/ucar/ucar_ws_copy/src/ucarmain2026/images_for_yolo"
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

def main():
    # 强制使用 V4L2 后端直接读取底层硬件，最稳定
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    
    # 锁定分辨率，与你们算法需要的尺寸保持一致
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("❌无法打开摄像头！请检查是否有其他程序（如巡线或 nanodet）正在占用 /dev/video0")
        return

    #清空初始硬件缓存与暖机
    print("正在清空底层图像缓存并等待自动曝光收敛...")
    for _ in range(15):  
        cap.grab()  # 只抽数据不解码，极速排空废片
    time.sleep(0.1) 
    print("✅ 缓存已排空，画面就绪！")


    print("========================================")
    print("截图工具已启动！")
    print("请确保鼠标焦点在这个弹出的视频窗口上")
    print("按下数字键 '0' 进行截图")
    print("按下字母键 'q' 退出程序")
    print("========================================")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("读取画面失败，重试中...")
            time.sleep(0.1)
            continue

        #翻转画面，回归物理现实方向
        # 1 代表水平翻转 (0是垂直，-1是水平加垂直)
        frame = cv2.flip(frame, 1) 


        # 显示实时画面
        cv2.imshow("Dataset Capture (Press '0' to save, 'q' to quit)", frame)

        # 监听键盘按键 (1ms 延迟)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('0'):
            # 生成时间戳，格式如：20260523_143059
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            file_name = f"capture_{timestamp}.jpg"
            save_path = os.path.join(SAVE_DIR, file_name)
            
            # 保存图片
            cv2.imwrite(save_path, frame)
            print(f"✅ 成功保存截图: {save_path}")

        elif key == ord('q') or key == 27: # 27 是 ESC 键
            print("退出采集。")
            break

    # 释放硬件资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()