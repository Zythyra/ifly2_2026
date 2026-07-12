#!/home/ucar/miniforge3/envs/opencv_env/bin/python
# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import cv2
import sys
import select

video = cv2.VideoCapture('/dev/video0')
if not video.isOpened():
    print("无法打开摄像头！")
    exit()

i = 0

while True:
    ret, frame = video.read()
    if not ret:
        print("无法获取视频帧")
        break

    resized_frame = cv2.resize(frame, (1280, 720))
    # cv2.imshow("Camera (Terminal Control)", resized_frame)

    # 检测终端输入（适用于SSH）
    if select.select([sys.stdin], [], [], 0)[0]:
        key = sys.stdin.read(1)
        if key == 's':
            i += 1
            imgname = f"/home/ucar/ucar_ws_copy/src/ucar_camera/for_loc/yang_1.jpg"
            cv2.imwrite(imgname, frame)
            print(f"手动保存: {imgname}")
        elif key == 'q':
            break

    # # 检测窗口关闭按钮
    # if cv2.getWindowProperty("Camera (Terminal Control)", cv2.WND_PROP_VISIBLE) < 1:
    #     break

video.release()
cv2.destroyAllWindows()