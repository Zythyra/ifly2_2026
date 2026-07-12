#! /home/ucar/miniforge3/envs/opencv_env/bin/python
# -*- coding: UTF-8 -*-

import torch

# 加载 .pth 文件
state_dict = torch.load('/home/ucar/ucar_ws_copy/src/yolo/src/yolov5-master/model_best.pth', map_location='cpu',weights_only=True)

# 保存为 .pt 文件
torch.save(state_dict, 'model_best.pt')