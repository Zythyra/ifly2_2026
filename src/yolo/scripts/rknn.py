#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import time
from rknnlite.api import RKNNLite

class YOLOv5RKNN:
    def __init__(self,
                 model_path,
                 target_size=640,
                 conf_threshold=0.85,
                 nms_threshold=0.45):
        self.target_size = target_size
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        
        self.class_names = ["Watermelon", "Cake", "Apple", "Banana", "Chili", "Tomato", "Milk", "Cola", "Potato"]
        self.num_classes = len(self.class_names)
        
        # YOLOv5 专用参数
        self.masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
        self.anchors = [[10,13], [16,30], [33,23], 
                        [30,61], [62,45], [59,119], 
                        [116,90], [156,198], [373,326]]
        
        self.rknn = self.init_model(model_path)
        print('[YOLOv5 RKNN] Model loaded successfully!')

    # ----------------- 模型初始化 -----------------
    def init_model(self, model_path):
        rknn = RKNNLite()
        ret = rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError('Load RKNN model failed!')
        ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        if ret != 0:
            raise RuntimeError('Init RKNN runtime failed!')
        return rknn

    # ----------------- 预处理 -----------------
    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def xywh2xyxy(self, x):
        y = np.copy(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
        y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
        y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
        y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
        return y

    # ----------------- 后处理过程 -----------------
    def process_output(self, input, mask, anchors):
        anchors = [anchors[i] for i in mask]
        grid_h, grid_w = map(int, input.shape[0:2])
        
        box_confidence = self.sigmoid(input[..., 4])
        box_class_probs = self.sigmoid(input[..., 5:])
        box_xy = self.sigmoid(input[..., :2]) * 2 - 0.5
        
        # 生成网格坐标
        col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
        row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
        col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        grid = np.concatenate((col, row), axis=-1)
        
        box_xy += grid
        box_xy *= int(self.target_size / grid_h)
        box_wh = pow(self.sigmoid(input[..., 2:4]) * 2, 2)
        box_wh = box_wh * anchors
        
        box = np.concatenate((box_xy, box_wh), axis=-1)
        return box, box_confidence, box_class_probs

    def filter_boxes(self, boxes, confidences, class_probs):
        boxes = boxes.reshape(-1, 4)
        confidences = confidences.reshape(-1)
        class_probs = class_probs.reshape(-1, class_probs.shape[-1])
        
        valid_mask = confidences >= self.conf_threshold
        boxes = boxes[valid_mask]
        confidences = confidences[valid_mask]
        class_probs = class_probs[valid_mask]
        
        class_max_score = np.max(class_probs, axis=-1)
        classes = np.argmax(class_probs, axis=-1)
        valid_class_mask = class_max_score >= self.conf_threshold
        
        boxes = boxes[valid_class_mask]
        classes = classes[valid_class_mask]
        scores = (class_max_score * confidences)[valid_class_mask]
        return boxes, classes, scores

    def nms_boxes(self, boxes, scores):
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]
        
        areas = w * h
        order = scores.argsort()[::-1]
        
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
            
            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1
            
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= self.nms_threshold)[0]
            order = order[inds + 1]
        return np.array(keep)

    # ----------------- 图像预处理 -----------------
    def preprocess(self, img):
        """Letterbox预处理，返回处理后的图像和变换参数"""
        h, w = img.shape[:2]
        scale = min(self.target_size / w, self.target_size / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        start_x = (self.target_size - new_w) // 2
        start_y = (self.target_size - new_h) // 2
        
        # 缩放图像
        resized = cv2.resize(img, (new_w, new_h))
        
        # 创建画布并放置图像
        canvas = np.full((self.target_size, self.target_size, 3), 114, dtype=np.uint8)
        canvas[start_y:start_y+new_h, start_x:start_x+new_w] = resized
        
        # 转换为RGB并归一化
        input_img = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.uint8)
        
        return input_img, scale, (start_x, start_y), (new_w, new_h), (w, h)

    # ----------------- 后处理 -----------------
    def postprocess(self, outputs, scale, pad, new_size, orig_size):
        """处理RKNN输出，返回检测框、置信度和类别"""
        # 准备输出数据
        input0_data = outputs[0].reshape([3, -1] + list(outputs[0].shape[-2:]))
        input1_data = outputs[1].reshape([3, -1] + list(outputs[1].shape[-2:]))
        input2_data = outputs[2].reshape([3, -1] + list(outputs[2].shape[-2:]))
        
        input_data = [
            np.transpose(input0_data, (2, 3, 0, 1)),
            np.transpose(input1_data, (2, 3, 0, 1)),
            np.transpose(input2_data, (2, 3, 0, 1))
        ]
        
        # 处理每个输出层
        boxes, classes, scores = [], [], []
        for input, mask in zip(input_data, self.masks):
            b, c, s = self.process_output(input, mask, self.anchors)
            b, c, s = self.filter_boxes(b, c, s)
            boxes.append(b)
            classes.append(c)
            scores.append(s)
        
        if not boxes:
            return np.empty((0, 4), dtype=np.float32), np.array([]), np.array([])
        
        # 合并所有检测结果
        boxes = np.concatenate(boxes)
        classes = np.concatenate(classes)
        scores = np.concatenate(scores)
        
        # 转换框坐标格式
        boxes = self.xywh2xyxy(boxes)
        
        # 坐标反变换到原始图像空间
        dw, dh = pad
        orig_w, orig_h = orig_size
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - dw) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - dh) / scale
        
        # 裁剪到图像边界
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, orig_w)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, orig_h)
        
        # 按类别分组NMS
        if len(classes) > 0:
            unique_classes = np.unique(classes)
            nms_boxes, nms_classes, nms_scores = [], [], []
            
            for cls in unique_classes:
                cls_mask = (classes == cls)
                cls_boxes = boxes[cls_mask]
                cls_scores = scores[cls_mask]
                
                if len(cls_boxes) > 0:
                    keep = self.nms_boxes(cls_boxes, cls_scores)
                    nms_boxes.append(cls_boxes[keep])
                    nms_classes.append(classes[cls_mask][keep])
                    nms_scores.append(cls_scores[keep])
            
            if nms_boxes:
                boxes = np.concatenate(nms_boxes)
                classes = np.concatenate(nms_classes)
                scores = np.concatenate(nms_scores)
        
        return boxes, scores, classes

    # ----------------- 可视化 -----------------
    def draw_detections(self, img, bboxes, scores, class_ids):
        out = img.copy()
        for box, sc, cid in zip(bboxes, scores, class_ids):
            x1, y1, x2, y2 = box.astype(int)
            x1 = max(0, x1); y1 = max(0, y1)
            x2 = min(img.shape[1]-1, x2); y2 = min(img.shape[0]-1, y2)
            
            label = f'{self.class_names[cid]} {sc:.2f}'
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(out, label, (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return out

    # ----------------- 推理 -----------------
    def inference(self, img):
        t0 = time.time()
        blob, scale, pad, new_size, orig_size = self.preprocess(img)
        outputs = self.rknn.inference(inputs=[blob])
        bboxes, scores, class_ids = self.postprocess(outputs, scale, pad, new_size, orig_size)
        dt = time.time() - t0
        # print(f'[Perf] Inference + Postprocess time: {dt*1000:.2f} ms')
        return bboxes, scores, class_ids

if __name__ == "__main__":
    detector = YOLOv5RKNN(
        model_path="rel_model.rknn",
        target_size=640,
        conf_threshold=0.65,
        nms_threshold=0.45
    )

    # 测试图像
    img_path = "test.jpg"
    img = cv2.imread(img_path)
    if img is None:
        print(f"Error: Unable to load image at {img_path}")
        exit(1)
    
    # 执行推理
    bboxes, scores, class_ids = detector.inference(img)
    
    print(f"Detected {len(bboxes)} objects")
    for i, (bb, sc, cid) in enumerate(zip(bboxes, scores, class_ids)):
        print(f'  Det {i}: cls={detector.class_names[cid]} score={sc:.3f} box={bb.round(1)}')
    
    # 可视化并保存结果
    if len(bboxes) > 0:
        vis = detector.draw_detections(img, bboxes, scores, class_ids)
        save_path = "result_fixed.jpg"
        cv2.imwrite(save_path, vis)
        print(f'[Save] Results saved to {save_path}')