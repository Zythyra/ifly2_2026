#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import time
from rknnlite.api import RKNNLite

class NanoDetRKNN:
    def __init__(self,
                 model_path,
                 target_size=320,
                 conf_threshold=0.65,
                 nms_threshold=0.45,
                 reg_max=7,
                 pre_nms_topk=1000):
        """
        pre_nms_topk: 进入 NMS 前按置信度保留的最大候选数（所有类别合计），防止无谓计算
        """
        self.target_size = target_size
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        self.reg_max = reg_max
        self.pre_nms_topk = pre_nms_topk

        self.class_names = ['Red', 'Green']
        self.num_classes = len(self.class_names)

        # 与训练时一致的 stride
        self.strides = [8, 16, 32, 64]

        self.rknn = self.init_model(model_path)
        self.center_priors = self.generate_center_priors()  # shape: (N, 3) -> (cx, cy, stride)

    # ----------------- 模型初始化 -----------------
    def init_model(self, model_path):
        rknn = RKNNLite()
        ret = rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError('Load RKNN model failed!')
        ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        if ret != 0:
            raise RuntimeError('Init RKNN runtime failed!')
        print('[RKNN] model loaded successfully!')
        return rknn

    # ----------------- 生成 center priors -----------------
    def generate_center_priors(self):
        """
        生成所有特征层的网格点中心及对应 stride
        返回 shape (sum(hw), 3): (cx, cy, stride)
        """
        priors = []
        for stride in self.strides:
            feat_h = self.target_size // stride
            feat_w = self.target_size // stride
            for iy in range(feat_h):
                cy = iy * stride + stride * 0.5
                for ix in range(feat_w):
                    cx = ix * stride + stride * 0.5
                    priors.append([cx, cy, stride])
        priors = np.array(priors, dtype=np.float32)
        print(f'[Init] Total priors: {len(priors)}')
        return priors  # (N,3)

    # ----------------- 预处理 -----------------
    def preprocess(self, img):
        h, w = img.shape[:2]
        scale = min(self.target_size / w, self.target_size / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        start_x = (self.target_size - new_w) // 2
        start_y = (self.target_size - new_h) // 2
        resized = cv2.resize(img, (new_w, new_h))
        canvas = np.full((self.target_size, self.target_size, 3), 114, dtype=np.uint8)
        canvas[start_y:start_y + new_h, start_x:start_x + new_w] = resized
        return canvas, scale, start_x, start_y

    # ----------------- 激活函数 -----------------
    @staticmethod
    def sigmoid(x):
        return 1 / (1 + np.exp(-x))

    # ----------------- 解码距离分布 -----------------
    def decode_distance_predictions(self, dist_raw):
        """
        dist_raw: (N, 4, reg_max+1)
        返回: (N,4) 每个边的“离散期望值”, 还未乘 stride
        """
        # softmax
        dist = dist_raw - np.max(dist_raw, axis=-1, keepdims=True)
        dist = np.exp(dist)
        dist = dist / np.sum(dist, axis=-1, keepdims=True)
        project = np.arange(self.reg_max + 1, dtype=np.float32)
        dist = np.sum(dist * project, axis=-1)  # (N,4)
        return dist

    # ----------------- 将距离转换为 bbox -----------------
    def distance2bbox(self, priors_xy, distances):
        """
        priors_xy: (N,2) (cx,cy)
        distances: (N,4) 已经乘过 stride 的 L,T,R,B
        输出: (N,4) x1,y1,x2,y2
        """
        x1 = priors_xy[:, 0] - distances[:, 0]
        y1 = priors_xy[:, 1] - distances[:, 1]
        x2 = priors_xy[:, 0] + distances[:, 2]
        y2 = priors_xy[:, 1] + distances[:, 3]
        return np.stack([x1, y1, x2, y2], axis=-1)

    # ----------------- 计算 IoU 向量化 -----------------
    @staticmethod
    def bbox_iou(box, boxes):
        """
        box: (4,)  boxes: (M,4)
        """
        xx1 = np.maximum(box[0], boxes[:, 0])
        yy1 = np.maximum(box[1], boxes[:, 1])
        xx2 = np.minimum(box[2], boxes[:, 2])
        yy2 = np.minimum(box[3], boxes[:, 3])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h

        area1 = (box[2] - box[0]) * (box[3] - box[1])
        area2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        union = area1 + area2 - inter
        return inter / np.maximum(union, 1e-6)

    # ----------------- 按类别的 NMS -----------------
    def multi_class_nms(self, boxes, scores, class_ids, iou_thr):
        """
        boxes: (N,4)
        scores: (N,)
        class_ids: (N,)
        """
        keep_indices = []
        for c in np.unique(class_ids):
            cls_mask = (class_ids == c)
            cls_boxes = boxes[cls_mask]
            cls_scores = scores[cls_mask]
            order = np.argsort(cls_scores)[::-1]

            while order.size > 0:
                i = order[0]
                keep_indices.append(np.where(cls_mask)[0][i])
                if order.size == 1:
                    break
                ious = self.bbox_iou(cls_boxes[i], cls_boxes[order[1:]])
                remain = np.where(ious < iou_thr)[0]
                order = order[remain + 1]
        return keep_indices

    # ----------------- 后处理 -----------------
    def postprocess(self, outputs, scale, start_x, start_y):
        """
        outputs: RKNN inference 返回 list，假设只有一个输出:
                 shape (N, num_classes + 4*(reg_max+1))
        """
        out = outputs[0]
        out = np.squeeze(out)  # (N, C + 4*(reg_max+1))
        if out.ndim != 2:
            raise ValueError(f'Unexpected output shape: {out.shape}')

        # 拆分分类与距离分布
        cls_raw = out[:, :self.num_classes]                # (N, num_classes)
        dist_raw = out[:, self.num_classes:]               # (N, 4*(reg_max+1))
        dist_raw = dist_raw.reshape(-1, 4, self.reg_max + 1)

        cls_scores = self.sigmoid(cls_raw)                 # sigmoid 分类
        max_cls_scores = np.max(cls_scores, axis=1)
        label_ids = np.argmax(cls_scores, axis=1)

        # 先根据 conf_threshold 初步筛选
        valid_mask = max_cls_scores > self.conf_threshold
        # print(f'[Post] Raw candidates: {out.shape[0]}, passing conf({self.conf_threshold}): {np.sum(valid_mask)}')

        if not np.any(valid_mask):
            return np.empty((0, 4), dtype=np.float32), np.array([]), np.array([])

        priors_valid = self.center_priors[valid_mask]      # (M,3)
        cls_scores_valid = cls_scores[valid_mask]
        max_scores_valid = max_cls_scores[valid_mask]
        label_ids_valid = label_ids[valid_mask]
        dist_valid = dist_raw[valid_mask]

        # 预截断 TopK（按最大类别分数）
        if self.pre_nms_topk is not None and max_scores_valid.shape[0] > self.pre_nms_topk:
            topk_idx = np.argsort(max_scores_valid)[::-1][:self.pre_nms_topk]
            priors_valid = priors_valid[topk_idx]
            cls_scores_valid = cls_scores_valid[topk_idx]
            max_scores_valid = max_scores_valid[topk_idx]
            label_ids_valid = label_ids_valid[topk_idx]
            dist_valid = dist_valid[topk_idx]
            print(f'[Post] After topk({self.pre_nms_topk}) -> {len(topk_idx)}')

        # 解码 LTRB (未乘 stride)
        dist_decoded = self.decode_distance_predictions(dist_valid)  # (M,4)
        # 乘对应 stride
        strides = priors_valid[:, 2:3]  # (M,1)
        dist_decoded *= strides

        # 计算 bbox
        bboxes = self.distance2bbox(priors_valid[:, :2], dist_decoded)  # (M,4) in letterbox space(320x320)

        # 反 letterbox: 去 pad / 缩放回原图
        # (注意：如果框超出边界可裁剪)
        bboxes[:, [0, 2]] = (bboxes[:, [0, 2]] - start_x) / scale
        bboxes[:, [1, 3]] = (bboxes[:, [1, 3]] - start_y) / scale

        # 裁剪到原图尺寸（这里不知道原图尺寸，预处理没保存，补一下）
        # 由于我们没有直接保存原图 h,w，这里在调用处决定是否裁剪。若需要请在 inference 里传 h,w 进来。
        # 简化：保留原值（多数情况下不会越界太多）

        # NMS
        keep = self.multi_class_nms(bboxes, max_scores_valid, label_ids_valid, self.nms_threshold)
        print(f'[Post] After NMS keep: {len(keep)}')

        return bboxes[keep], max_scores_valid[keep], label_ids_valid[keep]

    # ----------------- 可视化 -----------------
    def draw_detections(self, img, bboxes, scores, class_ids):
        out = img.copy()
        for box, sc, cid in zip(bboxes, scores, class_ids):
            x1, y1, x2, y2 = box.astype(int)
            x1 = max(0, x1); y1 = max(0, y1)
            x2 = min(img.shape[1]-1, x2); y2 = min(img.shape[0]-1, y2)
            label = f'{self.class_names[cid]} {sc:.2f}'
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(out, label, (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return out

    # ----------------- 推理 -----------------
    def inference(self, img):
        h, w = img.shape[:2]
        t0 = time.time()
        blob, scale, start_x, start_y = self.preprocess(img)
        # 如果模型要求 RGB 可在此转换：blob = cv2.cvtColor(blob, cv2.COLOR_BGR2RGB)

        outputs = self.rknn.inference([blob])
        bboxes, scores, class_ids = self.postprocess(outputs, scale, start_x, start_y)

        dt = time.time() - t0
        # print(f'[Perf] Inference + Postprocess time: {dt*1000:.2f} ms')
        for i, (bb, sc, cid) in enumerate(zip(bboxes, scores, class_ids)):
            print(f'  Det {i}: cls={self.class_names[cid]} score={sc:.3f} box={bb.round(1)}')
        return bboxes, scores, class_ids

    # ----------------- 输出调试 -----------------
    def debug_model_outputs(self, outputs):
        print('=== Debug Outputs ===')
        for i, o in enumerate(outputs):
            o = np.squeeze(o)
            print(f'Output {i} shape={o.shape} dtype={o.dtype} min={o.min():.4f} max={o.max():.4f} mean={o.mean():.4f}')
            cls_raw = o[:, :self.num_classes]
            dist_raw = o[:, self.num_classes:]
            print(f'  cls range: [{cls_raw.min():.4f}, {cls_raw.max():.4f}]')
            print(f'  dist range: [{dist_raw.min():.4f}, {dist_raw.max():.4f}]')

if __name__ == "__main__":
    detector = NanoDetRKNN(
        model_path="/home/ucar/ucar_ws_copy/src/nanodet/src/rgl_model.rknn",
        target_size=320,
        conf_threshold=0.65,      # 先保持，若框仍多可提到 0.75~0.8
        nms_threshold=0.45,
        reg_max=7,
        pre_nms_topk=1000
    )

    img = cv2.imread("/home/ucar/ucar_ws_copy/src/ucar_camera/photo/j1.jpg")
    bboxes, scores, class_ids = detector.inference(img)
    vis = detector.draw_detections(img, bboxes, scores, class_ids)
    cv2.imwrite("/home/ucar/ucar_ws_copy/src/nanodet/src/result_fixed.jpg", vis)
    print('[Save] result_fixed.jpg 已写出')
