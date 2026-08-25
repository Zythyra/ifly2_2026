#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# V5_YOLOV5_TRAFFIC_20260820：红绿灯后端替换为YOLOv5 RKNN。
# V6_TRAFFIC_GREEN_COLOR_GUARD_20260824：位置过滤后增加HSV绿色有效性和
# 红色占优过滤，阻止红灯被三分类模型高置信度误判为直行。
# -*- coding: utf-8 -*-

"""讯飞2026 RKNN统一ROS服务。

保留原有NanoDet文字框/OCR接口，红绿灯检测改用YOLOv5 RKNN。

detect_result_srv.detect_start 命令：
  -1：打开 /dev/video0
  -2：释放摄像头
  -3：丢弃两帧缓存

  其他普通值：文字框模型检测
   4：文字框低阈值检测

  -10：切换到红绿灯三分类模型
   10：红绿灯正常阈值检测
   14：红绿灯低阈值检测
  -11：切回文字框+OCR模型（测试/恢复用）

红绿灯类别固定：
  0：左转
  1：右转
  2：直行

当前YOLOv5交通灯RKNN模型要求：
  输入  640x640 RGB
  输出  [1, 25200, 8]
  类别  3
"""

import os
import sys
import threading
import time
import logging


# catkin_install_python 会生成由系统Python运行的包装器，从而绕过本文件的
# shebang。必须在导入 rospy、OpenCV 和 RKNNLite 之前切换到二代车现成的
# rknn_env，否则会出现 ModuleNotFoundError: No module named 'rknnlite'。
RKNN_PYTHON = "/home/ucar/miniforge3/envs/rknn_env/bin/python3"
if os.path.realpath(sys.executable) != os.path.realpath(RKNN_PYTHON):
    if not os.path.isfile(RKNN_PYTHON):
        raise RuntimeError("找不到RKNN Python解释器：{}".format(RKNN_PYTHON))
    source_script = os.path.realpath(__file__)
    os.execv(
        RKNN_PYTHON,
        [RKNN_PYTHON, source_script] + sys.argv[1:],
    )

import cv2
import numpy as np
import rospy
from rknnlite.api import RKNNLite
from sensor_msgs.msg import Image

from ros_nanodet.srv import detect_result_srv
from ros_nanodet.srv import detect_result_srvResponse
from ros_nanodet.srv import ocr_result_srv
from ros_nanodet.srv import ocr_result_srvResponse


# RKNNLite 1.4.0 会把标准 logging 的双向级别映射替换成
# C/E/W/I/D。ROS 一方面需要把 "DEBUG" 等名称解析为数值，另一方面
# rosgraph handler 又要求 LogRecord.levelname 是 "INFO" 等完整名称。
# logging.addLevelName() 会同时恢复“数值 -> 名称”和“名称 -> 数值”。
def restore_python_logging_levels():
    standard_levels = (
        (logging.NOTSET, "NOTSET"),
        (logging.DEBUG, "DEBUG"),
        (logging.INFO, "INFO"),
        (logging.WARNING, "WARNING"),
        (logging.ERROR, "ERROR"),
        (logging.CRITICAL, "CRITICAL"),
    )
    for level_value, level_name in standard_levels:
        logging.addLevelName(level_value, level_name)

    # ROS/Python 配置中仍可能使用这两个标准别名。只补名称到数值的映射，
    # 不覆盖 WARNING 和 CRITICAL 的规范显示名称。
    logging._nameToLevel["WARN"] = logging.WARNING
    logging._nameToLevel["FATAL"] = logging.CRITICAL


restore_python_logging_levels()


DEFAULT_MODEL_PATH = (
    "/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/models/"
    "nanodet_textbox/target_textbox_fp16.rknn"
)
DEFAULT_TRAFFIC_MODEL_PATH = (
    "/home/ucar/ucar_ws_copy/src/ros_nanodet/src/nanodet-0.2.0/"
    "workspace/model2026/best_yolov5_640_fp16.rknn"
)
DEFAULT_VIDEO_PATH = (
    "/home/ucar/ucar_ws_copy/src/ucarmain2026/"
    "nanodet_debug/nanodet_textbox.avi"
)
DEFAULT_OCR_MODEL_PATH = (
    "/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/rknn_model_zoo/"
    "examples/PPOCR/PPOCR-Rec/model/ppocrv4_rec_fp16.rknn"
)
DEFAULT_OCR_DICT_PATH = (
    "/home/ucar/ucar_ws_copy/src/ros_nanodet/ocr/rknn_model_zoo/"
    "examples/PPOCR/PPOCR-Rec/model/ppocr_keys_v1.txt"
)


def sigmoid(values):
    values = np.clip(values.astype(np.float32), -50.0, 50.0)
    return 1.0 / (1.0 + np.exp(-values))


def softmax(values, axis=-1):
    values = values.astype(np.float32)
    values = values - np.max(values, axis=axis, keepdims=True)
    exp_values = np.exp(values)
    denominator = np.maximum(
        np.sum(exp_values, axis=axis, keepdims=True),
        1e-12,
    )
    return exp_values / denominator


def make_resize_matrix(raw_width, raw_height, dst_width, dst_height):
    """复现 NanoDet 1.0 keep_ratio=True 的居中缩放矩阵。"""
    ratio = min(
        float(dst_width) / float(raw_width),
        float(dst_height) / float(raw_height),
    )

    center = np.eye(3, dtype=np.float32)
    center[0, 2] = -float(raw_width) / 2.0
    center[1, 2] = -float(raw_height) / 2.0

    resize = np.eye(3, dtype=np.float32)
    resize[0, 0] = ratio
    resize[1, 1] = ratio

    translate = np.eye(3, dtype=np.float32)
    translate[0, 2] = float(dst_width) / 2.0
    translate[1, 2] = float(dst_height) / 2.0

    return np.matmul(translate, np.matmul(resize, center))


def restore_boxes(boxes, inverse_matrix, raw_width, raw_height):
    """把模型输入坐标系中的框映射回原始相机图像。"""
    if boxes.size == 0:
        return boxes.astype(np.float32)

    count = boxes.shape[0]
    points = np.ones((count * 4, 3), dtype=np.float32)
    points[:, :2] = boxes[:, [0, 1, 2, 3, 0, 3, 2, 1]].reshape(-1, 2)
    points = np.matmul(points, inverse_matrix.T)
    points = points[:, :2] / np.maximum(points[:, 2:3], 1e-12)
    points = points.reshape(count, 8)

    x_values = points[:, [0, 2, 4, 6]]
    y_values = points[:, [1, 3, 5, 7]]
    restored = np.stack(
        [
            x_values.min(axis=1),
            y_values.min(axis=1),
            x_values.max(axis=1),
            y_values.max(axis=1),
        ],
        axis=1,
    )

    restored[:, [0, 2]] = restored[:, [0, 2]].clip(0, raw_width - 1)
    restored[:, [1, 3]] = restored[:, [1, 3]].clip(0, raw_height - 1)
    return restored.astype(np.float32)


def nms(boxes, scores, iou_threshold=0.6, max_count=100):
    """单类别 NMS。"""
    if boxes.size == 0:
        return np.empty((0,), dtype=np.int32)

    x0 = boxes[:, 0]
    y0 = boxes[:, 1]
    x1 = boxes[:, 2]
    y1 = boxes[:, 3]
    areas = np.maximum(0.0, x1 - x0) * np.maximum(0.0, y1 - y0)
    order = np.argsort(scores)[::-1]
    keep = []

    while order.size > 0 and len(keep) < max_count:
        index = int(order[0])
        keep.append(index)
        if order.size == 1:
            break

        remaining = order[1:]
        xx0 = np.maximum(x0[index], x0[remaining])
        yy0 = np.maximum(y0[index], y0[remaining])
        xx1 = np.minimum(x1[index], x1[remaining])
        yy1 = np.minimum(y1[index], y1[remaining])

        width = np.maximum(0.0, xx1 - xx0)
        height = np.maximum(0.0, yy1 - yy0)
        intersection = width * height
        union = areas[index] + areas[remaining] - intersection
        iou = intersection / np.maximum(union, 1e-12)
        order = remaining[iou <= iou_threshold]

    return np.asarray(keep, dtype=np.int32)


class NanoDetPlusRKNN(object):
    """NanoDet-Plus 1.0 RKNN通用多类别推理及后处理。"""

    def __init__(
        self,
        model_path,
        num_classes=1,
        class_names=None,
        input_size=320,
        strides=(8, 16, 32, 64),
        reg_max=7,
        pre_nms_topk=1000,
        nms_threshold=0.6,
    ):
        self.model_path = model_path
        self.input_size = int(input_size)
        self.strides = tuple(int(value) for value in strides)
        self.reg_max = int(reg_max)
        self.num_classes = int(num_classes)

        if self.num_classes < 1:
            raise RuntimeError(
                "NanoDet类别数量非法：{}".format(
                    self.num_classes
                )
            )

        if class_names is None:
            self.class_names = [
                "class_{}".format(index)
                for index in range(self.num_classes)
            ]
        else:
            self.class_names = list(class_names)

        if len(self.class_names) != self.num_classes:
            raise RuntimeError(
                "class_names数量={}与num_classes={}不一致".format(
                    len(self.class_names),
                    self.num_classes,
                )
            )

        self.output_channels = (
            self.num_classes
            + 4 * (self.reg_max + 1)
        )
        self.pre_nms_topk = int(pre_nms_topk)
        self.nms_threshold = float(nms_threshold)
        self.project = np.arange(
            self.reg_max + 1,
            dtype=np.float32,
        )
        self.center_priors = self._make_center_priors()
        self.expected_points = self.center_priors.shape[0]
        self.rknn = None
        self._load_model()

    def _make_center_priors(self):
        priors = []

        for stride in self.strides:
            feature_width = self.input_size // stride
            feature_height = self.input_size // stride

            grid_y, grid_x = np.meshgrid(
                np.arange(
                    feature_height,
                    dtype=np.float32,
                ),
                np.arange(
                    feature_width,
                    dtype=np.float32,
                ),
                indexing="ij",
            )

            center_x = (
                grid_x.reshape(-1)
                * float(stride)
            )
            center_y = (
                grid_y.reshape(-1)
                * float(stride)
            )

            stride_values = np.full_like(
                center_x,
                float(stride),
            )

            priors.append(
                np.stack(
                    [
                        center_x,
                        center_y,
                        stride_values,
                    ],
                    axis=1,
                )
            )

        return np.concatenate(
            priors,
            axis=0,
        ).astype(np.float32)

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            raise RuntimeError(
                "找不到RKNN模型：{}".format(
                    self.model_path
                )
            )

        self.rknn = RKNNLite()

        result = self.rknn.load_rknn(
            self.model_path
        )

        if result != 0:
            raise RuntimeError(
                "加载RKNN模型失败，返回值={}，路径={}".format(
                    result,
                    self.model_path,
                )
            )

        core_mask = getattr(
            RKNNLite,
            "NPU_CORE_0_1_2",
            None,
        )

        if core_mask is None:
            result = self.rknn.init_runtime()
        else:
            result = self.rknn.init_runtime(
                core_mask=core_mask
            )

        if result != 0:
            raise RuntimeError(
                "初始化RKNN Runtime失败，返回值={}".format(
                    result
                )
            )

        restore_python_logging_levels()

        rospy.loginfo(
            "RKNN NanoDet模型加载成功：%s",
            self.model_path,
        )
        rospy.loginfo(
            "模型后处理：input=%d，points=%d，"
            "classes=%d，channels=%d，names=%s",
            self.input_size,
            self.expected_points,
            self.num_classes,
            self.output_channels,
            str(self.class_names),
        )

    def _normalize_output(self, output):
        values = np.asarray(output)
        raw_shape = values.shape
        values = np.squeeze(values)

        if values.ndim == 1:
            expected_size = (
                self.expected_points
                * self.output_channels
            )

            if values.size != expected_size:
                raise RuntimeError(
                    "RKNN输出元素数错误：shape={}，"
                    "size={}，期望={}".format(
                        raw_shape,
                        values.size,
                        expected_size,
                    )
                )

            values = values.reshape(
                self.expected_points,
                self.output_channels,
            )

        elif values.ndim == 2:
            if values.shape == (
                self.output_channels,
                self.expected_points,
            ):
                values = values.T

            elif values.shape != (
                self.expected_points,
                self.output_channels,
            ):
                raise RuntimeError(
                    "RKNN输出shape错误：原始={}，"
                    "压缩后={}，期望=({}, {})".format(
                        raw_shape,
                        values.shape,
                        self.expected_points,
                        self.output_channels,
                    )
                )
        else:
            raise RuntimeError(
                "RKNN输出维度错误：原始={}，压缩后={}".format(
                    raw_shape,
                    values.shape,
                )
            )

        return (
            values.astype(np.float32),
            raw_shape,
        )

    def _decode(self, output, score_threshold):
        predictions, raw_shape = (
            self._normalize_output(output)
        )

        class_values = predictions[
            :,
            :self.num_classes
        ]

        # NanoDet导出可能已经包含sigmoid。
        if (
            class_values.min() >= 0.0
            and class_values.max() <= 1.0
        ):
            class_probabilities = class_values
        else:
            class_probabilities = sigmoid(
                class_values
            )

        class_ids = np.argmax(
            class_probabilities,
            axis=1,
        ).astype(np.int32)

        class_scores = np.max(
            class_probabilities,
            axis=1,
        )

        valid = np.where(
            class_scores
            >= float(score_threshold)
        )[0]

        if valid.size == 0:
            return (
                np.empty(
                    (0, 4),
                    dtype=np.float32,
                ),
                np.empty(
                    (0,),
                    dtype=np.float32,
                ),
                np.empty(
                    (0,),
                    dtype=np.int32,
                ),
                raw_shape,
            )

        if (
            self.pre_nms_topk > 0
            and valid.size > self.pre_nms_topk
        ):
            valid_scores = class_scores[
                valid
            ]

            order = np.argsort(
                valid_scores
            )[::-1][
                :self.pre_nms_topk
            ]

            valid = valid[
                order
            ]

        priors = self.center_priors[
            valid
        ]

        scores = class_scores[
            valid
        ]

        candidate_classes = class_ids[
            valid
        ]

        distance_logits = predictions[
            valid,
            self.num_classes:
        ]

        distance_logits = (
            distance_logits.reshape(
                -1,
                4,
                self.reg_max + 1,
            )
        )

        distance_distribution = softmax(
            distance_logits,
            axis=2,
        )

        distances = np.sum(
            distance_distribution
            * self.project.reshape(
                1,
                1,
                -1,
            ),
            axis=2,
        )

        distances = (
            distances
            * priors[:, 2:3]
        )

        boxes = np.stack(
            [
                priors[:, 0]
                - distances[:, 0],

                priors[:, 1]
                - distances[:, 1],

                priors[:, 0]
                + distances[:, 2],

                priors[:, 1]
                + distances[:, 3],
            ],
            axis=1,
        )

        boxes[:, [0, 2]] = (
            boxes[:, [0, 2]]
            .clip(
                0,
                self.input_size - 1,
            )
        )

        boxes[:, [1, 3]] = (
            boxes[:, [1, 3]]
            .clip(
                0,
                self.input_size - 1,
            )
        )

        # 多类别NMS必须按类别分别执行，
        # 避免不同类别的重叠候选互相压掉。
        keep_indices = []

        for class_id in np.unique(
            candidate_classes
        ):
            class_positions = np.where(
                candidate_classes
                == class_id
            )[0]

            local_keep = nms(
                boxes[class_positions],
                scores[class_positions],
                iou_threshold=
                    self.nms_threshold,
                max_count=100,
            )

            keep_indices.extend(
                class_positions[
                    local_keep
                ].tolist()
            )

        if not keep_indices:
            return (
                np.empty(
                    (0, 4),
                    dtype=np.float32,
                ),
                np.empty(
                    (0,),
                    dtype=np.float32,
                ),
                np.empty(
                    (0,),
                    dtype=np.int32,
                ),
                raw_shape,
            )

        keep_indices = np.asarray(
            keep_indices,
            dtype=np.int32,
        )

        # 最终统一按置信度从高到低排序。
        # traffic_light.cpp会优先使用第一个有效候选。
        global_order = np.argsort(
            scores[keep_indices]
        )[::-1]

        keep_indices = keep_indices[
            global_order
        ]

        return (
            boxes[keep_indices],
            scores[keep_indices],
            candidate_classes[
                keep_indices
            ],
            raw_shape,
        )

    def infer(
        self,
        frame,
        score_threshold
    ):
        raw_height, raw_width = (
            frame.shape[:2]
        )

        resize_matrix = (
            make_resize_matrix(
                raw_width,
                raw_height,
                self.input_size,
                self.input_size,
            )
        )

        model_input = cv2.warpPerspective(
            frame,
            resize_matrix,
            (
                self.input_size,
                self.input_size,
            ),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        try:
            outputs = self.rknn.inference(
                inputs=[model_input],
                data_format=["nhwc"],
            )
        except TypeError:
            outputs = self.rknn.inference(
                inputs=[model_input]
            )

        if (
            outputs is None
            or len(outputs) != 1
        ):
            raise RuntimeError(
                "RKNN输出数量错误：{}".format(
                    None
                    if outputs is None
                    else len(outputs)
                )
            )

        (
            boxes,
            scores,
            class_ids,
            raw_shape,
        ) = self._decode(
            outputs[0],
            score_threshold,
        )

        if boxes.size != 0:
            inverse_matrix = np.linalg.inv(
                resize_matrix
            )

            boxes = restore_boxes(
                boxes,
                inverse_matrix,
                raw_width,
                raw_height,
            )

        return (
            boxes,
            scores,
            class_ids,
            raw_shape,
        )

    def warmup(self, count=3):
        image = np.zeros(
            (
                self.input_size,
                self.input_size,
                3,
            ),
            dtype=np.uint8,
        )

        for _ in range(
            max(
                0,
                int(count)
            )
        ):
            self.infer(
                image,
                0.99
            )

        rospy.loginfo(
            "RKNN模型预热完成，共%d次",
            max(
                0,
                int(count)
            ),
        )

    def release(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None


class YOLOv5RKNN(object):
    """已验证的640输入、单输出YOLOv5 RKNN推理与后处理。"""

    CLASS_NAMES = ("left", "right", "straight")

    def __init__(
        self,
        model_path,
        input_size=640,
        nms_threshold=0.45,
    ):
        self.model_path = model_path
        self.input_size = int(input_size)
        self.expected_points = 25200
        self.output_channels = 8
        self.nms_threshold = float(nms_threshold)
        self.rknn = None
        self._load_model()

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            raise RuntimeError(
                "找不到YOLOv5 RKNN模型：{}".format(
                    self.model_path
                )
            )

        self.rknn = RKNNLite()
        result = self.rknn.load_rknn(self.model_path)

        if result not in (0, None):
            raise RuntimeError(
                "加载YOLOv5 RKNN失败，返回值={}".format(result)
            )

        core_mask = getattr(
            RKNNLite,
            "NPU_CORE_0_1_2",
            None,
        )

        if core_mask is None:
            result = self.rknn.init_runtime()
        else:
            result = self.rknn.init_runtime(
                core_mask=core_mask
            )

        if result not in (0, None):
            self.release()
            raise RuntimeError(
                "初始化YOLOv5 RKNN Runtime失败，返回值={}".format(
                    result
                )
            )

        restore_python_logging_levels()
        rospy.loginfo(
            "YOLOv5红绿灯模型加载成功：%s",
            self.model_path,
        )
        rospy.loginfo(
            "YOLOv5接口：input=640x640 RGB，output=(1,25200,8)，"
            "class 0=左转，1=右转，2=直行"
        )

    def _letterbox(self, frame):
        raw_height, raw_width = frame.shape[:2]
        ratio = min(
            float(self.input_size) / float(raw_width),
            float(self.input_size) / float(raw_height),
        )
        resized_width = max(
            1,
            int(round(raw_width * ratio)),
        )
        resized_height = max(
            1,
            int(round(raw_height * ratio)),
        )
        resized = cv2.resize(
            frame,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )

        left = (self.input_size - resized_width) // 2
        top = (self.input_size - resized_height) // 2
        canvas = np.full(
            (self.input_size, self.input_size, 3),
            114,
            dtype=np.uint8,
        )
        canvas[
            top:top + resized_height,
            left:left + resized_width,
        ] = resized

        # 该模型转换时使用RGB输入；相机帧是OpenCV BGR。
        rgb = cv2.cvtColor(
            canvas,
            cv2.COLOR_BGR2RGB,
        )
        return (
            np.ascontiguousarray(rgb),
            ratio,
            left,
            top,
        )

    def _normalize_output(self, output):
        values = np.asarray(output)
        raw_shape = tuple(values.shape)
        values = np.squeeze(values)

        if values.ndim == 1:
            expected = (
                self.expected_points
                * self.output_channels
            )
            if values.size != expected:
                raise RuntimeError(
                    "YOLOv5输出元素数错误：shape={}，size={}，期望={}".format(
                        raw_shape,
                        values.size,
                        expected,
                    )
                )
            values = values.reshape(
                self.expected_points,
                self.output_channels,
            )
        elif values.ndim == 2:
            if values.shape == (
                self.output_channels,
                self.expected_points,
            ):
                values = values.T
            elif values.shape != (
                self.expected_points,
                self.output_channels,
            ):
                raise RuntimeError(
                    "YOLOv5输出shape错误：原始={}，处理后={}".format(
                        raw_shape,
                        tuple(values.shape),
                    )
                )
        else:
            raise RuntimeError(
                "YOLOv5输出维度错误：{}".format(raw_shape)
            )

        return values.astype(np.float32), raw_shape

    @staticmethod
    def _as_probability(values):
        if values.min() >= 0.0 and values.max() <= 1.0:
            return values.astype(np.float32)
        return sigmoid(values)

    @staticmethod
    def _empty_result(raw_shape):
        return (
            np.empty((0, 4), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
            np.empty((0,), dtype=np.int32),
            raw_shape,
        )

    def _decode(self, output, score_threshold):
        predictions, raw_shape = self._normalize_output(output)
        objectness = self._as_probability(predictions[:, 4])
        class_probabilities = self._as_probability(
            predictions[:, 5:8]
        )
        class_ids = np.argmax(
            class_probabilities,
            axis=1,
        ).astype(np.int32)
        scores = objectness * np.max(
            class_probabilities,
            axis=1,
        )
        valid = np.where(
            scores >= float(score_threshold)
        )[0]

        if valid.size == 0:
            return self._empty_result(raw_shape)

        if valid.size > 3000:
            valid = valid[
                np.argsort(scores[valid])[::-1][:3000]
            ]

        xywh = predictions[valid, :4]
        candidate_scores = scores[valid]
        candidate_classes = class_ids[valid]
        boxes = np.stack(
            (
                xywh[:, 0] - xywh[:, 2] * 0.5,
                xywh[:, 1] - xywh[:, 3] * 0.5,
                xywh[:, 0] + xywh[:, 2] * 0.5,
                xywh[:, 1] + xywh[:, 3] * 0.5,
            ),
            axis=1,
        ).astype(np.float32)
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(
            0,
            self.input_size - 1,
        )
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(
            0,
            self.input_size - 1,
        )

        valid_size = np.where(
            (boxes[:, 2] - boxes[:, 0] >= 1.0)
            & (boxes[:, 3] - boxes[:, 1] >= 1.0)
        )[0]
        boxes = boxes[valid_size]
        candidate_scores = candidate_scores[valid_size]
        candidate_classes = candidate_classes[valid_size]

        keep_indices = []
        for class_id in np.unique(candidate_classes):
            positions = np.where(
                candidate_classes == class_id
            )[0]
            local_keep = nms(
                boxes[positions],
                candidate_scores[positions],
                iou_threshold=self.nms_threshold,
                max_count=100,
            )
            keep_indices.extend(
                positions[local_keep].tolist()
            )

        if not keep_indices:
            return self._empty_result(raw_shape)

        keep_indices = np.asarray(
            keep_indices,
            dtype=np.int32,
        )
        keep_indices = keep_indices[
            np.argsort(candidate_scores[keep_indices])[::-1]
        ]
        return (
            boxes[keep_indices],
            candidate_scores[keep_indices],
            candidate_classes[keep_indices],
            raw_shape,
        )

    @staticmethod
    def _restore_boxes(
        boxes,
        ratio,
        left,
        top,
        width,
        height,
    ):
        if boxes.size == 0:
            return boxes

        restored = boxes.copy()
        restored[:, [0, 2]] = (
            restored[:, [0, 2]] - left
        ) / max(ratio, 1e-12)
        restored[:, [1, 3]] = (
            restored[:, [1, 3]] - top
        ) / max(ratio, 1e-12)
        restored[:, [0, 2]] = restored[:, [0, 2]].clip(
            0,
            width - 1,
        )
        restored[:, [1, 3]] = restored[:, [1, 3]].clip(
            0,
            height - 1,
        )
        return restored.astype(np.float32)

    def infer(self, frame, score_threshold):
        raw_height, raw_width = frame.shape[:2]
        model_input, ratio, left, top = self._letterbox(frame)

        try:
            outputs = self.rknn.inference(
                inputs=[model_input],
                data_format=["nhwc"],
            )
        except TypeError:
            outputs = self.rknn.inference(
                inputs=[model_input]
            )

        if outputs is None or len(outputs) != 1:
            raise RuntimeError(
                "YOLOv5 RKNN输出数量错误：{}".format(
                    None if outputs is None else len(outputs)
                )
            )

        boxes, scores, class_ids, raw_shape = self._decode(
            outputs[0],
            score_threshold,
        )
        boxes = self._restore_boxes(
            boxes,
            ratio,
            left,
            top,
            raw_width,
            raw_height,
        )
        return boxes, scores, class_ids, raw_shape

    def warmup(self, count=3):
        image = np.zeros(
            (self.input_size, self.input_size, 3),
            dtype=np.uint8,
        )
        for _ in range(max(0, int(count))):
            self.infer(image, 0.99)
        rospy.loginfo(
            "YOLOv5模型预热完成，共%d次",
            max(0, int(count)),
        )

    def release(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None


class PPOCRRecognizerRKNN(object):
    """压缩包中PP-OCRv4 Rec模型的RKNN推理和原始CTC解码。"""

    INPUT_HEIGHT = 48
    INPUT_WIDTH = 320

    def __init__(self, model_path, dictionary_path):
        self.model_path = model_path
        self.dictionary_path = dictionary_path
        self.rknn = None
        self.characters = self._load_dictionary()
        self._load_model()

    def _load_dictionary(self):
        if not os.path.isfile(self.dictionary_path):
            raise RuntimeError(
                "找不到PP-OCR字符字典：{}".format(self.dictionary_path)
            )

        with open(self.dictionary_path, "r", encoding="utf-8") as file:
            dictionary = file.read().splitlines()

        # 与压缩包中的CTCLabelDecode完全一致：0号为blank，字典末尾
        # 额外加入一个空格。6623个字典字符 + blank + space = 6625类。
        characters = ["blank"] + dictionary + [" "]
        if len(characters) != 6625:
            raise RuntimeError(
                "PP-OCR字符数量错误：实际={}，期望=6625，字典={}".format(
                    len(characters),
                    self.dictionary_path,
                )
            )
        return characters

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            raise RuntimeError("找不到PP-OCR RKNN模型：{}".format(self.model_path))

        self.rknn = RKNNLite()
        result = self.rknn.load_rknn(self.model_path)
        if result not in (0, None):
            self.rknn.release()
            self.rknn = None
            raise RuntimeError(
                "加载PP-OCR RKNN模型失败，返回值={}，路径={}".format(
                    result,
                    self.model_path,
                )
            )

        core_mask = getattr(RKNNLite, "NPU_CORE_AUTO", None)
        if core_mask is None:
            result = self.rknn.init_runtime()
        else:
            result = self.rknn.init_runtime(core_mask=core_mask)

        if result not in (0, None):
            self.rknn.release()
            self.rknn = None
            raise RuntimeError(
                "初始化PP-OCR RKNN Runtime失败，返回值={}".format(result)
            )

        # RKNNLite 1.4.0会修改Python logging的双向级别映射。
        restore_python_logging_levels()
        rospy.loginfo("PP-OCRv4文字识别模型加载成功：%s", self.model_path)
        rospy.loginfo(
            "PP-OCR参数：输入=%dx%d BGR，归一化=float32/255，字符数=%d",
            self.INPUT_WIDTH,
            self.INPUT_HEIGHT,
            len(self.characters),
        )

    @staticmethod
    def _preprocess(crop):
        if crop is None or crop.size == 0:
            raise RuntimeError("PP-OCR输入裁剪图为空")

        # 严格复现压缩包的已验证流程：BGR原图直接拉伸到320x48，
        # 不转RGB、不保持宽高比、不补边，随后转float32并除以255。
        resized = cv2.resize(
            crop,
            (PPOCRRecognizerRKNN.INPUT_WIDTH, PPOCRRecognizerRKNN.INPUT_HEIGHT),
            interpolation=cv2.INTER_LINEAR,
        )
        return resized.astype(np.float32) / 255.0

    def _normalize_output(self, output):
        predictions = np.asarray(output, dtype=np.float32)
        raw_shape = tuple(predictions.shape)
        predictions = np.squeeze(predictions)

        if predictions.ndim == 2:
            predictions = np.expand_dims(predictions, axis=0)

        if predictions.ndim != 3:
            raise RuntimeError(
                "PP-OCR输出维度错误：原始shape={}，处理后shape={}".format(
                    raw_shape,
                    tuple(predictions.shape),
                )
            )

        character_count = len(self.characters)
        if predictions.shape[2] != character_count:
            if predictions.shape[1] == character_count:
                predictions = predictions.transpose(0, 2, 1)
            else:
                raise RuntimeError(
                    "PP-OCR输出类别数错误：shape={}，字典类别数={}".format(
                        tuple(predictions.shape),
                        character_count,
                    )
                )

        return predictions, raw_shape

    def _ctc_decode(self, predictions):
        indices = predictions.argmax(axis=2)[0]
        probabilities = predictions.max(axis=2)[0]

        text_characters = []
        selected_probabilities = []
        previous_index = None

        for index, probability in zip(indices, probabilities):
            character_index = int(index)

            # CTC连续重复字符只保留第一个；blank会打断重复序列。
            if character_index == previous_index:
                continue
            previous_index = character_index

            if character_index == 0:
                continue

            text_characters.append(self.characters[character_index])
            selected_probabilities.append(float(probability))

        text = "".join(text_characters)
        confidence = (
            float(np.mean(selected_probabilities))
            if selected_probabilities
            else 0.0
        )
        return text, confidence

    def recognize(self, crop):
        model_input = self._preprocess(crop)
        outputs = self.rknn.inference(inputs=[model_input])
        if outputs is None or len(outputs) != 1:
            raise RuntimeError(
                "PP-OCR RKNN输出数量错误：{}".format(
                    None if outputs is None else len(outputs)
                )
            )

        predictions, raw_shape = self._normalize_output(outputs[0])
        text, confidence = self._ctc_decode(predictions)
        return text, confidence, raw_shape

    def warmup(self, count=1):
        image = np.zeros(
            (self.INPUT_HEIGHT, self.INPUT_WIDTH, 3),
            dtype=np.uint8,
        )
        for _ in range(max(0, int(count))):
            self.recognize(image)
        restore_python_logging_levels()
        rospy.loginfo("PP-OCR模型预热完成，共%d次", max(0, int(count)))

    def release(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None


class Detect2026Server(object):
    TRAFFIC_CLASS_NAMES = (
        "left",
        "right",
        "straight",
    )

    def __init__(self):
        self.model_path = rospy.get_param(
            "~model_path",
            DEFAULT_MODEL_PATH,
        )

        self.traffic_model_path = rospy.get_param(
            "~traffic_model_path",
            DEFAULT_TRAFFIC_MODEL_PATH,
        )

        self.ocr_model_path = rospy.get_param(
            "~ocr_model_path",
            DEFAULT_OCR_MODEL_PATH,
        )

        self.ocr_dict_path = rospy.get_param(
            "~ocr_dict_path",
            DEFAULT_OCR_DICT_PATH,
        )

        self.camera_device = rospy.get_param(
            "~camera_device",
            "/dev/video0",
        )

        self.score_threshold = float(
            rospy.get_param(
                "~score_threshold",
                0.50,
            )
        )

        self.low_score_threshold = float(
            rospy.get_param(
                "~low_score_threshold",
                0.45,
            )
        )

        self.traffic_score_threshold = float(
            rospy.get_param(
                "~traffic_score_threshold",
                0.25,
            )
        )

        self.traffic_low_score_threshold = float(
            rospy.get_param(
                "~traffic_low_score_threshold",
                0.15,
            )
        )

        self.nms_threshold = float(
            rospy.get_param(
                "~nms_threshold",
                0.60,
            )
        )

        self.traffic_nms_threshold = float(
            rospy.get_param(
                "~traffic_nms_threshold",
                0.45,
            )
        )

        # 红绿灯检测框位置过滤。这里的坐标比例基于YOLOv5检测框
        # 已经还原后的原始相机图像尺寸，而不是640x640模型输入尺寸。
        self.traffic_center_x_min_ratio = float(
            rospy.get_param(
                "~traffic_center_x_min_ratio",
                0.35,
            )
        )

        self.traffic_center_x_max_ratio = float(
            rospy.get_param(
                "~traffic_center_x_max_ratio",
                0.66,
            )
        )

        self.traffic_center_y_min_ratio = float(
            rospy.get_param(
                "~traffic_center_y_min_ratio",
                0.22,
            )
        )

        self.traffic_center_y_max_ratio = float(
            rospy.get_param(
                "~traffic_center_y_max_ratio",
                0.38,
            )
        )

        # 红绿灯模型只有左/右/直行三个绿色方向类别，没有红灯类别。
        # 因此YOLO高分框还必须通过框内颜色检查，红灯框才不会被强制
        # 分到最相似的straight类别后返回给总控。
        self.traffic_green_color_filter_enabled = bool(
            rospy.get_param(
                "~traffic_green_color_filter_enabled",
                True,
            )
        )
        self.traffic_color_roi_inset_ratio = float(
            rospy.get_param(
                "~traffic_color_roi_inset_ratio",
                0.08,
            )
        )
        self.traffic_green_h_min = int(
            rospy.get_param("~traffic_green_h_min", 30)
        )
        self.traffic_green_h_max = int(
            rospy.get_param("~traffic_green_h_max", 100)
        )
        self.traffic_red_h_low_max = int(
            rospy.get_param("~traffic_red_h_low_max", 15)
        )
        self.traffic_red_h_high_min = int(
            rospy.get_param("~traffic_red_h_high_min", 165)
        )
        self.traffic_color_s_min = int(
            rospy.get_param("~traffic_color_s_min", 55)
        )
        self.traffic_color_v_min = int(
            rospy.get_param("~traffic_color_v_min", 45)
        )
        self.traffic_green_min_ratio = float(
            rospy.get_param("~traffic_green_min_ratio", 0.010)
        )
        self.traffic_red_min_ratio = float(
            rospy.get_param("~traffic_red_min_ratio", 0.020)
        )
        self.traffic_red_to_green_max_ratio = float(
            rospy.get_param(
                "~traffic_red_to_green_max_ratio",
                1.0,
            )
        )

        self.traffic_color_roi_inset_ratio = min(
            0.40,
            max(0.0, self.traffic_color_roi_inset_ratio),
        )
        self.traffic_green_h_min = min(
            179, max(0, self.traffic_green_h_min)
        )
        self.traffic_green_h_max = min(
            179,
            max(self.traffic_green_h_min, self.traffic_green_h_max),
        )
        self.traffic_red_h_low_max = min(
            179, max(0, self.traffic_red_h_low_max)
        )
        self.traffic_red_h_high_min = min(
            179, max(0, self.traffic_red_h_high_min)
        )
        self.traffic_color_s_min = min(
            255, max(0, self.traffic_color_s_min)
        )
        self.traffic_color_v_min = min(
            255, max(0, self.traffic_color_v_min)
        )
        self.traffic_green_min_ratio = min(
            1.0, max(0.0, self.traffic_green_min_ratio)
        )
        self.traffic_red_min_ratio = min(
            1.0, max(0.0, self.traffic_red_min_ratio)
        )
        self.traffic_red_to_green_max_ratio = max(
            0.0, self.traffic_red_to_green_max_ratio
        )

        self.flip_horizontal = bool(
            rospy.get_param(
                "~flip_horizontal",
                True,
            )
        )

        self.debug_video_path = rospy.get_param(
            "~debug_video_path",
            DEFAULT_VIDEO_PATH,
        )

        self.debug_video_fps = float(
            rospy.get_param(
                "~debug_video_fps",
                3.0,
            )
        )

        self.warmup_count = int(
            rospy.get_param(
                "~warmup_count",
                3,
            )
        )

        self.traffic_warmup_count = int(
            rospy.get_param(
                "~traffic_warmup_count",
                3,
            )
        )

        self.ocr_warmup_count = int(
            rospy.get_param(
                "~ocr_warmup_count",
                1,
            )
        )

        self.enable_debug_image = bool(
            rospy.get_param(
                "~enable_debug_image",
                False,
            )
        )

        self.enable_debug_video = bool(
            rospy.get_param(
                "~enable_debug_video",
                False,
            )
        )

        # 正式运行默认关闭HighGUI；调试时可在launch中显式开启。
        self.show_live_window = bool(
            rospy.get_param(
                "~show_live_window",
                False,
            )
        )

        self.live_window_name = str(
            rospy.get_param(
                "~live_window_name",
                "traffic_light_live",
            )
        )

        self.startup_mode = str(
            rospy.get_param(
                "~startup_mode",
                "text",
            )
        ).strip().lower()

        if self.startup_mode not in (
            "text",
            "traffic",
        ):
            rospy.logwarn(
                "startup_mode=%s非法，使用text",
                self.startup_mode,
            )
            self.startup_mode = "text"

        self.lock = threading.Lock()

        # V4：HighGUI全部由Python主线程执行，Service回调只更新最新帧。
        self.live_frame_lock = threading.Lock()
        self.latest_live_frame = None
        self.live_window_open = False
        self.live_window_should_close = False

        self.detector = None
        self.ocr_recognizer = None
        self.traffic_detector = None

        self.current_mode = None

        self.cap = None
        self.camera_active = False

        self.video_writer = None
        self.output_shape_logged = {
            "text": False,
            "traffic": False,
        }

        # 分阶段加载：
        # 正式整合时默认text，完成巡检后再command=-10切traffic；
        # 单独测试traffic_light.cpp时可在launch里startup_mode=traffic，
        # 避免先加载文字/OCR再释放。
        if self.startup_mode == "traffic":
            if not self.switch_to_traffic_mode():
                raise RuntimeError(
                    "启动时加载红绿灯模型失败"
                )
        else:
            if not self.switch_to_text_mode():
                raise RuntimeError(
                    "启动时加载文字/OCR模型失败"
                )

        self.service = rospy.Service(
            "nanodet_detect",
            detect_result_srv,
            self.handle_request,
        )

        self.ocr_service = rospy.Service(
            "nanodet_ocr",
            ocr_result_srv,
            self.handle_ocr_request,
        )

        self.debug_image_publisher = None

        if self.enable_debug_image:
            self.debug_image_publisher = rospy.Publisher(
                "/nanodet/debug_image",
                Image,
                queue_size=1,
            )

        rospy.on_shutdown(
            self.shutdown
        )

        rospy.loginfo(
            "RKNN统一服务已就绪（文字NanoDet/OCR，红绿灯YOLOv5）："
            "/nanodet_detect、/nanodet_ocr，"
            "当前模式=%s，图像发布=%s，视频录制=%s，实时窗口=%s",
            self.current_mode,
            self.enable_debug_image,
            self.enable_debug_video,
            self.show_live_window,
        )

        rospy.loginfo(
            "红绿灯检测框中心位置过滤："
            "X比例=[%.3f, %.3f]，Y比例=[%.3f, %.3f]",
            self.traffic_center_x_min_ratio,
            self.traffic_center_x_max_ratio,
            self.traffic_center_y_min_ratio,
            self.traffic_center_y_max_ratio,
        )

        rospy.loginfo(
            "红绿灯HSV绿色保护：启用=%s，绿色H=[%d,%d]，"
            "S>=%d，V>=%d，绿色最小占比=%.3f；"
            "红色占比>=%.3f且红/绿>%.2f时剔除",
            self.traffic_green_color_filter_enabled,
            self.traffic_green_h_min,
            self.traffic_green_h_max,
            self.traffic_color_s_min,
            self.traffic_color_v_min,
            self.traffic_green_min_ratio,
            self.traffic_red_min_ratio,
            self.traffic_red_to_green_max_ratio,
        )

    @staticmethod
    def make_control_response(
        value=-1
    ):
        response = detect_result_srvResponse()

        response.class_name.append(
            int(value)
        )
        response.x0.append(-1)
        response.y0.append(-1)
        response.x1.append(-1)
        response.y1.append(-1)

        return response

    def release_text_models(self):
        if self.detector is not None:
            self.detector.release()
            self.detector = None

        if self.ocr_recognizer is not None:
            self.ocr_recognizer.release()
            self.ocr_recognizer = None

    def release_traffic_model(self):
        if self.traffic_detector is not None:
            self.traffic_detector.release()
            self.traffic_detector = None

    def switch_to_text_mode(self):
        if (
            self.current_mode == "text"
            and self.detector is not None
            and self.ocr_recognizer is not None
        ):
            rospy.loginfo(
                "已经处于文字框+OCR模式"
            )
            return True

        rospy.logwarn(
            "切换到文字框+OCR模型..."
        )

        self.release_traffic_model()

        try:
            self.detector = NanoDetPlusRKNN(
                model_path=self.model_path,
                num_classes=1,
                class_names=("target",),
                input_size=320,
                strides=(8, 16, 32, 64),
                reg_max=7,
                pre_nms_topk=1000,
                nms_threshold=
                    self.nms_threshold,
            )

            self.detector.warmup(
                self.warmup_count
            )

            self.ocr_recognizer = (
                PPOCRRecognizerRKNN(
                    model_path=
                        self.ocr_model_path,
                    dictionary_path=
                        self.ocr_dict_path,
                )
            )

            self.ocr_recognizer.warmup(
                self.ocr_warmup_count
            )

        except Exception as error:
            self.release_text_models()
            self.current_mode = None

            rospy.logerr(
                "加载文字/OCR模型失败：%s",
                error,
            )
            return False

        self.current_mode = "text"
        self.output_shape_logged["text"] = False

        restore_python_logging_levels()

        rospy.loginfo(
            "已切换到文字框+OCR模式"
        )

        return True

    def switch_to_traffic_mode(self):
        if (
            self.current_mode == "traffic"
            and self.traffic_detector is not None
        ):
            rospy.loginfo(
                "已经处于红绿灯三分类模式"
            )
            return True

        rospy.logwarn(
            "切换到YOLOv5红绿灯三分类RKNN模型..."
        )

        # 为降低NPU/内存占用，正式切换后释放文字框和OCR模型。
        self.release_text_models()

        try:
            self.traffic_detector = YOLOv5RKNN(
                model_path=
                    self.traffic_model_path,
                input_size=640,
                nms_threshold=
                    self.traffic_nms_threshold,
            )

            self.traffic_detector.warmup(
                self.traffic_warmup_count
            )

        except Exception as error:
            self.release_traffic_model()
            self.current_mode = None

            rospy.logerr(
                "加载YOLOv5红绿灯RKNN模型失败：%s",
                error,
            )
            return False

        self.current_mode = "traffic"
        self.output_shape_logged["traffic"] = False

        restore_python_logging_levels()

        rospy.loginfo(
            "YOLOv5红绿灯三分类模型切换完成："
            "class 0=左转，1=右转，2=直行"
        )

        return True

    def open_camera(self):
        if (
            self.camera_active
            and self.cap is not None
            and self.cap.isOpened()
        ):
            rospy.loginfo(
                "摄像头已经处于打开状态"
            )
            return True

        self.release_camera()

        self.cap = cv2.VideoCapture(
            self.camera_device,
            cv2.CAP_V4L2,
        )

        if not self.cap.isOpened():
            self.cap.release()
            self.cap = None
            self.camera_active = False

            rospy.logerr(
                "无法打开摄像头：%s",
                self.camera_device,
            )
            return False

        self.cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1,
        )

        self.camera_active = True

        rospy.loginfo(
            "摄像头打开成功：%s",
            self.camera_device,
        )

        return True

    def release_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        self.camera_active = False

        # 不在Service回调线程操作HighGUI，仅通知主线程关闭窗口。
        self.request_live_window_close()

        rospy.loginfo(
            "摄像头已经释放"
        )

    def clear_camera_buffer(self):
        if (
            not self.camera_active
            or self.cap is None
            or not self.cap.isOpened()
        ):
            rospy.logwarn(
                "摄像头尚未打开，无法清空缓存"
            )
            return False

        self.cap.grab()
        self.cap.grab()

        return True

    def ensure_video_writer(
        self,
        frame
    ):
        if (
            not self.enable_debug_video
            or not self.debug_video_path
        ):
            return

        if (
            self.video_writer is not None
            and self.video_writer.isOpened()
        ):
            return

        output_directory = os.path.dirname(
            self.debug_video_path
        )

        if output_directory:
            try:
                os.makedirs(
                    output_directory,
                    exist_ok=True,
                )
            except OSError as error:
                rospy.logwarn(
                    "无法创建调试视频目录：%s",
                    error,
                )
                return

        height, width = frame.shape[:2]

        fourcc = cv2.VideoWriter_fourcc(
            *"XVID"
        )

        self.video_writer = cv2.VideoWriter(
            self.debug_video_path,
            fourcc,
            self.debug_video_fps,
            (width, height),
        )

        if not self.video_writer.isOpened():
            rospy.logwarn(
                "无法创建调试视频：%s",
                self.debug_video_path,
            )

            self.video_writer.release()
            self.video_writer = None

    def class_name_for_id(
        self,
        class_id,
        mode
    ):
        if mode == "traffic":
            if (
                0 <= class_id
                < len(self.TRAFFIC_CLASS_NAMES)
            ):
                return self.TRAFFIC_CLASS_NAMES[
                    class_id
                ]
            return "traffic_{}".format(
                class_id
            )

        return "target"

    def draw_detections(
        self,
        frame,
        boxes,
        scores,
        class_ids,
        mode
    ):
        for (
            box,
            score,
            class_id
        ) in zip(
            boxes,
            scores,
            class_ids
        ):
            x0, y0, x1, y1 = [
                int(round(value))
                for value in box
            ]

            cv2.rectangle(
                frame,
                (x0, y0),
                (x1, y1),
                (0, 0, 0),
                2,
            )

            name = self.class_name_for_id(
                int(class_id),
                mode,
            )

            text = "{}:{} {:.2f}".format(
                int(class_id),
                name,
                float(score),
            )

            cv2.putText(
                frame,
                text,
                (
                    x0,
                    max(
                        15,
                        y0 + 5,
                    ),
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2,
            )

    def publish_debug_image(
        self,
        frame
    ):
        if (
            not self.enable_debug_image
            or self.debug_image_publisher is None
        ):
            return

        image = np.ascontiguousarray(
            frame
        )

        message = Image()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "camera"
        message.height = image.shape[0]
        message.width = image.shape[1]
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = image.shape[1] * 3
        message.data = image.tobytes()

        self.debug_image_publisher.publish(
            message
        )

    def filter_traffic_boxes_by_position(
        self,
        frame,
        boxes,
        scores,
        class_ids
    ):
        """按还原到原始图像后的检测框中心位置过滤红绿灯候选。"""
        if boxes.size == 0:
            return boxes, scores, class_ids

        frame_height, frame_width = frame.shape[:2]
        keep_indices = []

        for index, (box, score, class_id) in enumerate(
            zip(boxes, scores, class_ids)
        ):
            x0, y0, x1, y1 = [
                float(value)
                for value in box
            ]

            center_x = (x0 + x1) * 0.5
            center_y = (y0 + y1) * 0.5
            center_x_ratio = center_x / max(float(frame_width), 1.0)
            center_y_ratio = center_y / max(float(frame_height), 1.0)

            x_valid = (
                self.traffic_center_x_min_ratio
                <= center_x_ratio
                <= self.traffic_center_x_max_ratio
            )
            y_valid = (
                self.traffic_center_y_min_ratio
                <= center_y_ratio
                <= self.traffic_center_y_max_ratio
            )

            if x_valid and y_valid:
                keep_indices.append(index)
                continue

            rospy.logwarn(
                "红绿灯位置过滤：剔除候选 "
                "class=%d(%s)，score=%.4f，"
                "中心=(%.1f, %.1f)，中心比例=(%.3f, %.3f)，"
                "允许X=[%.3f, %.3f]，Y=[%.3f, %.3f]",
                int(class_id),
                self.class_name_for_id(
                    int(class_id),
                    "traffic",
                ),
                float(score),
                center_x,
                center_y,
                center_x_ratio,
                center_y_ratio,
                self.traffic_center_x_min_ratio,
                self.traffic_center_x_max_ratio,
                self.traffic_center_y_min_ratio,
                self.traffic_center_y_max_ratio,
            )

        if not keep_indices:
            rospy.logwarn(
                "红绿灯位置过滤失败：原始候选=%d，"
                "全部超出允许中心范围，本次/nanodet_detect返回空检测",
                len(boxes),
            )
            return (
                boxes[:0],
                scores[:0],
                class_ids[:0],
            )

        if len(keep_indices) != len(boxes):
            rospy.loginfo(
                "红绿灯位置过滤完成：原始候选=%d，保留=%d，剔除=%d",
                len(boxes),
                len(keep_indices),
                len(boxes) - len(keep_indices),
            )

        keep_indices = np.asarray(
            keep_indices,
            dtype=np.int32,
        )

        return (
            boxes[keep_indices],
            scores[keep_indices],
            class_ids[keep_indices],
        )

    def traffic_box_color_ratios(
        self,
        frame,
        box
    ):
        """计算检测框内部有效绿色和红色像素占比。"""
        frame_height, frame_width = frame.shape[:2]
        x0, y0, x1, y1 = [float(value) for value in box]

        box_width = max(1.0, x1 - x0)
        box_height = max(1.0, y1 - y0)
        inset_x = box_width * self.traffic_color_roi_inset_ratio
        inset_y = box_height * self.traffic_color_roi_inset_ratio

        crop_x0 = max(
            0,
            min(frame_width - 1, int(round(x0 + inset_x))),
        )
        crop_y0 = max(
            0,
            min(frame_height - 1, int(round(y0 + inset_y))),
        )
        crop_x1 = max(
            crop_x0 + 1,
            min(frame_width, int(round(x1 - inset_x))),
        )
        crop_y1 = max(
            crop_y0 + 1,
            min(frame_height, int(round(y1 - inset_y))),
        )

        crop = frame[crop_y0:crop_y1, crop_x0:crop_x1]
        if crop.size == 0:
            return 0.0, 0.0

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        lower_green = np.array(
            [
                self.traffic_green_h_min,
                self.traffic_color_s_min,
                self.traffic_color_v_min,
            ],
            dtype=np.uint8,
        )
        upper_green = np.array(
            [self.traffic_green_h_max, 255, 255],
            dtype=np.uint8,
        )
        green_mask = cv2.inRange(
            hsv,
            lower_green,
            upper_green,
        )

        lower_red_1 = np.array(
            [0, self.traffic_color_s_min, self.traffic_color_v_min],
            dtype=np.uint8,
        )
        upper_red_1 = np.array(
            [self.traffic_red_h_low_max, 255, 255],
            dtype=np.uint8,
        )
        lower_red_2 = np.array(
            [
                self.traffic_red_h_high_min,
                self.traffic_color_s_min,
                self.traffic_color_v_min,
            ],
            dtype=np.uint8,
        )
        upper_red_2 = np.array(
            [179, 255, 255],
            dtype=np.uint8,
        )
        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red_1, upper_red_1),
            cv2.inRange(hsv, lower_red_2, upper_red_2),
        )

        pixel_count = float(max(1, crop.shape[0] * crop.shape[1]))
        green_ratio = float(cv2.countNonZero(green_mask)) / pixel_count
        red_ratio = float(cv2.countNonZero(red_mask)) / pixel_count
        return green_ratio, red_ratio

    def filter_traffic_boxes_by_color(
        self,
        frame,
        boxes,
        scores,
        class_ids
    ):
        """只允许具有有效绿色且不被红色主导的方向灯框通过。"""
        if (
            not self.traffic_green_color_filter_enabled
            or boxes.size == 0
        ):
            return boxes, scores, class_ids

        keep_indices = []

        for index, (box, score, class_id) in enumerate(
            zip(boxes, scores, class_ids)
        ):
            green_ratio, red_ratio = self.traffic_box_color_ratios(
                frame,
                box,
            )

            enough_green = (
                green_ratio >= self.traffic_green_min_ratio
            )
            red_dominant = (
                red_ratio >= self.traffic_red_min_ratio
                and red_ratio >
                    green_ratio *
                    self.traffic_red_to_green_max_ratio
            )

            if enough_green and not red_dominant:
                keep_indices.append(index)
                rospy.loginfo(
                    "红绿灯颜色过滤通过：class=%d(%s)，score=%.4f，"
                    "绿色占比=%.4f，红色占比=%.4f",
                    int(class_id),
                    self.class_name_for_id(int(class_id), "traffic"),
                    float(score),
                    green_ratio,
                    red_ratio,
                )
                continue

            reject_reason = (
                "绿色不足"
                if not enough_green
                else "红色占优"
            )
            rospy.logwarn(
                "红绿灯颜色过滤：剔除候选class=%d(%s)，score=%.4f，"
                "原因=%s，绿色占比=%.4f(要求>=%.4f)，"
                "红色占比=%.4f",
                int(class_id),
                self.class_name_for_id(int(class_id), "traffic"),
                float(score),
                reject_reason,
                green_ratio,
                self.traffic_green_min_ratio,
                red_ratio,
            )

        if not keep_indices:
            rospy.logwarn(
                "红绿灯颜色过滤失败：原始候选=%d，"
                "没有候选同时满足绿色有效且红色不占优；"
                "本次/nanodet_detect返回空检测",
                len(boxes),
            )
            return boxes[:0], scores[:0], class_ids[:0]

        keep_indices = np.asarray(keep_indices, dtype=np.int32)
        return (
            boxes[keep_indices],
            scores[keep_indices],
            class_ids[keep_indices],
        )

    def capture_and_detect(
        self,
        command
    ):
        if (
            not self.camera_active
            or self.cap is None
            or not self.cap.isOpened()
        ):
            rospy.logerr(
                "摄像头未打开，请先发送command=-1"
            )
            return None

        success, frame = self.cap.read()

        if (
            not success
            or frame is None
        ):
            rospy.logerr(
                "从摄像头获取图像失败"
            )
            return None

        if self.flip_horizontal:
            frame = cv2.flip(
                frame,
                1,
            )

        traffic_request = (
            command in (10, 14)
        )

        if traffic_request:
            if self.current_mode != "traffic":
                if not self.switch_to_traffic_mode():
                    return None

            detector = self.traffic_detector

            threshold = (
                self.traffic_low_score_threshold
                if command == 14
                else self.traffic_score_threshold
            )

            mode = "traffic"

        else:
            if self.current_mode != "text":
                rospy.logerr(
                    "当前处于traffic模式，"
                    "文字检测前请command=-11切回text"
                )
                return None

            detector = self.detector

            threshold = (
                self.low_score_threshold
                if command == 4
                else self.score_threshold
            )

            mode = "text"

        if detector is None:
            rospy.logerr(
                "当前模式%s没有可用检测器",
                mode,
            )
            return None

        try:
            (
                boxes,
                scores,
                class_ids,
                raw_shape,
            ) = detector.infer(
                frame,
                threshold,
            )
        except Exception as error:
            rospy.logerr(
                "%s模型检测失败：%s",
                mode,
                error,
            )
            return None

        # YOLOv5红绿灯检测框此时已经由infer()还原到原始相机坐标。
        # 在返回/nanodet_detect结果之前按检测框中心位置进行过滤。
        if mode == "traffic":
            (
                boxes,
                scores,
                class_ids,
            ) = self.filter_traffic_boxes_by_position(
                frame,
                boxes,
                scores,
                class_ids,
            )

            (
                boxes,
                scores,
                class_ids,
            ) = self.filter_traffic_boxes_by_color(
                frame,
                boxes,
                scores,
                class_ids,
            )

        if not self.output_shape_logged[
            mode
        ]:
            rospy.loginfo(
                "%s RKNN原始输出shape：%s",
                mode,
                str(raw_shape),
            )

            self.output_shape_logged[
                mode
            ] = True

        return (
            frame,
            boxes,
            scores,
            class_ids,
            threshold,
            mode,
        )

    def show_live_frame(
        self,
        frame,
        boxes,
        scores,
        class_ids,
        mode
    ):
        """Service线程仅缓存最新标注帧，不直接操作OpenCV HighGUI。"""
        if not self.show_live_window:
            return

        live_frame = frame.copy()

        self.draw_detections(
            live_frame,
            boxes,
            scores,
            class_ids,
            mode,
        )

        with self.live_frame_lock:
            self.latest_live_frame = live_frame
            self.live_window_should_close = False

    def process_live_window(self):
        """只允许Python主线程调用：imshow + waitKey。"""
        if not self.show_live_window:
            return

        frame_to_show = None
        should_close = False

        with self.live_frame_lock:
            if self.latest_live_frame is not None:
                frame_to_show = self.latest_live_frame.copy()
            should_close = self.live_window_should_close

        if should_close:
            if self.live_window_open:
                try:
                    cv2.destroyWindow(self.live_window_name)
                    cv2.waitKey(1)
                except cv2.error:
                    pass
                self.live_window_open = False

            with self.live_frame_lock:
                self.live_window_should_close = False
            return

        if frame_to_show is None:
            if self.live_window_open:
                try:
                    cv2.waitKey(1)
                except cv2.error:
                    pass
            return

        try:
            cv2.imshow(
                self.live_window_name,
                frame_to_show,
            )
            self.live_window_open = True
            cv2.waitKey(1)
        except cv2.error as error:
            rospy.logwarn_throttle(
                2.0,
                "OpenCV实时窗口显示失败：%s",
                str(error),
            )

    def request_live_window_close(self):
        """Service线程只发关闭请求，真正销毁窗口由主线程完成。"""
        if not self.show_live_window:
            return

        with self.live_frame_lock:
            self.latest_live_frame = None
            self.live_window_should_close = True

    def close_live_window_now(self):
        """进程退出时由主线程最终关闭所有OpenCV窗口。"""
        if not self.show_live_window:
            return

        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except cv2.error:
            pass

        with self.live_frame_lock:
            self.latest_live_frame = None
            self.live_window_should_close = False

        self.live_window_open = False

    def output_debug_frame(
        self,
        frame,
        boxes,
        scores,
        class_ids,
        mode
    ):
        if (
            not self.enable_debug_image
            and not self.enable_debug_video
        ):
            return

        debug_frame = frame.copy()

        self.draw_detections(
            debug_frame,
            boxes,
            scores,
            class_ids,
            mode,
        )

        if self.enable_debug_image:
            self.publish_debug_image(
                debug_frame
            )

        if self.enable_debug_video:
            self.ensure_video_writer(
                debug_frame
            )

            if self.video_writer is not None:
                self.video_writer.write(
                    debug_frame
                )

    def handle_request(
        self,
        request
    ):
        with self.lock:
            command = int(
                request.detect_start
            )

            if command == -1:
                success = self.open_camera()
                return self.make_control_response(
                    -1 if success else -99
                )

            if command == -2:
                self.release_camera()
                return self.make_control_response(
                    -2
                )

            if command == -3:
                success = self.clear_camera_buffer()
                return self.make_control_response(
                    -3 if success else -99
                )

            if command == -10:
                success = self.switch_to_traffic_mode()
                return self.make_control_response(
                    -10 if success else -99
                )

            if command == -11:
                success = self.switch_to_text_mode()
                return self.make_control_response(
                    -11 if success else -99
                )

            response = detect_result_srvResponse()

            detection = self.capture_and_detect(
                command
            )

            if detection is None:
                return response

            (
                frame,
                boxes,
                scores,
                class_ids,
                threshold,
                mode,
            ) = detection

            for (
                box,
                score,
                class_id
            ) in zip(
                boxes,
                scores,
                class_ids
            ):
                x0, y0, x1, y1 = [
                    int(round(value))
                    for value in box
                ]

                response.class_name.append(
                    int(class_id)
                )
                response.x0.append(x0)
                response.y0.append(y0)
                response.x1.append(x1)
                response.y1.append(y1)

                if mode == "traffic":
                    rospy.loginfo(
                        "红绿灯候选："
                        "class=%d(%s)，score=%.4f，"
                        "box=(%d,%d)-(%d,%d)",
                        int(class_id),
                        self.class_name_for_id(
                            int(class_id),
                            mode,
                        ),
                        float(score),
                        x0,
                        y0,
                        x1,
                        y1,
                    )

            self.show_live_frame(
                frame,
                boxes,
                scores,
                class_ids,
                mode,
            )

            self.output_debug_frame(
                frame,
                boxes,
                scores,
                class_ids,
                mode,
            )

            rospy.loginfo(
                "%s检测完成：数量=%d，阈值=%.2f",
                mode,
                len(response.class_name),
                threshold,
            )

            return response

    def handle_ocr_request(
        self,
        request
    ):
        with self.lock:
            command = int(
                request.command
            )

            response = (
                ocr_result_srvResponse()
            )

            if command == -1:
                response.success = (
                    self.open_camera()
                )
                return response

            if command == -2:
                self.release_camera()
                response.success = True
                return response

            if command == -3:
                response.success = (
                    self.clear_camera_buffer()
                )
                return response

            if self.current_mode != "text":
                rospy.logerr(
                    "当前处于红绿灯模式，"
                    "OCR不可用；如需恢复请先通过"
                    "/nanodet_detect发送command=-11"
                )

                response.success = False
                return response

            detection = self.capture_and_detect(
                command
            )

            if detection is None:
                response.success = False
                return response

            (
                frame,
                boxes,
                scores,
                class_ids,
                threshold,
                mode,
            ) = detection

            frame_height, frame_width = (
                frame.shape[:2]
            )

            for (
                box,
                detect_score,
                class_id
            ) in zip(
                boxes,
                scores,
                class_ids
            ):
                x0, y0, x1, y1 = [
                    int(round(value))
                    for value in box
                ]

                x0 = max(
                    0,
                    min(
                        frame_width,
                        x0,
                    ),
                )
                x1 = max(
                    0,
                    min(
                        frame_width,
                        x1,
                    ),
                )
                y0 = max(
                    0,
                    min(
                        frame_height,
                        y0,
                    ),
                )
                y1 = max(
                    0,
                    min(
                        frame_height,
                        y1,
                    ),
                )

                if (
                    x1 <= x0
                    or y1 <= y0
                ):
                    rospy.logwarn(
                        "忽略无效文字框："
                        "(%d,%d)-(%d,%d)",
                        x0,
                        y0,
                        x1,
                        y1,
                    )
                    continue

                crop = frame[
                    y0:y1,
                    x0:x1
                ]

                try:
                    (
                        text,
                        confidence,
                        raw_shape,
                    ) = self.ocr_recognizer.recognize(
                        crop
                    )
                except Exception as error:
                    rospy.logerr(
                        "PP-OCR识别失败，"
                        "框=(%d,%d)-(%d,%d)：%s",
                        x0,
                        y0,
                        x1,
                        y1,
                        error,
                    )
                    continue

                response.text.append(
                    text
                )
                response.confidence.append(
                    confidence
                )
                response.detect_score.append(
                    float(
                        detect_score
                    )
                )
                response.x0.append(x0)
                response.y0.append(y0)
                response.x1.append(x1)
                response.y1.append(y1)

                rospy.loginfo(
                    "PP-OCR原始文字='%s'，"
                    "OCR置信度=%.4f，"
                    "检测置信度=%.4f，"
                    "框=(%d,%d)-(%d,%d)，"
                    "输出shape=%s",
                    text,
                    confidence,
                    float(
                        detect_score
                    ),
                    x0,
                    y0,
                    x1,
                    y1,
                    str(
                        raw_shape
                    ),
                )

            response.success = True

            self.output_debug_frame(
                frame,
                boxes,
                scores,
                class_ids,
                mode,
            )

            rospy.loginfo(
                "文字框检测与OCR完成："
                "检测框=%d，成功识别=%d，阈值=%.2f",
                len(boxes),
                len(response.text),
                threshold,
            )

            return response

    def shutdown(self):
        with self.lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None

            self.camera_active = False

            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None

            self.release_text_models()
            self.release_traffic_model()

            # shutdown回调可能不在主线程，只发关闭请求。
            self.request_live_window_close()

        rospy.loginfo(
            "RKNN统一检测节点已经退出"
        )


def main():
    rospy.init_node(
        "nanodet_detect",
        anonymous=False
    )

    server = Detect2026Server()

    if not server.show_live_window:
        rospy.spin()
        return

    rospy.loginfo(
        "OpenCV实时窗口采用主线程刷新模式，"
        "Service回调不再执行imshow/waitKey"
    )

    try:
        # rospy的Service请求由内部线程处理；
        # Python主线程只负责OpenCV HighGUI事件循环。
        while not rospy.is_shutdown():
            server.process_live_window()
            time.sleep(0.02)

    finally:
        server.close_live_window_now()


if __name__ == "__main__":
    main()