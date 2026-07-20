#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# -*- coding: utf-8 -*-

"""NanoDet-Plus 1.0 单类别文字框 RKNN ROS 服务。

保持 detect2025.py 的 ROS 服务接口：
  -1：打开 /dev/video0
  -2：释放摄像头
  -3：丢弃两帧缓存
   4：以较低阈值检测
  其他值：采集一帧，返回全部文字框

模型要求：
  输入  [1, 3, 320, 320]
  输出  [1, 2125, 33]
  类别  target（ID=0）
  stride [8, 16, 32, 64]
  reg_max 7
"""

import os
import sys
import threading
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
    """NanoDet-Plus 1.0 RKNN推理及后处理。"""

    def __init__(
        self,
        model_path,
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
        self.num_classes = 1
        self.output_channels = self.num_classes + 4 * (self.reg_max + 1)
        self.pre_nms_topk = int(pre_nms_topk)
        self.nms_threshold = float(nms_threshold)
        self.project = np.arange(self.reg_max + 1, dtype=np.float32)
        self.center_priors = self._make_center_priors()
        self.expected_points = self.center_priors.shape[0]
        self.rknn = None
        self._load_model()

    def _make_center_priors(self):
        """与 NanoDet 1.0 get_single_level_center_priors 保持一致。"""
        priors = []
        for stride in self.strides:
            feature_width = self.input_size // stride
            feature_height = self.input_size // stride
            grid_y, grid_x = np.meshgrid(
                np.arange(feature_height, dtype=np.float32),
                np.arange(feature_width, dtype=np.float32),
                indexing="ij",
            )
            center_x = grid_x.reshape(-1) * float(stride)
            center_y = grid_y.reshape(-1) * float(stride)
            stride_values = np.full_like(center_x, float(stride))
            priors.append(
                np.stack([center_x, center_y, stride_values], axis=1)
            )
        return np.concatenate(priors, axis=0).astype(np.float32)

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            raise RuntimeError("找不到RKNN模型：{}".format(self.model_path))

        self.rknn = RKNNLite()
        result = self.rknn.load_rknn(self.model_path)
        if result != 0:
            raise RuntimeError(
                "加载RKNN模型失败，返回值={}，路径={}".format(
                    result,
                    self.model_path,
                )
            )

        core_mask = getattr(RKNNLite, "NPU_CORE_0_1_2", None)
        if core_mask is None:
            result = self.rknn.init_runtime()
        else:
            result = self.rknn.init_runtime(core_mask=core_mask)

        if result != 0:
            raise RuntimeError("初始化RKNN Runtime失败，返回值={}".format(result))

        # 某些RKNNLite版本也可能在创建实例或初始化Runtime时再次修改映射。
        restore_python_logging_levels()

        rospy.loginfo("RKNN文字框模型加载成功：%s", self.model_path)
        rospy.loginfo(
            "模型后处理参数：input=%d，points=%d，channels=%d",
            self.input_size,
            self.expected_points,
            self.output_channels,
        )

    def _normalize_output(self, output):
        """兼容 [1,N,C]、[1,N,C,1]、[1,C,N] 和展平输出。"""
        values = np.asarray(output)
        raw_shape = values.shape
        values = np.squeeze(values)

        if values.ndim == 1:
            expected_size = self.expected_points * self.output_channels
            if values.size != expected_size:
                raise RuntimeError(
                    "RKNN输出元素数错误：shape={}，size={}，期望={}".format(
                        raw_shape,
                        values.size,
                        expected_size,
                    )
                )
            values = values.reshape(self.expected_points, self.output_channels)

        elif values.ndim == 2:
            if values.shape == (self.output_channels, self.expected_points):
                values = values.T
            elif values.shape != (self.expected_points, self.output_channels):
                raise RuntimeError(
                    "RKNN输出shape错误：原始={}，压缩后={}，期望=({}, {})".format(
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

        return values.astype(np.float32), raw_shape

    def _decode(self, output, score_threshold):
        predictions, raw_shape = self._normalize_output(output)

        class_values = predictions[:, :self.num_classes]

        # NanoDet-Plus 1.0 的 ONNX 导出通常已经包含 sigmoid。
        if class_values.min() >= 0.0 and class_values.max() <= 1.0:
            class_scores = class_values[:, 0]
        else:
            class_scores = sigmoid(class_values[:, 0])

        valid = np.where(class_scores >= float(score_threshold))[0]
        if valid.size == 0:
            return (
                np.empty((0, 4), dtype=np.float32),
                np.empty((0,), dtype=np.float32),
                raw_shape,
            )

        if self.pre_nms_topk > 0 and valid.size > self.pre_nms_topk:
            valid_scores = class_scores[valid]
            order = np.argsort(valid_scores)[::-1][:self.pre_nms_topk]
            valid = valid[order]

        priors = self.center_priors[valid]
        scores = class_scores[valid]
        distance_logits = predictions[valid, self.num_classes:]
        distance_logits = distance_logits.reshape(
            -1,
            4,
            self.reg_max + 1,
        )
        distance_distribution = softmax(distance_logits, axis=2)
        distances = np.sum(
            distance_distribution * self.project.reshape(1, 1, -1),
            axis=2,
        )
        distances = distances * priors[:, 2:3]

        boxes = np.stack(
            [
                priors[:, 0] - distances[:, 0],
                priors[:, 1] - distances[:, 1],
                priors[:, 0] + distances[:, 2],
                priors[:, 1] + distances[:, 3],
            ],
            axis=1,
        )
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, self.input_size - 1)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, self.input_size - 1)

        keep = nms(
            boxes,
            scores,
            iou_threshold=self.nms_threshold,
            max_count=100,
        )
        return boxes[keep], scores[keep], raw_shape

    def infer(self, frame, score_threshold):
        raw_height, raw_width = frame.shape[:2]
        resize_matrix = make_resize_matrix(
            raw_width,
            raw_height,
            self.input_size,
            self.input_size,
        )
        model_input = cv2.warpPerspective(
            frame,
            resize_matrix,
            (self.input_size, self.input_size),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        # 均值和方差已经在 ONNX->RKNN 转换时写入模型，板端传BGR uint8。
        try:
            outputs = self.rknn.inference(
                inputs=[model_input],
                data_format=["nhwc"],
            )
        except TypeError:
            outputs = self.rknn.inference(inputs=[model_input])

        if outputs is None or len(outputs) != 1:
            raise RuntimeError(
                "RKNN输出数量错误：{}".format(
                    None if outputs is None else len(outputs)
                )
            )

        boxes, scores, raw_shape = self._decode(outputs[0], score_threshold)
        if boxes.size != 0:
            inverse_matrix = np.linalg.inv(resize_matrix)
            boxes = restore_boxes(
                boxes,
                inverse_matrix,
                raw_width,
                raw_height,
            )
        return boxes, scores, raw_shape

    def warmup(self, count=3):
        image = np.zeros(
            (self.input_size, self.input_size, 3),
            dtype=np.uint8,
        )
        for _ in range(max(0, int(count))):
            self.infer(image, 0.99)
        rospy.loginfo("RKNN模型预热完成，共%d次", max(0, int(count)))

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
    def __init__(self):
        self.model_path = rospy.get_param("~model_path", DEFAULT_MODEL_PATH)
        self.ocr_model_path = rospy.get_param(
            "~ocr_model_path",
            DEFAULT_OCR_MODEL_PATH,
        )
        self.ocr_dict_path = rospy.get_param(
            "~ocr_dict_path",
            DEFAULT_OCR_DICT_PATH,
        )
        self.camera_device = rospy.get_param("~camera_device", "/dev/video0")
        self.score_threshold = float(rospy.get_param("~score_threshold", 0.50))
        self.low_score_threshold = float(
            rospy.get_param("~low_score_threshold", 0.45)
        )
        self.nms_threshold = float(rospy.get_param("~nms_threshold", 0.60))
        self.flip_horizontal = bool(rospy.get_param("~flip_horizontal", True))
        self.debug_video_path = rospy.get_param(
            "~debug_video_path",
            DEFAULT_VIDEO_PATH,
        )
        self.debug_video_fps = float(rospy.get_param("~debug_video_fps", 3.0))
        self.warmup_count = int(rospy.get_param("~warmup_count", 3))
        self.ocr_warmup_count = int(
            rospy.get_param("~ocr_warmup_count", 1)
        )
        self.enable_debug_image = bool(
            rospy.get_param("~enable_debug_image", False)
        )
        self.enable_debug_video = bool(
            rospy.get_param("~enable_debug_video", False)
        )

        self.detector = NanoDetPlusRKNN(
            model_path=self.model_path,
            input_size=320,
            strides=(8, 16, 32, 64),
            reg_max=7,
            pre_nms_topk=1000,
            nms_threshold=self.nms_threshold,
        )
        self.detector.warmup(self.warmup_count)
        self.ocr_recognizer = PPOCRRecognizerRKNN(
            model_path=self.ocr_model_path,
            dictionary_path=self.ocr_dict_path,
        )
        self.ocr_recognizer.warmup(self.ocr_warmup_count)

        self.lock = threading.Lock()
        self.cap = None
        self.camera_active = False
        self.video_writer = None
        self.output_shape_logged = False

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
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "文字框检测与OCR服务已就绪：/nanodet_detect、/nanodet_ocr，"
            "图像发布=%s，视频录制=%s",
            self.enable_debug_image,
            self.enable_debug_video,
        )

    @staticmethod
    def make_control_response():
        response = detect_result_srvResponse()
        response.class_name.append(-1)
        response.x0.append(-1)
        response.y0.append(-1)
        response.x1.append(-1)
        response.y1.append(-1)
        return response

    def open_camera(self):
        if self.camera_active and self.cap is not None and self.cap.isOpened():
            rospy.loginfo("摄像头已经处于打开状态")
            return True

        self.release_camera()
        self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = None
            self.camera_active = False
            rospy.logerr("无法打开摄像头：%s", self.camera_device)
            return False

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.camera_active = True
        rospy.loginfo("摄像头打开成功：%s", self.camera_device)
        return True

    def release_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.camera_active = False
        rospy.loginfo("摄像头已经释放")

    def clear_camera_buffer(self):
        if not self.camera_active or self.cap is None or not self.cap.isOpened():
            rospy.logwarn("摄像头尚未打开，无法清空缓存")
            return False
        self.cap.grab()
        self.cap.grab()
        return True

    def ensure_video_writer(self, frame):
        if not self.enable_debug_video or not self.debug_video_path:
            return
        if self.video_writer is not None and self.video_writer.isOpened():
            return

        output_directory = os.path.dirname(self.debug_video_path)
        if output_directory:
            try:
                os.makedirs(output_directory, exist_ok=True)
            except OSError as error:
                rospy.logwarn("无法创建调试视频目录：%s", error)
                return

        height, width = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.video_writer = cv2.VideoWriter(
            self.debug_video_path,
            fourcc,
            self.debug_video_fps,
            (width, height),
        )
        if not self.video_writer.isOpened():
            rospy.logwarn("无法创建调试视频：%s", self.debug_video_path)
            self.video_writer.release()
            self.video_writer = None

    @staticmethod
    def draw_detections(frame, boxes, scores):
        for box, score in zip(boxes, scores):
            x0, y0, x1, y1 = [int(round(value)) for value in box]
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 0, 0), 2)
            text = "target: {:.2f}:{}".format(score, y1 - y0)
            cv2.putText(
                frame,
                text,
                (x0, max(15, y0 + 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2,
            )

    def publish_debug_image(self, frame):
        """发布当前标注画面，不依赖cv_bridge，也不启动ROS相机节点。"""
        if not self.enable_debug_image or self.debug_image_publisher is None:
            return
        image = np.ascontiguousarray(frame)
        message = Image()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "camera"
        message.height = image.shape[0]
        message.width = image.shape[1]
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = image.shape[1] * 3
        message.data = image.tobytes()
        self.debug_image_publisher.publish(message)

    def capture_and_detect(self, command):
        """采集一帧并完成NanoDet推理，供检测和OCR两个服务共用。"""
        if not self.camera_active or self.cap is None or not self.cap.isOpened():
            rospy.logerr("摄像头未打开，请先发送command=-1")
            return None

        success, frame = self.cap.read()
        if not success or frame is None:
            rospy.logerr("从摄像头获取图像失败")
            return None

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)

        threshold = (
            self.low_score_threshold
            if command == 4
            else self.score_threshold
        )

        try:
            boxes, scores, raw_shape = self.detector.infer(frame, threshold)
        except Exception as error:
            rospy.logerr("文字框检测失败：%s", error)
            return None

        if not self.output_shape_logged:
            rospy.loginfo("NanoDet RKNN原始输出shape：%s", str(raw_shape))
            self.output_shape_logged = True

        return frame, boxes, scores, threshold

    def output_debug_frame(self, frame, boxes, scores):
        if not self.enable_debug_image and not self.enable_debug_video:
            return

        debug_frame = frame.copy()
        self.draw_detections(debug_frame, boxes, scores)

        if self.enable_debug_image:
            self.publish_debug_image(debug_frame)

        if self.enable_debug_video:
            self.ensure_video_writer(debug_frame)
            if self.video_writer is not None:
                self.video_writer.write(debug_frame)

    def handle_request(self, request):
        with self.lock:
            command = int(request.detect_start)

            if command == -1:
                self.open_camera()
                return self.make_control_response()

            if command == -2:
                self.release_camera()
                return self.make_control_response()

            if command == -3:
                self.clear_camera_buffer()
                return self.make_control_response()

            response = detect_result_srvResponse()
            detection = self.capture_and_detect(command)
            if detection is None:
                return response
            frame, boxes, scores, threshold = detection

            for box, score in zip(boxes, scores):
                x0, y0, x1, y1 = [int(round(value)) for value in box]
                response.class_name.append(0)
                response.x0.append(x0)
                response.y0.append(y0)
                response.x1.append(x1)
                response.y1.append(y1)

            self.output_debug_frame(frame, boxes, scores)

            rospy.loginfo(
                "文字框检测完成：数量=%d，阈值=%.2f",
                len(response.class_name),
                threshold,
            )
            return response

    def handle_ocr_request(self, request):
        with self.lock:
            command = int(request.command)
            response = ocr_result_srvResponse()

            if command == -1:
                response.success = self.open_camera()
                return response

            if command == -2:
                self.release_camera()
                response.success = True
                return response

            if command == -3:
                response.success = self.clear_camera_buffer()
                return response

            detection = self.capture_and_detect(command)
            if detection is None:
                response.success = False
                return response

            frame, boxes, scores, threshold = detection
            frame_height, frame_width = frame.shape[:2]

            for box, detect_score in zip(boxes, scores):
                # 按用户实测要求严格使用NanoDet原框，不进行任何扩边。
                x0, y0, x1, y1 = [int(round(value)) for value in box]
                x0 = max(0, min(frame_width, x0))
                x1 = max(0, min(frame_width, x1))
                y0 = max(0, min(frame_height, y0))
                y1 = max(0, min(frame_height, y1))

                if x1 <= x0 or y1 <= y0:
                    rospy.logwarn(
                        "忽略无效文字框：(%d,%d)-(%d,%d)",
                        x0,
                        y0,
                        x1,
                        y1,
                    )
                    continue

                crop = frame[y0:y1, x0:x1]
                try:
                    text, confidence, raw_shape = self.ocr_recognizer.recognize(
                        crop
                    )
                except Exception as error:
                    rospy.logerr(
                        "PP-OCR识别失败，框=(%d,%d)-(%d,%d)：%s",
                        x0,
                        y0,
                        x1,
                        y1,
                        error,
                    )
                    continue

                response.text.append(text)
                response.confidence.append(confidence)
                response.detect_score.append(float(detect_score))
                response.x0.append(x0)
                response.y0.append(y0)
                response.x1.append(x1)
                response.y1.append(y1)

                rospy.loginfo(
                    "PP-OCR原始文字='%s'，OCR置信度=%.4f，"
                    "检测置信度=%.4f，框=(%d,%d)-(%d,%d)，输出shape=%s",
                    text,
                    confidence,
                    float(detect_score),
                    x0,
                    y0,
                    x1,
                    y1,
                    str(raw_shape),
                )

            response.success = True
            self.output_debug_frame(frame, boxes, scores)
            rospy.loginfo(
                "文字框检测与OCR完成：检测框=%d，成功识别=%d，阈值=%.2f",
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

            if self.detector is not None:
                self.detector.release()
                self.detector = None

            if self.ocr_recognizer is not None:
                self.ocr_recognizer.release()
                self.ocr_recognizer = None

        rospy.loginfo("NanoDet-Plus文字框检测与PP-OCR识别节点已经退出")


def main():
    rospy.init_node("nanodet_detect", anonymous=False)
    Detect2026Server()
    rospy.spin()


if __name__ == "__main__":
    main()