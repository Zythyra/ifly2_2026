#!/home/ucar/miniforge3/envs/rknn_env/bin/python3
# -*- coding: utf-8 -*-

"""RK3588 YOLOv5 红绿灯三分类独立测试程序。

模型接口：
  输入：1 x 640 x 640 x 3，RGB，uint8（RKNN内部完成归一化）
  输出：1 x 25200 x 8
  类别：0=左转，1=右转，2=直行

运行示例：
  /home/ucar/miniforge3/envs/rknn_env/bin/python3 \
      traffic_light_yolov5_rknn_test.py \
      --model ./best_yolov5_640_fp16.rknn

按键：
  q / ESC：退出
  s      ：保存当前标注画面
  r      ：清空连续帧稳定计数
"""

import argparse
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
from rknnlite.api import RKNNLite


CLASS_NAMES = ("LEFT", "RIGHT", "STRAIGHT")
CLASS_NAMES_CN = ("左转", "右转", "直行")
INPUT_SIZE = 640
EXPECTED_POINTS = 25200
OUTPUT_CHANNELS = 8


def parse_arguments():
    script_directory = os.path.dirname(os.path.realpath(__file__))
    default_model = os.path.join(
        script_directory,
        "best_yolov5_640_fp16.rknn",
    )

    parser = argparse.ArgumentParser(
        description="RK3588 YOLOv5红绿灯三分类独立测试"
    )
    parser.add_argument(
        "--model",
        default=default_model,
        help="RKNN模型路径，默认读取脚本同目录下的模型",
    )
    parser.add_argument(
        "--camera",
        default="/dev/video0",
        help="摄像头设备，默认/dev/video0",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="置信度阈值，默认0.25",
    )
    parser.add_argument(
        "--nms",
        type=float,
        default=0.45,
        help="NMS IoU阈值，默认0.45",
    )
    parser.add_argument(
        "--stable-frames",
        type=int,
        default=3,
        help="连续相同类别确认帧数，默认3；设为1表示单帧确认",
    )
    parser.add_argument(
        "--flush-frames",
        type=int,
        default=8,
        help="打开摄像头后丢弃的缓存帧数，默认8",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=3,
        help="NPU模型预热次数，默认3",
    )
    parser.add_argument(
        "--no-mirror",
        action="store_true",
        help="关闭水平镜像；默认与现有小车程序一致，启用水平镜像",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="不显示OpenCV窗口，只在终端打印检测结果",
    )
    parser.add_argument(
        "--save-dir",
        default="/tmp/traffic_light_yolov5_test",
        help="按s保存画面的目录",
    )
    return parser.parse_args()


def sigmoid(values):
    values = np.clip(values.astype(np.float32), -50.0, 50.0)
    return 1.0 / (1.0 + np.exp(-values))


def as_probability(values):
    values = np.asarray(values, dtype=np.float32)
    if values.size == 0:
        return values
    if values.min() >= 0.0 and values.max() <= 1.0:
        return values
    return sigmoid(values)


def letterbox(frame, input_size=INPUT_SIZE):
    raw_height, raw_width = frame.shape[:2]
    ratio = min(
        float(input_size) / float(raw_width),
        float(input_size) / float(raw_height),
    )

    resized_width = max(1, int(round(raw_width * ratio)))
    resized_height = max(1, int(round(raw_height * ratio)))
    resized = cv2.resize(
        frame,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )

    left = (input_size - resized_width) // 2
    top = (input_size - resized_height) // 2

    canvas = np.full(
        (input_size, input_size, 3),
        114,
        dtype=np.uint8,
    )
    canvas[
        top:top + resized_height,
        left:left + resized_width,
    ] = resized

    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb), ratio, left, top


def normalize_output(output):
    values = np.asarray(output)
    raw_shape = tuple(values.shape)
    values = np.squeeze(values)

    if values.ndim == 1:
        expected_size = EXPECTED_POINTS * OUTPUT_CHANNELS
        if values.size != expected_size:
            raise RuntimeError(
                "输出元素数量错误：原始shape={}，实际={}，期望={}".format(
                    raw_shape,
                    values.size,
                    expected_size,
                )
            )
        values = values.reshape(EXPECTED_POINTS, OUTPUT_CHANNELS)

    elif values.ndim == 2:
        if values.shape == (OUTPUT_CHANNELS, EXPECTED_POINTS):
            values = values.T
        elif values.shape != (EXPECTED_POINTS, OUTPUT_CHANNELS):
            raise RuntimeError(
                "输出shape错误：原始={}，压缩后={}，期望=({}, {})".format(
                    raw_shape,
                    tuple(values.shape),
                    EXPECTED_POINTS,
                    OUTPUT_CHANNELS,
                )
            )
    else:
        raise RuntimeError(
            "输出维度错误：原始={}，压缩后={}".format(
                raw_shape,
                tuple(values.shape),
            )
        )

    return values.astype(np.float32), raw_shape


def nms(boxes, scores, iou_threshold, max_count=100):
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
        order = remaining[iou <= float(iou_threshold)]

    return np.asarray(keep, dtype=np.int32)


def empty_result(raw_shape):
    return (
        np.empty((0, 4), dtype=np.float32),
        np.empty((0,), dtype=np.float32),
        np.empty((0,), dtype=np.int32),
        raw_shape,
    )


def decode(output, score_threshold, nms_threshold):
    predictions, raw_shape = normalize_output(output)

    objectness = as_probability(predictions[:, 4])
    class_probabilities = as_probability(predictions[:, 5:8])
    class_ids = np.argmax(class_probabilities, axis=1).astype(np.int32)
    class_scores = np.max(class_probabilities, axis=1)
    scores = objectness * class_scores

    valid = np.where(scores >= float(score_threshold))[0]
    if valid.size == 0:
        return empty_result(raw_shape)

    if valid.size > 3000:
        order = np.argsort(scores[valid])[::-1][:3000]
        valid = valid[order]

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

    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, INPUT_SIZE - 1)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, INPUT_SIZE - 1)

    valid_size = np.where(
        (boxes[:, 2] - boxes[:, 0] >= 1.0)
        & (boxes[:, 3] - boxes[:, 1] >= 1.0)
    )[0]
    boxes = boxes[valid_size]
    candidate_scores = candidate_scores[valid_size]
    candidate_classes = candidate_classes[valid_size]

    if boxes.size == 0:
        return empty_result(raw_shape)

    keep_indices = []
    for class_id in np.unique(candidate_classes):
        class_positions = np.where(candidate_classes == class_id)[0]
        local_keep = nms(
            boxes[class_positions],
            candidate_scores[class_positions],
            nms_threshold,
        )
        keep_indices.extend(class_positions[local_keep].tolist())

    if not keep_indices:
        return empty_result(raw_shape)

    keep_indices = np.asarray(keep_indices, dtype=np.int32)
    keep_indices = keep_indices[
        np.argsort(candidate_scores[keep_indices])[::-1]
    ]

    return (
        boxes[keep_indices],
        candidate_scores[keep_indices],
        candidate_classes[keep_indices],
        raw_shape,
    )


def restore_boxes(boxes, ratio, left, top, raw_width, raw_height):
    if boxes.size == 0:
        return boxes

    restored = boxes.copy()
    restored[:, [0, 2]] = (
        restored[:, [0, 2]] - float(left)
    ) / max(float(ratio), 1e-12)
    restored[:, [1, 3]] = (
        restored[:, [1, 3]] - float(top)
    ) / max(float(ratio), 1e-12)

    restored[:, [0, 2]] = restored[:, [0, 2]].clip(0, raw_width - 1)
    restored[:, [1, 3]] = restored[:, [1, 3]].clip(0, raw_height - 1)
    return restored


def draw_detections(frame, boxes, scores, class_ids):
    colors = (
        (255, 128, 0),
        (0, 165, 255),
        (0, 255, 0),
    )

    for box, score, class_id in zip(boxes, scores, class_ids):
        class_id = int(class_id)
        x0, y0, x1, y1 = [int(round(value)) for value in box]
        color = colors[class_id]
        label = "{} {:.3f}".format(CLASS_NAMES[class_id], float(score))

        cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
        cv2.putText(
            frame,
            label,
            (x0, max(22, y0 - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
        )


class StableDirection(object):
    def __init__(self, required_frames):
        self.required_frames = max(1, int(required_frames))
        self.last_class_id = -1
        self.count = 0
        self.confirmed_class_id = -1

    def reset(self):
        self.last_class_id = -1
        self.count = 0
        self.confirmed_class_id = -1

    def update(self, class_id):
        class_id = int(class_id)
        if class_id == self.last_class_id:
            self.count += 1
        else:
            self.last_class_id = class_id
            self.count = 1
            self.confirmed_class_id = -1

        newly_confirmed = False
        if (
            self.count >= self.required_frames
            and self.confirmed_class_id != class_id
        ):
            self.confirmed_class_id = class_id
            newly_confirmed = True

        return newly_confirmed


def open_camera(device, width, height, flush_frames):
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError("无法打开摄像头：{}".format(device))

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(max(0, int(flush_frames))):
        camera.grab()

    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(
        "摄像头打开成功：{}，实际分辨率={}x{}".format(
            device,
            actual_width,
            actual_height,
        )
    )
    return camera


def load_model(model_path):
    if not os.path.isfile(model_path):
        raise RuntimeError("找不到RKNN模型：{}".format(model_path))

    rknn = RKNNLite()
    result = rknn.load_rknn(model_path)
    if result not in (0, None):
        rknn.release()
        raise RuntimeError("加载RKNN模型失败，返回值={}".format(result))

    core_mask = getattr(RKNNLite, "NPU_CORE_0_1_2", None)
    if core_mask is None:
        result = rknn.init_runtime()
    else:
        result = rknn.init_runtime(core_mask=core_mask)

    if result not in (0, None):
        rknn.release()
        raise RuntimeError(
            "初始化RKNN Runtime失败，返回值={}".format(result)
        )

    print("RKNN模型加载成功：{}".format(model_path))
    print("模型接口：RGB 640x640 -> (1,25200,8)")
    return rknn


def run_inference(rknn, frame, conf_threshold, nms_threshold):
    raw_height, raw_width = frame.shape[:2]
    model_input, ratio, left, top = letterbox(frame)

    try:
        outputs = rknn.inference(
            inputs=[model_input],
            data_format=["nhwc"],
        )
    except TypeError:
        outputs = rknn.inference(inputs=[model_input])

    if outputs is None or len(outputs) != 1:
        raise RuntimeError(
            "RKNN输出数量错误：{}".format(
                None if outputs is None else len(outputs)
            )
        )

    boxes, scores, class_ids, raw_shape = decode(
        outputs[0],
        conf_threshold,
        nms_threshold,
    )
    boxes = restore_boxes(
        boxes,
        ratio,
        left,
        top,
        raw_width,
        raw_height,
    )
    return boxes, scores, class_ids, raw_shape


def save_frame(frame, save_directory):
    os.makedirs(save_directory, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output_path = os.path.join(
        save_directory,
        "traffic_light_{}.jpg".format(timestamp),
    )
    if not cv2.imwrite(output_path, frame):
        print("保存画面失败：{}".format(output_path))
        return
    print("已保存画面：{}".format(output_path))


def main():
    arguments = parse_arguments()
    mirror_image = not arguments.no_mirror
    stable = StableDirection(arguments.stable_frames)

    rknn = None
    camera = None
    output_shape_logged = False
    smoothed_fps = 0.0

    try:
        rknn = load_model(arguments.model)

        print("开始进行{}次模型预热...".format(max(0, arguments.warmup)))
        warmup_image = np.zeros((640, 640, 3), dtype=np.uint8)
        for _ in range(max(0, arguments.warmup)):
            run_inference(
                rknn,
                warmup_image,
                0.99,
                arguments.nms,
            )
        print("模型预热完成")

        camera = open_camera(
            arguments.camera,
            arguments.width,
            arguments.height,
            arguments.flush_frames,
        )

        print("========== YOLOv5红绿灯测试开始 ==========")
        print("类别：0=左转，1=右转，2=直行")
        print(
            "阈值：conf={:.3f}，nms={:.3f}，稳定帧={}".format(
                arguments.conf,
                arguments.nms,
                max(1, arguments.stable_frames),
            )
        )
        print("水平镜像：{}".format("开启" if mirror_image else "关闭"))
        if not arguments.headless:
            print("按q或ESC退出，按s保存画面，按r重置稳定计数")

        while True:
            success, frame = camera.read()
            if not success or frame is None:
                print("读取摄像头失败，继续重试")
                time.sleep(0.05)
                continue

            if mirror_image:
                frame = cv2.flip(frame, 1)

            inference_start = time.perf_counter()
            boxes, scores, class_ids, raw_shape = run_inference(
                rknn,
                frame,
                arguments.conf,
                arguments.nms,
            )
            inference_ms = (time.perf_counter() - inference_start) * 1000.0

            if not output_shape_logged:
                print("RKNN原始输出shape：{}".format(raw_shape))
                output_shape_logged = True

            instant_fps = 1000.0 / max(inference_ms, 1e-6)
            if smoothed_fps <= 0.0:
                smoothed_fps = instant_fps
            else:
                smoothed_fps = smoothed_fps * 0.9 + instant_fps * 0.1

            if len(class_ids) == 0:
                stable.reset()
                terminal_result = "未检测到"
            else:
                best_class_id = int(class_ids[0])
                best_score = float(scores[0])
                terminal_result = "{}({}) {:.3f}，稳定={}/{}".format(
                    CLASS_NAMES_CN[best_class_id],
                    CLASS_NAMES[best_class_id],
                    best_score,
                    stable.count + 1
                    if stable.last_class_id == best_class_id
                    else 1,
                    stable.required_frames,
                )

                if stable.update(best_class_id):
                    print(
                        "[方向确认] {}，score={:.3f}".format(
                            CLASS_NAMES_CN[best_class_id],
                            best_score,
                        )
                    )

            print(
                "\r检测：{}；推理={:.1f}ms，FPS={:.1f}      ".format(
                    terminal_result,
                    inference_ms,
                    smoothed_fps,
                ),
                end="",
                flush=True,
            )

            display = frame.copy()
            draw_detections(display, boxes, scores, class_ids)
            cv2.putText(
                display,
                "Inference: {:.1f} ms  FPS: {:.1f}".format(
                    inference_ms,
                    smoothed_fps,
                ),
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )

            if arguments.headless:
                continue

            cv2.imshow("YOLOv5 Traffic Light RKNN Test", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                print("\n收到退出按键")
                break
            if key == ord("s"):
                print()
                save_frame(display, arguments.save_dir)
            elif key == ord("r"):
                stable.reset()
                print("\n已清空连续帧稳定计数")

    except KeyboardInterrupt:
        print("\n收到Ctrl+C，程序退出")
    except Exception as error:
        print("\n测试程序异常：{}".format(error))
        return 1
    finally:
        if camera is not None:
            camera.release()
        if rknn is not None:
            rknn.release()
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except cv2.error:
            pass

    print("YOLOv5红绿灯测试结束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
