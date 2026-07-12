import cv2
import os
import time
import torch
import argparse
import rospy
from nanodet.util import cfg, load_config, Logger
from nanodet.model.arch import build_model
from nanodet.util import load_model_weight
from nanodet.data.transform import Pipeline
from nanodet.data.collate import naive_collate
from nanodet.data.batch_process import stack_batch_img
m = 0
image_ext = ['.jpg', '.jpeg', '.webp', '.bmp', '.png']
video_ext = ['mp4', 'mov', 'avi', 'mkv']


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('demo', default='image', help='demo type, eg. image, video and webcam')
    parser.add_argument('--config', help='model config file path')
    parser.add_argument('--model', help='model file path')
    parser.add_argument('--path', default='./demo', help='path to images or video')
    parser.add_argument('--camid', type=int, default=0, help='webcam demo camera id')
    parser.add_argument('--save_result', action='store_true', help='whether to save the inference result of image/video')
    args = parser.parse_args()
    return args


class Predictor(object):
    def __init__(self, cfg, model_path, logger, device='cpu'):
        self.cfg = cfg
        self.device = device
        model = build_model(cfg.model)
        ckpt = torch.load(model_path, map_location=lambda storage, loc: storage)
        load_model_weight(model, ckpt, logger)
        if cfg.model.arch.backbone.name == 'RepVGG':
            deploy_config = cfg.model
            deploy_config.arch.backbone.update({'deploy': True})
            deploy_model = build_model(deploy_config)
            from nanodet.model.backbone.repvgg import repvgg_det_model_convert
            model = repvgg_det_model_convert(model, deploy_model)
        self.model = model.to(device).eval()
        self.pipeline = Pipeline(cfg.data.val.pipeline, cfg.data.val.keep_ratio)

    def inference(self, img):
        img_info = {"id": 0}
        if isinstance(img, str):
            img_info["file_name"] = os.path.basename(img)
            img = cv2.imread(img)
        else:
            img_info["file_name"] = None

        height, width = img.shape[:2]
        img_info["height"] = height
        img_info["width"] = width
        meta = dict(img_info=img_info, raw_img=img, img=img)
        meta = self.pipeline(None, meta, self.cfg.data.val.input_size)
        meta["img"] = torch.from_numpy(meta["img"].transpose(2, 0, 1)).to(self.device)
        meta = naive_collate([meta])
        meta["img"] = stack_batch_img(meta["img"], divisible=32)
        with torch.no_grad():
            results = self.model.inference(meta)
        return meta, results

    def visualize(self, dets, meta, class_names, score_thres, wait=0):
        time1 = time.time()
        result_img = self.model.head.show_result(
            meta["raw_img"][0], dets, class_names, score_thres=score_thres, show=True
        )
        print("viz time: {:.3f}s".format(time.time() - time1))
        return result_img


def get_image_list(path):
    image_names = []
    for maindir, subdir, file_name_list in os.walk(path):
        for filename in file_name_list:
            apath = os.path.join(maindir, filename)
            ext = os.path.splitext(apath)[1]
            if ext in image_ext:
                image_names.append(apath)
    return image_names

def Histogram(frame):
    (b, g, r) = cv2.split(frame)
    bH = cv2.equalizeHist(b)
    gH = cv2.equalizeHist(g)
    rH = cv2.equalizeHist(r)
    result = cv2.merge((bH, gH, rH))
    return result

def detect(img, predictor):
    """
    标准化输出格式为: {class_id: [[x1, y1, x2, y2, score], ...], ...}
    替代原来的多层嵌套字典结构
    """
    # 获取原始预测结果
    meta, raw_result = predictor.inference(img)
    
    # 初始化结果字典
    detections = {}
    
    # 处理原始结果（假设raw_result是字典结构）
    if isinstance(raw_result, dict):
        # 遍历所有batch（通常只有batch=0）
        for batch_idx, class_dict in raw_result.items():
            # 如果 raw_result 包含 batch 维度，且每个 batch 的结构相同，可以只处理 batch_idx=0
            if batch_idx != 0:
                continue
            # 遍历所有类别
            for class_id, boxes in class_dict.items():
                class_id = int(class_id)  # 确保 class_id 是整数
                # 初始化当前类别的检测列表（如果尚未存在）
                if class_id not in detections:
                    detections[class_id] = []
                # 遍历该类别的所有检测框
                for box in boxes:
                    if len(box) >= 5:  # 确保数据完整
                        detections[class_id].append([
                            float(box[0]),  # x1
                            float(box[1]),  # y1
                            float(box[2]),  # x2
                            float(box[3]),  # y2
                            float(box[4]),  # score
                            # 如果需要 class_id，可以在这里添加，但通常在后续处理中已经知道
                            # int(class_id)
                        ])
                # 如果需要移除 score，只保留 [x1, y1, x2, y2]，可以调整如下：
                # detections[class_id].append([
                #     float(box[0]),  # x1
                #     float(box[1]),  # y1
                #     float(box[2]),  # x2
                #     float(box[3]),  # y2
                # ])
    
    # 如果需要按类别排序（可选）
    sorted_detections = {k: detections[k] for k in sorted(detections.keys())}
    
    return sorted_detections
# def detect(frame,predictor):
# #    global m
#     # ret_val, frame = cap.read()
# #    frame = Histogram(frame)
# #    m = m+1
#     # frame = cv2.resize(frame,dsize=(960,540))  # new
#     # if ret_val:
#     meta, res = predictor.inference(frame)
# #        cv2.imwrite('picture'+str(m)+'.jpg',frame)
# #        print("success to save" + str(m) + ".jpg")
#     return res
def init():
    # args = parse_args()
    path = '/dev/video0'
    # path = '0'
    model = '/home/ucar/ucar_ws_copy/src/nanodet/src/rgl_model.pth'
    config = '/home/ucar/ucar_ws_copy/src/nanodet/src/rgl_model.yml'
    torch.backends.cudnn.enabled = True
    torch.backends.cudnn.benchmark = True
    load_config(cfg, config)
    logger = Logger(-1, use_tensorboard=False)
    # predictor = Predictor(cfg, model, logger, device='cuda:0')
    predictor = Predictor(cfg, model, logger, device='cpu')
    # if demo == 'video' or demo == 'webcam':
    # cap = cv2.VideoCapture(path)
    # cap = cv2.VideoCapture(path)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH,960)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT,540)
    return predictor


def main():
    args = parse_args()
    torch.backends.cudnn.enabled = True
    torch.backends.cudnn.benchmark = True

    load_config(cfg, args.config)
    logger = Logger(-1, use_tensorboard=False)
    predictor = Predictor(cfg, args.model, logger, device='cpu')  # shibie
    logger.log('Press "Esc", "q" or "Q" to exit.')
    current_time = time.localtime()
    if args.demo == 'image':
        if os.path.isdir(args.path):
            files = get_image_list(args.path)
        else:
            files = [args.path]
        files.sort()
        for image_name in files:
            meta, res = predictor.inference(image_name)
            result_image = predictor.visualize(res, meta, cfg.class_names, 0.35)
            if args.save_result:
                save_folder = os.path.join(cfg.save_dir, time.strftime("%Y_%m_%d_%H_%M_%S", current_time))
                if not os.path.exists(save_folder):
                    os.mkdir(save_folder)
                save_file_name = os.path.join(save_folder, os.path.basename(image_name))
                cv2.imwrite(save_file_name, result_image)
            ch = cv2.waitKey(0)
            if ch == 27 or ch == ord('q') or ch == ord('Q'):
                break
    elif args.demo == 'video' or args.demo == 'webcam':
        cap = cv2.VideoCapture(args.path if args.demo == 'video' else args.camid)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,960)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,540)
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  # float
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  # float
        print(cv2.CAP_PROP_FRAME_WIDTH,cv2.CAP_PROP_FRAME_HEIGHT)
        print(width,height)
        fps = cap.get(cv2.CAP_PROP_FPS)
        save_folder = os.path.join(cfg.save_dir, time.strftime("%Y_%m_%d_%H_%M_%S", current_time))
        if not os.path.exists(save_folder):
            os.mkdir(save_folder)
        save_path = os.path.join(save_folder, args.path.split('/')[-1]) if args.demo == 'video' else os.path.join(save_folder, 'camera.mp4')
        # print(f'save_path is {save_path}')
        print('save_path is {}'.format(save_path))
        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (int(width), int(height)))
        while True:
            ret_val, frame = cap.read()
            #frame = Histogram(frame)
            k = cv2.waitKey(1)
            if k == ord('s'):
                cv2.imwrite('picture'+str(m)+'.jpg',frame)
                print("success to save" + str(m) + ".jpg")
                m = m+1
            if ret_val:
                meta, res = predictor.inference(frame)
                result_frame = predictor.visualize(res, meta, cfg.class_names, 0.4)
                #if args.save_result:
                    #vid_writer.write(result_frame)
                ch = cv2.waitKey(1)
                if ch == 27 or ch == ord('q') or ch == ord('Q'):
                    break
            else:
                break


if __name__ == '__main__':
    main()
