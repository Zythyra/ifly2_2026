from .gfl import GFL
from .one_stage import OneStage
import copy
import warnings
from .one_stage import OneStage

def build_model(model_cfg):
    # if model_cfg.arch.name == 'GFL':
    #     model = GFL(model_cfg.arch.backbone, model_cfg.arch.fpn, model_cfg.arch.head)
    # elif model_cfg.arch.name == 'OneStageDetector':
    #     model = OneStage(model_cfg.arch.backbone, model_cfg.arch.fpn, model_cfg.arch.head)
    # else:
    #     raise NotImplementedError
    # return model
    model_cfg = copy.deepcopy(model_cfg)
    name = model_cfg.arch.pop("name")
    if name == "GFL":
        warnings.warn(
            "Model architecture name is changed to 'OneStageDetector'. "
            "The name 'GFL' is deprecated, please change the model->arch->name "
            "in your YAML config file to OneStageDetector."
        )
        model = OneStage(
            model_cfg.arch.backbone, model_cfg.arch.fpn, model_cfg.arch.head
        )
    elif name == "OneStageDetector":
        model = OneStage(
            model_cfg.arch.backbone, model_cfg.arch.fpn, model_cfg.arch.head
        )
    else:
        raise NotImplementedError
    return model