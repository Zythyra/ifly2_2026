# -*- coding: utf-8 -*-

from rknnlite.api import RKNNLite


class RKNN_model_container:
    """
    RK3588/RK3588S开发板专用RKNNLite执行器。

    对外接口保持与原rknn_executor.py一致：
    1. 初始化模型
    2. run(inputs)
    3. release()
    """

    def __init__(
        self,
        model_path,
        target=None,
        device_id=None,
    ):
        # 这两个参数是电脑端RKNN接口使用的。
        # 板端RKNNLite不需要，但保留形参以兼容原调用方式。
        self.target = target
        self.device_id = device_id
        self.rknn = RKNNLite()

        print("--> Load RKNN model")

        ret = self.rknn.load_rknn(model_path)

        if ret not in (0, None):
            self.rknn.release()
            raise RuntimeError(
                f"加载RKNN模型失败，返回值：{ret}"
            )

        print("done")

        print("--> Init RKNNLite runtime")

        # 独立测试阶段让系统自动选择NPU核心。
        # 后续和NanoDet联调时再调整核心分配。
        try:
            ret = self.rknn.init_runtime(
                core_mask=RKNNLite.NPU_CORE_AUTO
            )
        except TypeError:
            # 兼容某些旧版Lite2接口。
            ret = self.rknn.init_runtime()

        if ret not in (0, None):
            self.rknn.release()
            raise RuntimeError(
                f"初始化RKNNLite失败，返回值：{ret}"
            )

        print("done")

    def run(self, inputs):
        if isinstance(inputs, tuple):
            inputs = list(inputs)
        elif not isinstance(inputs, list):
            inputs = [inputs]

        outputs = self.rknn.inference(
            inputs=inputs
        )

        if outputs is None:
            raise RuntimeError(
                "RKNNLite推理返回None"
            )

        return outputs

    def release(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None
