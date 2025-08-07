#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from statsmodels.nonparametric.smoothers_lowess import lowess
from statsmodels.robust.robust_linear_model import RLM
from statsmodels.stats.diagnostic import het_breuschpagan
import statsmodels.api as sm
from scipy import stats
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String, Bool
import threading
import time

# 全局变量用于存储数据
raw_data = None
processed_data = None
fit_result = None
data_lock = threading.Lock()

# 1. 数据预处理：识别并降低极端异常值的影响
def preprocess_data(x, y, z_threshold=3.0):
    """通过Z-score方法识别极端异常值并进行处理"""
    # 计算Z-score
    z_scores = np.abs(stats.zscore(y))

    # 标记非异常值
    non_outlier_mask = z_scores < z_threshold

    # 获取上下分位数
    upper_limit = np.percentile(y[non_outlier_mask], 95)
    lower_limit = np.percentile(y[non_outlier_mask], 5)

    # 对极端异常值进行截断处理而非直接删除，保留数据分布
    y_processed = y.copy()
    y_processed[z_scores >= z_threshold] = upper_limit
    y_processed[z_scores <= -z_threshold] = lower_limit

    return x, y_processed


# 2. 生成测试数据（含异常值）
def generate_data(data_type="linear", noise_level=0.5, outlier_strength=8):
    x = np.linspace(0, 10, 100)
    np.random.seed(42)  # 固定随机种子，保证结果可复现

    if data_type == "linear":
        # 线性数据（加噪声和异常值）
        y_true = 2 * x + 3
        y = y_true + np.random.normal(0, noise_level, 100)  # 高斯噪声
        y[10:15] += outlier_strength  # 添加异常值
    else:
        # 非线性数据（更复杂的曲线）
        y_true = np.sin(x) * 5 + x * 0.5 + np.cos(x / 2) * 3
        y = y_true + np.random.normal(0, noise_level, 100)
        y[20:25] -= outlier_strength  # 添加异常值
        y[60:65] += outlier_strength * 0.8  # 添加更多异常值

    return x, y, y_true


# 3. 线性趋势检验
def test_linearity(x, y):
    x_with_const = sm.add_constant(x)

    # 用稳健线性回归拟合
    rlm = RLM(y, x_with_const)
    results = rlm.fit()
    y_pred = results.predict(x_with_const)

    # 计算残差
    residuals = y - y_pred

    # 结合Breusch-Pagan检验和残差平方和检验
    bp_test = het_breuschpagan(residuals, x_with_const)
    p_value = bp_test[1]

    # 计算残差平方和，作为辅助判断
    sse = np.sum(residuals ** 2)
    sst = np.sum((y - np.mean(y)) ** 2)
    r_squared = 1 - (sse / sst)  # 决定系数，越接近1线性越好

    # 综合判断：p值 >= 0.05 且 决定系数 >= 0.7 认为是线性
    is_linear = (p_value >= 0.05) and (r_squared >= 0.7)
    return is_linear, results, x_with_const, r_squared


# 4. 改进的Lowess拟合（针对非线性数据优化）
def improved_lowess(x, y, data_complexity=0.5):
    """
    data_complexity: 数据复杂度 (0-1)，值越高表示数据越复杂，需要更灵活的拟合
                    0.1-0.3: 简单非线性
                    0.4-0.6: 中等复杂度
                    0.7-0.9: 高复杂度
    """
    # 根据数据复杂度自动调整参数
    if data_complexity < 0.4:
        frac = 0.4  # 带宽，值越小曲线越灵活
        it = 3  # 迭代次数，影响稳健性
        delta = 0.01  # 控制平滑度
    elif data_complexity < 0.7:
        frac = 0.3
        it = 4
        delta = 0.005
    else:
        frac = 0.2
        it = 5
        delta = 0.001

    # 应用Lowess拟合
    lowess_result = lowess(
        y, x,
        frac=frac,  # 带宽参数，控制使用数据的比例
        it=it,  # 稳健迭代次数
        delta=delta,  # 控制x方向的平滑度
        is_sorted=True  # 假设x是排序的，加速计算
    )

    return lowess_result[:, 1]


# 5. 自适应拟合主函数
def adaptive_fit(x, y, manual_complexity=None):
    # 数据预处理
    x_processed, y_processed = preprocess_data(x, y)

    # 线性检验
    is_linear, linear_model, x_with_const, r_squared = test_linearity(x_processed, y_processed)

    if is_linear:
        # 线性数据：用稳健线性回归
        y_fit = linear_model.predict(x_with_const)
        model_name = "稳健线性回归"
    else:
        # 非线性数据：用改进的Lowess
        # 如果未指定复杂度，则根据残差自动估计
        if manual_complexity is None:
            # 残差越大，数据可能越复杂
            complexity = min(0.9, max(0.1, 1 - r_squared))
        else:
            complexity = manual_complexity

        y_fit = improved_lowess(x_processed, y_processed, complexity)
        model_name = f"稳健Lowess (复杂度: {complexity:.1f})"

    return y_fit, model_name, is_linear, x_processed, y_processed


# 6. 可视化结果
def plot_results(x, y, y_true, y_fit, model_name, is_linear, ax=None):
    if ax is None:
        ax = plt.gca()
    
    ax.clear()
    
    # 绘制原始数据和异常值
    z_scores = np.abs(stats.zscore(y))
    outliers = z_scores >= 3.0

    ax.scatter(x[~outliers], y[~outliers], alpha=0.6,
               label="有效数据", color="lightblue")
    ax.scatter(x[outliers], y[outliers], alpha=0.6,
               label="异常值", color="red", marker="x")

    # 绘制真实趋势和拟合曲线
    ax.plot(x, y_true, "k--", label="真实趋势", linewidth=1.5)
    ax.plot(x, y_fit, "r-", linewidth=2.5, label=model_name)

    ax.set_title(f"拟合结果：{'线性数据' if is_linear else '非线性数据'}")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.legend()
    ax.grid(alpha=0.3)
    
    return ax


# 数据生成节点
def data_generator_node():
    global raw_data
    rospy.init_node('data_fitting_system', anonymous=True)
    
    # 获取参数
    data_type = rospy.get_param('~data_type', 'linear')
    noise_level = rospy.get_param('~noise_level', 0.5)
    outlier_strength = rospy.get_param('~outlier_strength', 8)
    update_rate = rospy.get_param('~update_rate', 1.0)  # 数据更新频率(Hz)
    
    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        x, y, y_true = generate_data(data_type, noise_level, outlier_strength)
        
        with data_lock:
            raw_data = (x, y, y_true)
            
        rospy.loginfo(f"生成{data_type}数据，共{len(x)}个样本")
        rate.sleep()


# 数据处理和拟合节点
def processing_node():
    global processed_data, fit_result, raw_data
    
    while not rospy.is_shutdown():
        with data_lock:
            current_raw = raw_data
        
        if current_raw is not None:
            x, y, y_true = current_raw
            y_fit, model_name, is_linear, x_processed, y_processed = adaptive_fit(x, y)
            
            with data_lock:
                processed_data = (x_processed, y_processed)
                fit_result = (y_fit, model_name, is_linear)
            
            rospy.loginfo(f"完成拟合：{model_name}")
        
        time.sleep(0.1)  # 短暂休眠，减少CPU占用


# 可视化节点
def visualization_node():
    global raw_data, fit_result
    
    plt.ion()  # 开启交互模式
    fig, ax = plt.subplots(figsize=(12, 7))
    plt.tight_layout()
    
    while not rospy.is_shutdown():
        with data_lock:
            current_raw = raw_data
            current_fit = fit_result
        
        if current_raw is not None and current_fit is not None:
            x, y, y_true = current_raw
            y_fit, model_name, is_linear = current_fit
            
            plot_results(x, y, y_true, y_fit, model_name, is_linear, ax)
            plt.draw()
        
        plt.pause(0.1)  # 短暂暂停，允许图形刷新


if __name__ == '__main__':
    try:
        # 启动数据生成线程
        generator_thread = threading.Thread(target=data_generator_node, daemon=True)
        generator_thread.start()
        
        # 启动处理线程
        processing_thread = threading.Thread(target=processing_node, daemon=True)
        processing_thread.start()
        
        # 启动可视化线程（主线程）
        visualization_node()
        
    except rospy.ROSInterruptException:
        plt.close('all')
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")
        plt.close('all')
