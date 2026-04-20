import os
import pandas as pd
import matplotlib.pyplot as plt

def smooth_data(data, smoothing_factor=0.6):
    """对数据应用指数移动平均进行平滑处理。
    参数：
        data: 需要平滑的原始数据，类型为 pandas Series。
        smoothing_factor: 平滑因子，介于 0 和 1 之间，值越大，平滑效果越明显。
    返回：
        平滑后的数据，类型为 pandas Series。
    """
    return data.ewm(alpha=1 - smoothing_factor).mean()

def plot_metrics_in_row(experiment_names, metrics_to_plot, metric_titles, base_directory='实验结果',
                        experiment_labels=None, figure_size=(18, 6), smoothing=False, smoothing_factor=0.6):
    # 检查实验标签数量是否匹配
    if experiment_labels is not None and len(experiment_labels) != len(experiment_names):
        raise ValueError(f"实验标签数量（{len(experiment_labels)}）与实验数量（{len(experiment_names)}）不匹配")

    # 如果未提供标签，则使用实验名称
    if experiment_labels is None:
        experiment_labels = experiment_names

    # 创建绘图对象，指定图像大小和子图布局
    fig, axs = plt.subplots(1, len(metrics_to_plot), figsize=figure_size)

    # 如果只有一个指标，axs不是数组，需要转换为数组
    if len(metrics_to_plot) == 1:
        axs = [axs]

    # 为每个指标创建子图
    for idx, (metric, title) in enumerate(zip(metrics_to_plot, metric_titles)):
        ax = axs[idx]
        all_metric_data = []  # 用于存储所有实验的指标数据，以便确定 y 轴范围
        for i, name in enumerate(experiment_names):
            file_path = os.path.join(base_directory, name, 'results.csv')
            if not os.path.exists(file_path):
                raise FileNotFoundError(f"实验 '{name}' 的结果文件未找到，路径为 '{file_path}'")

            data = pd.read_csv(file_path)
            curve_label = experiment_labels[i]
            epochs = data.index  # 假设每个 epoch 对应一行

            # 提取所需的指标数据
            column_name = [col for col in data.columns if col.strip() == metric]
            if not column_name:
                raise ValueError(f"在实验 '{name}' 的数据列中未找到指标 '{metric}'")
            metric_data = data[column_name[0]]

            # 对数据进行平滑处理（如果需要）
            if smoothing and 'mAP50' in metric:
                metric_data = smooth_data(metric_data, smoothing_factor)

            all_metric_data.append(metric_data)

            # 绘制曲线
            ax.plot(epochs, metric_data, label=curve_label)

        ax.set_title(title)
        ax.set_xlabel('Epoch')
        ax.legend()

        # 动态设置 y 轴范围
        combined_data = pd.concat(all_metric_data)
        y_min, y_max = combined_data.min(), combined_data.max()
        y_range = y_max - y_min
        y_padding = y_range * 0.1  # 在数据范围的基础上增加10%的填充
        ax.set_ylim(y_min - y_padding, y_max + y_padding)

        # 设置 y 轴标签
        if 'loss' in metric.lower():
            ax.set_ylabel('Loss')
        elif 'mAP' in metric:
            ax.set_ylabel('mAP')

    plt.tight_layout()
    # 保存和显示图像
    filename = 'metrics_row_plot.png'
    plt.savefig(filename)
    plt.show()

# 要绘制的指标及其标题
metrics_to_plot = [
    'train/box_loss',       # 训练框损失
    'train/cls_loss',       # 训练分类损失
]

metric_titles = [
    'Training Box Loss',
    'Training Classification Loss',
]

# 实验名称（结果文件夹名称）
experiment_names = ['exp17', 'exp38', 'exp33', 'exp37', 'exp21']

# 实验的标签（用于图例）
experiment_labels = ['YOLOv11', 'Gold-YOLO', 'YOLOv10', 'YOLOv8', 'EMSD-YOLO']

# 调用函数绘制指标，并对 mAP 曲线进行平滑处理
plot_metrics_in_row(
    experiment_names=experiment_names,
    metrics_to_plot=metrics_to_plot,
    metric_titles=metric_titles,
    base_directory='实验结果',
    experiment_labels=experiment_labels,
    smoothing=True,          # 启用平滑处理
    smoothing_factor=0.00    # 平滑因子，可根据需要调整
)
