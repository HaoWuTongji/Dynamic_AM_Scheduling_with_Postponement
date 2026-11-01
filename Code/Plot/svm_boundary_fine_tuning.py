
"""
Author: Hao Wu
Description:
Standalone script to fine-tune the SVM decision boundaries for ALL sensitivity analysis groups.
Reads from 'Sensitivity_Analysis_Results.xlsx'.
Allows independent tuning of C and gamma for each of the 9 curves to ensure maximum academic smoothness.
Outputs both Individual Scatter-Boundary plots and Comparative Overlay plots.
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.svm import SVC
import os
from matplotlib.lines import Line2D

# ==============================================================================
# 1. 核心调节区：独立调整三大实验组（9条曲线）的 SVM 超参数
# ==============================================================================
# 调参指南：
# C (对错判的惩罚): 越小曲线越平滑泛化，越大曲线越会扭曲去迎合少数异常点。
# gamma (核影响半径): 越小曲线越趋近于平滑的全局弧线，越大越容易在局部画出“一个个的圈”。
# 对于极度失衡的数据（例如 K_fine = 50 只有极少量的推迟点），请务必降低 C 和 gamma！

svm_hyperparams = {
    'Arrival_Rate': {
        '0.5': {'C': 5.0, 'gamma': 'scale'},  # \lambda / 2
        'Base': {'C': 5.0, 'gamma': 'scale'},  # Baseline \lambda
        '2.0': {'C': 5.0, 'gamma': 'scale'}  # 2\lambda
    },
    'Operator_Cost': {
        '100': {'C': 5.0, 'gamma': 'scale'},  # C_opr = 100
        '300': {'C': 5.0, 'gamma': 'scale'},  # Baseline C_opr = 300
        '500': {'C': 5.0, 'gamma': 'scale'}  # C_opr = 500
    },
    'Tardiness_Penalty': {
        '10': {'C': 5.0, 'gamma': 'scale'},  # K_fine = 10
        '30': {'C': 5.0, 'gamma': 'scale'},  # Baseline K_fine = 30
        '50': {'C': 5.0, 'gamma': 'scale'}  # K_fine = 50
    }
}

# ==============================================================================
# 2. 严谨的学术标签字典
# ==============================================================================
ACADEMIC_LABELS = {
    'Arrival_Rate': {
        '0.5': r'$\lambda / 2$',
        'Base': r'$\lambda$',
        '2.0': r'$2\lambda$'
    },
    'Operator_Cost': {
        '100': r'$C_{opr} = 100$',
        '300': r'$C_{opr} = 300$',
        '500': r'$C_{opr} = 500$'
    },
    'Tardiness_Penalty': {
        '10': r'$K_{fine} = 10$',
        '30': r'$K_{fine} = 30$',
        '50': r'$K_{fine} = 50$'
    }
}


def extract_key(group_name, label):
    """从杂乱的列名中提取配置 Key"""
    label_lower = str(label).lower()
    if group_name == 'Arrival_Rate':
        if '2' in label_lower and '/' in label_lower:
            return '0.5'
        elif '2' in label_lower:
            return '2.0'
        else:
            return 'Base'
    elif group_name == 'Operator_Cost':
        if '100' in label_lower:
            return '100'
        elif '500' in label_lower:
            return '500'
        else:
            return '300'
    elif group_name == 'Tardiness_Penalty':
        if '10' in label_lower:
            return '10'
        elif '50' in label_lower:
            return '50'
        else:
            return '30'
    return 'Base'


def load_all_data(filepath):
    """鲁棒的数据加载器，兼容所有的输出格式并映射学术标签"""
    df = pd.read_excel(filepath)
    all_datasets = {'Arrival_Rate': [], 'Operator_Cost': [], 'Tardiness_Penalty': []}

    # 兼容宽表格式 (Wide Format)
    for group_name in all_datasets.keys():
        util_col = f"{group_name}_ActualUtil"
        if util_col not in df.columns:
            util_col = 'Actual_Util(%)' if 'Actual_Util(%)' in df.columns else 'Target_Util(%)'

        action_cols = [c for c in df.columns if group_name in c and 'Action' in c]
        for action_col in action_cols:
            X = df[[util_col, 'Target_Slack(h)']].values
            y_str = df[action_col].values

            # 处理字符串形式的动作
            if isinstance(y_str[0], str):
                y = np.array([1 if val == 'Produce' else 0 for val in y_str])
            else:
                y = y_str  # 已经是整数形式 1, 0

            key = extract_key(group_name, action_col)
            label = ACADEMIC_LABELS[group_name][key]

            all_datasets[group_name].append({'label': label, 'X': X, 'y': y, 'key': key})

    # 强制固定绘图顺序：低、基准、高
    sort_order = {'0.5': 0, '100': 0, '10': 0, 'Base': 1, '300': 1, '30': 1, '2.0': 2, '500': 2, '50': 2}
    for group_name in all_datasets:
        all_datasets[group_name] = sorted(all_datasets[group_name], key=lambda d: sort_order.get(d['key'], 99))

    return all_datasets


def plot_individual_boundary(X, y, label, filename, svm_C, svm_gamma):
    """绘制带有散点和背景填充的单组实验边界图"""
    sns.set_theme(style="whitegrid", context="paper", font_scale=1.4)
    plt.rcParams["font.family"] = "serif"

    fig, ax = plt.subplots(figsize=(10, 8), dpi=300)
    clf = SVC(kernel='rbf', C=svm_C, gamma=svm_gamma)

    # 确保至少有两个类别的数据才能拟合 SVM
    if len(np.unique(y)) > 1:
        clf.fit(X, y)
        xx, yy = np.meshgrid(np.linspace(0, 100, 300), np.linspace(-15, 45, 300))
        Z = clf.decision_function(np.c_[xx.ravel(), yy.ravel()])
        Z = Z.reshape(xx.shape)

        ax.contour(xx, yy, Z, levels=[0], linewidths=2.5, colors='black', linestyles='--')
        ax.contourf(xx, yy, Z, levels=[0, Z.max()], colors=['#d62728'], alpha=0.1)
        ax.contourf(xx, yy, Z, levels=[Z.min(), 0], colors=['#1f77b4'], alpha=0.1)
    else:
        # 如果一侧全为某一类
        bg_color = '#d62728' if y[0] == 1 else '#1f77b4'
        ax.set_facecolor((0, 0, 0, 0))  # transparent
        ax.axhspan(-15, 45, facecolor=bg_color, alpha=0.1)

    # 绘制散点
    ax.scatter(X[y == 0, 0], X[y == 0, 1], c='#1f77b4', marker='o', s=70, edgecolors='white', label='Postpone')
    ax.scatter(X[y == 1, 0], X[y == 1, 1], c='#d62728', marker='s', s=70, edgecolors='white', label='Produce')

    ax.set_xlim(0, 100)
    ax.set_ylim(-12, 42)
    ax.set_xlabel("Area Utilization (%)", fontweight='bold', labelpad=12)
    ax.set_ylabel("Average Slack (h)", fontweight='bold', labelpad=12)
    # ax.set_title(f"Empirical Switching Boundary: {label}", pad=15, fontweight='bold')

    ax.legend(loc='upper left', frameon=True, shadow=True)
    plt.tight_layout()
    plt.savefig(filename, bbox_inches='tight')
    plt.close()
    return clf


def plot_comparative_boundaries(models_and_data, group_name, filename):
    """绘制高水平运筹学对比图 (Overlay)"""
    sns.set_theme(style="whitegrid", context="paper", font_scale=1.4)
    plt.rcParams["font.family"] = "serif"

    fig, ax = plt.subplots(figsize=(10, 8), dpi=300)

    # 学术配色与线型：蓝（低），黑（基准），红（高）
    colors = ['#1f77b4', 'black', '#d62728']
    linestyles = ['-', '--', '-.']

    xx, yy = np.meshgrid(np.linspace(0, 100, 300), np.linspace(-15, 45, 300))
    custom_lines = []
    labels = []

    for idx, item in enumerate(models_and_data):
        clf = item['model']
        label = item['label']

        # 仅当模型成功拟合时才画边界线
        if hasattr(clf, "decision_function"):
            Z = clf.decision_function(np.c_[xx.ravel(), yy.ravel()])
            Z = Z.reshape(xx.shape)
            ax.contour(xx, yy, Z, levels=[0], colors=[colors[idx]], linestyles=[linestyles[idx]], linewidths=3.0)

        custom_lines.append(Line2D([0], [0], color=colors[idx], lw=3.0, linestyle=linestyles[idx]))
        labels.append(label)

    ax.set_xlim(0, 100)
    ax.set_ylim(-12, 42)
    ax.set_xlabel("Area Utilization (%)", fontweight='bold', labelpad=12)
    ax.set_ylabel("Average Slack (h)", fontweight='bold', labelpad=12)
    # title = f"Empirical Switching Boundary Sensitivity: {group_name.replace('_', ' ')}"
    # ax.set_title(title, pad=15, fontweight='bold')

    ax.legend(custom_lines, labels, loc='lower right', frameon=True, shadow=True)#, title="Configuration"

    plt.tight_layout()
    plt.savefig(filename, bbox_inches='tight')
    plt.close()


if __name__ == '__main__':
    data_file = 'Sensitivity_Analysis_Results.xlsx'

    if not os.path.exists(data_file):
        print(f"❌ 找不到数据文件 {data_file}！")
    else:
        print(f"✅ 读取数据文件: {data_file}")
        all_datasets = load_all_data(data_file)

        output_dir = "Tuned_Boundary_Plots261009"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # 遍历三大敏感性分析组
        for group_name, datasets in all_datasets.items():
            if not datasets:
                continue

            print(f"\n🚀 开始处理组别: {group_name}")
            models_and_data = []

            # 1. 绘制并保存每一条独立的边界图
            for idx, item in enumerate(datasets):
                key = item['key']
                label = item['label']

                # 读取您设置的超参数
                svm_c = svm_hyperparams[group_name][key]['C']
                svm_gamma = svm_hyperparams[group_name][key]['gamma']

                print(f"   -> 拟合曲线 {label} (C={svm_c}, gamma={svm_gamma})")
                filename_indiv = os.path.join(output_dir, f"Individual_{group_name}_{key}.svg")

                clf = plot_individual_boundary(item['X'], item['y'], label, filename_indiv, svm_C=svm_c,
                                               svm_gamma=svm_gamma)

                models_and_data.append({'label': label, 'model': clf})

            # 2. 绘制叠加的对比图
            filename_comp_svg = os.path.join(output_dir, f"Comparative_{group_name}.svg")
            filename_comp_png = os.path.join(output_dir, f"Comparative_{group_name}.png")
            plot_comparative_boundaries(models_and_data, group_name, filename_comp_svg)
            plot_comparative_boundaries(models_and_data, group_name, filename_comp_png)
            print(f"   ✅ [对比图已保存] {filename_comp_svg}")

        print(f"\n🎉 所有图表已生成至文件夹: {output_dir}/")