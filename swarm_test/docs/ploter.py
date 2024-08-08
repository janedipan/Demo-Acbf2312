import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# 获取当前脚本的目录
script_dir = os.path.dirname(os.path.abspath(__file__))

# 绘制距离-时间曲线图
def plot1():
    csv_file = script_dir + "/data0729/data_copy1.csv"

    df = pd.read_csv(csv_file)

    x1 = df.iloc[4]; y1 = df.iloc[5] # N+DCBF:1,5
    x2 = df.iloc[2]; y2 = df.iloc[3] # A+ACBF:3 

    x_r = [i*0.02-0.09 for i in range(y1.size)]

    palettes = sns.color_palette("hls", 8)
    plotsize = (9,5)
    # 创建图形和轴
    sns.set_theme(style="whitegrid", rc={'font.family':'serif', "axes.labelsize":15})
    fig, axs = plt.subplots(figsize = plotsize)
    axs.set_xlabel("Time [s]")
    axs.set_ylabel("Obstacle Distance [m]")
    axs.set_xlim(2.5, 5.5)
    axs.set_ylim(0, 4.0)

    axs.plot(x_r, 0.8*np.ones(len(x_r)), linestyle="--", color="red", linewidth=3, label='Min safe distance')

    # 绘制折线图
    sns.lineplot(x=x_r, y=y1, label='N+DCBF', linewidth=3)
    sns.lineplot(x=x2, y=y2, label='Proposed A+ACBF', linewidth=3)

    # 设置网格线
    axs.grid(True)

    # 添加图例
    legend = plt.legend(loc='upper right')
    for text in legend.get_texts():
        text.set_fontsize(15)
    legend.set_visible(True)

    # 显示图形
    plt.show()


# 绘制距离直方图
def plot2():
    csv_file = script_dir + "/data0729/data_copy2.csv"
    df = pd.read_csv(csv_file)

    nums = 6.25/0.125
    x_value = [i*0.125 for i in range(int(nums))]
    y1_value = [0 for i in range(int(nums))]
    y2_value = [0 for i in range(int(nums))]

    y1_data = df.iloc[0].tolist() # A+ACBF:0
    y2_data = df.iloc[2].tolist() # N+DCBF:1,2
    for i in range(len(y1_data)):
        index = int(y1_data[i]/0.125)
        y1_value[index] += 1

    for i in range(len(y2_data)):
        index = int(y2_data[i]/0.125)
        y2_value[index] += 1
    
    palettes = sns.color_palette("hls", 8)

    # 创建两个DataFrame
    df1 = pd.DataFrame({
        'category': x_value,
        'value': y1_value
    })

    df2 = pd.DataFrame({
        'category': x_value,
        'value': y2_value
    })

    # 创建一个图形和两个轴对象
    sns.set_theme(style="whitegrid", rc={'font.family':'serif', "axes.labelsize":15})
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5))  # 2行1列的子图布局
    
    # 在第一个子图上绘制df的柱状图
    sns.barplot(ax=ax1, x='category', y='value', data=df1, color=palettes[0], edgecolor='black', width=1.0, label='Proposed A+ACBF')
    # 在第二个子图上绘制df2的柱状图
    sns.barplot(ax=ax2, x='category', y='value', data=df2, color=palettes[5], edgecolor='black', width=1.0, label='N+DCBF')

    # 隐藏 ax1, ax2 子图的 x 轴和 y 轴标签
    ax1.set_xlabel("")
    ax1.set_ylabel("")
    ax2.set_xlabel("")
    ax2.set_ylabel("")
    # 设置 x 轴刻度
    ax1.set_xticks([i for i in range(0, len(x_value), 8)])  # 每隔2个单位显示一个刻度
    ax2.set_xticks([i for i in range(0, len(x_value), 8)])  # 每隔2个单位显示一个刻度

    # 设置x，y轴刻度线
    # ax1.grid(True)

    fig.supxlabel('Distance [m]')
    fig.supylabel('Number of Samples')

    # 添加图例
    # 添加图例到 ax1，并设置图例字体大小
    legend1 = ax1.legend(prop={'size': 15})
    # 添加图例到 ax2，并设置图例字体大小
    legend2 = ax2.legend(prop={'size': 15})
    legend1.set_visible(True)
    legend2.set_visible(True)

    # 调整子图间距
    plt.tight_layout()

    # 显示图表
    plt.show()

def plot3():
    csv_file = script_dir + "/data0807/data.csv"
    df = pd.read_csv(csv_file)
    y1 = df.iloc[0].tolist() # 前端路径搜索时间
    y2 = df.iloc[1].tolist() # 后端时间-无障碍物
    y3 = df.iloc[2].tolist() # 后端时间-有障碍物

    y1_u = [i/1000.0 for i in y1]
    y2_u = [i/1000.0 for i in y2]
    y3_u = [i/1000.0 for i in y3]

    # 将数组放入一个字典中，并为每组数据指定一个标签
    data_dict = {
        'Path searching': y1_u,
        'MPC w/o obstacle': y2_u,
        'MPC w/ obstacle': y3_u
    }

    df_use = pd.DataFrame(data_dict)

    # 使用NumPy的std函数计算标准差  
    std_path_searching = np.std(data_dict['Path searching'])  
    std_mpc_wo_obstacle = np.std(data_dict['MPC w/o obstacle'])  
    std_mpc_w_obstacle = np.std(data_dict['MPC w/ obstacle'])  
    
    print("Path searching standard deviation:", std_path_searching)  
    print("MPC w/o obstacle standard deviation:", std_mpc_wo_obstacle)  
    print("MPC w obstacle standard deviation:", std_mpc_w_obstacle)

    # 使用NumPy计算均值  
    mean_path_searching = np.mean(data_dict['Path searching'])  
    mean_mpc_wo_obstacle = np.mean(data_dict['MPC w/o obstacle'])  
    mean_mpc_w_obstacle = np.mean(data_dict['MPC w/ obstacle'])  
    
    print("Path searching mean:", mean_path_searching)  
    print("MPC w/o obstacle mean:", mean_mpc_wo_obstacle)  
    print("MPC w obstacle mean:", mean_mpc_w_obstacle)

    sns.set_theme(style="whitegrid", rc={'font.family':'serif', "axes.labelsize":15})
    fig, ax1 = plt.subplots(figsize=(9, 5)) 
    ax1.set_ylabel("Runtime [s]", fontsize=15)
    ax1.tick_params(axis='x', labelsize=15)  # 设置x轴刻度的字体大小  
    ax1.tick_params(axis='y', labelsize=15)

    # 绘制箱线图，指定x为分类标签，y为数据值
    sns.boxplot(data=df_use, width=0.3, color='white', linewidth=2,ax=ax1, 
                medianprops=dict(color='orange', linewidth=2),
                flierprops=dict(marker='x', markeredgecolor='red', markersize=8))

    ax1.set_ylim(0, 0.12)
    # 设置x，y轴刻度线
    ax1.grid(True)

    # 显示图表
    plt.show()

def plot_test():
    csv_file1 = script_dir + "/data0807/data-front.csv"
    df1 = pd.read_csv(csv_file1)
    y1 = df1.iloc[0].tolist() # 前端路径搜索时间

    df2 = pd.read_csv(script_dir + "/data0807/data-end.csv")
    y2_t = df2.iloc[4].tolist() # 后端路径搜索标记
    y2_v = df2.iloc[5].tolist() # 后端路径搜索时间

    y2 = [] # 后端时间-无障碍物
    y3 = [] # 后端时间-有障碍物
    for i in range(len(y2_t)):
        if (y2_t[i] == 1):
            if y2_v[i]>70.0:
                y3.append(y2_v[i])
        else:
            y2.append(y2_v[i])
    # print(len(y1))5
    print(len(y2))
    print(y2)
    # print(len(y3))
    # print(y3)


if __name__ == '__main__':
    plot1()
