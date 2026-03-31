from geom3d import * 
from math import *
import numpy as np
import matplotlib.pyplot as plt
import statistics
#主目标组
real_tgts_init_ = Point3(0,200,0)

print(real_tgts_init_.to_list())
real_tgts_list = []
xy_list = []
for i in range(-2,3,1):
    xy_list.append([7*cos(pi*i/4),7*sin(pi*i/4)])
for i in range(2,7,1):
    xy_list.append([7*cos(pi*i/4),7*sin(pi*i/4)])
for j in range(5):
    real_tgts_list.append(Point3(xy_list[j][0],xy_list[j][1]+200,0))
for j in range(5):
    real_tgts_list.append(Point3(xy_list[j+5][0],xy_list[j+5][1]+200,10))
print("当前真目标组的个数为",len(real_tgts_list))
for temp in real_tgts_list:
    print(temp.to_list())
"""
    会发现本该为0的小数,有待优化
"""
print("**************优化后**************")
def generate_real_targets()->list:
    real_tgts_list = []
    xy_list = []
    for i in range(-2,3,1):
        xy_list.append([7*cos(pi*i/4),7*sin(pi*i/4)])
    for i in range(2,7,1):
        xy_list.append([7*cos(pi*i/4),7*sin(pi*i/4)])
    for j in range(5):
        temp = Point3(xy_list[j][0],xy_list[j][1]+200,0).clean()
        real_tgts_list.append(temp)
    for j in range(5):
        temp = Point3(xy_list[j+5][0],xy_list[j+5][1]+200,10).clean()
        real_tgts_list.append(temp)
    return real_tgts_list

list_my = generate_real_targets()
for temp in list_my:
    print(temp.to_list())
"""
    优化成功
"""
print("**************投影后**************")
#导弹初始坐标，速率，速度矢量
miss_init_pos = Point3(20000,0,2000)
miss_spd = 300.0
miss_vel = miss_init_pos.unit() * -1 * miss_spd
t_max  = miss_init_pos.norm() / miss_spd
def plot_1d(arr):
    """
    Visualize a 1D array with a composite chart:
    - Line plot: original data values
    - Bar plot: growth rate (%)
    arr: 1D array or list
    """
    plt.figure(figsize=(10, 5))
    
    x = np.arange(len(arr))
    title = '1D Array Visualization (Value + Growth Rate)'

    # 计算增长率（百分比）
    growth = np.zeros_like(arr, dtype=float)
    growth[1:] = np.diff(arr) / arr[:-1] * 100  # 增长率公式

    # 双轴绘图
    ax1 = plt.gca()
    ax2 = ax1.twinx()

    # 主Y轴：折线图（原始数据）
    ax1.plot(x, arr, linewidth=2.5, color='#2E86AB', label='Original Value')
    ax1.set_ylabel('Original Value', color='#2E86AB', fontsize=12)
    ax1.tick_params(axis='y', labelcolor='#2E86AB')

    # 次Y轴：柱状图（增长率 %）
    ax2.bar(x, growth, alpha=0.6, color='#F24236', label='Growth Rate (%)')
    ax2.set_ylabel('Growth Rate (%)', color='#F24236', fontsize=12)
    ax2.tick_params(axis='y', labelcolor='#F24236')

    # 标题与网格
    plt.title(title, fontsize=14)
    ax1.grid(alpha=0.3)
    plt.tight_layout()
    plt.show()

dt = 0.1
ans = []
ps_per = []
ps_cur = []
while dt <= 40:
    yoz_tgts = []
    #导弹的实时位置
    miss_pos_2 = miss_init_pos + miss_vel * dt
    for temp in list_my:
        yoz_tgts.append(miss_pos_2.intersect_yoz(temp))
    if len(ps_per) == 0:
        ps_per = yoz_tgts
    ans.append(convex_hull_area_yoz(yoz_tgts))
    dt+=0.1
ps_cur = yoz_tgts
print(convex_hull_area_yoz(ps_per))#153.82
print(convex_hull_area_yoz(ps_cur))#153.77
print(convex_hull_area_yoz(ps_cur+ps_per))#153.87
print(max(ans)-min(ans))#0.0508
print(statistics.mean(ans))#153.8066
relative_change = (max(ans) - min(ans)) / statistics.mean(ans)
print(f"相对变化率: {relative_change:.2%}")#0.03%
plot_1d(ans)#单调均匀变小
"""
    结论,真目标组关于导弹实时位置在yoz平面的投影,在运动50秒内变化很小,为了后续节约时间开支,故假设投影不变
    在工程仿真、制导仿真里，这种变化完全可以视为噪声级、可忽略。
    变化极其平稳、无突变、无抖动
"""

