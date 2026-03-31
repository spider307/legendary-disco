from geom3d import * 
from math import *
import time
#坐标原点,标准重力加速度
ORIGIN3 = Point3()
g0 = -9.8
#导弹初始坐标，速率，速度矢量
miss_init_pos = Point3(20000,0,2000)
miss_spd = 300.0
miss_vel = miss_init_pos.unit() * -1 * miss_spd
#FY1无人机初始坐标，速率，方向角，速度矢量
FY1_init_pos = Point3(17800,0,1800)
FY1_spd = 120
FY1_yaw_v = pi
FY1_vel = Point3(FY1_spd*cos(FY1_yaw_v),FY1_spd*sin(FY1_yaw_v),0)
# 云团浓度有效维持时长,云团浓度有效遮蔽半径,匀速运动矢量
t_conc_eff: float = 20.0
r_conc_eff: float = 10.0
conc_vel = Point3(0,0,-3)

#主目标组
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
#real_tgts = Point3(0,200,0)

#投弹时刻，引爆延迟时长
t_drop = 1.5
dt_det = 3.6
#有效遮挡时长
t_mask = 0.0
#本次剖分模拟合适的时长上限
t_mesh_up = (10 + (miss_init_pos - FY1_init_pos).norm()) / (miss_spd - FY1_spd)
t_mesh_up =  min(t_mesh_up-t_drop-dt_det,t_conc_eff)
#t在0至t_drop+dt_det之间
t = t_drop+dt_det
#导弹做匀速运动
miss_pos_1 = miss_init_pos + miss_vel * t
#无人机在0至t_drop载弹水平匀速运动，t_drop至t_drop+dt_det烟雾弹做平抛运动
cloud_ctr_x = FY1_init_pos.x + FY1_vel.x * t
cloud_ctr_y = FY1_init_pos.y + FY1_vel.y * t
cloud_ctr_z = FY1_init_pos.z + (g0 * dt_det ** 2)/2
#云团的初始位置
cloud_ctr_1 = Point3(cloud_ctr_x,cloud_ctr_y,cloud_ctr_z)

#时刻,真目标点决定的优化函数
def zero_fun(T)->float:
    miss_pos = miss_pos_1 + miss_vel * T
    temp = cloud_ctr_1.z + conc_vel.z * T
    if  temp >= 0:
        cloud_pos = Point3(cloud_ctr_1.x, cloud_ctr_1.y, temp)
    else:
        cloud_pos = Point3(cloud_ctr_1.x, cloud_ctr_1.y, 0)
    fun = cloud_pos.dist_point_segment(miss_pos, real_tgts)
    return fun - r_conc_eff

#寻找单谷函数的极值点
def find_valley(f,low,high,step=0.1):
    ts = []
    fs = []
    t = low
    while t <= high:
        ts.append(t)
        fs.append(f(t))
        t+=step
    idx = fs.index(min(fs))
    return ts[idx]

# 左根：降落段 入遮蔽零点
def root_in(f, a, b):
    for _ in range(15):
        m = (a+b)/2
        if f(m) > 0:   
            a = m
        else:
            b = m
    return (a+b)/2

# 右根：抬升段 出遮蔽零点
def root_out(f, a, b):
    for _ in range(15):
        m = (a+b)/2
        if f(m) < 0:
            a = m
        else:
            b = m
    return (a+b)/2

#######主程序########
t_start, t_current, t_end = 0,[],0
t_mask = []
start = time.time()  # 开始计时
for real_tgts in list_my:
    valley = find_valley(zero_fun,0,t_mesh_up)
    root_light = root_in(zero_fun,0,valley)
    root_right = root_out(zero_fun,valley,t_mesh_up)
    t_mask.append(root_right - root_light)
end = time.time()    # 结束计时
run_time = end - start
print(f"程序运行时间：{run_time:.4f} 秒")

print(min(t_mask))
print(len(t_mask))

"""
综上,一个真目标,采用点到线段的距离断言,得到结论:在 8.039+5.1 秒时,遮蔽;在 9.439+5.1 秒时,退出遮蔽;共计1.39秒有效时长
程序运行时间维持在1至3毫秒
另外,10个真目标需要2至5毫秒
"""




