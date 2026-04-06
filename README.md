# 2025年高教社杯全国大学生数学建模竞赛A题 完整解题代码
本仓库为2025年高教社杯全国大学生数学建模竞赛A题（无人机遮蔽问题）的完整解题方案，包含Python+MATLAB双语言实现，覆盖第一、二、三问全流程。

## 📂 文件结构
### 第一问（真目标组考量）
- `第一问对真目标组的考量.py`：真目标组分析思路
- `第一问第一版.py` / `第一问第二版.py`：第一问代码迭代版本
- `geom3d.py`：三维几何计算工具类（点到线段距离、坐标变换等）

### 第二问（云团遮挡优化）
- `Copy_of_di_er_wen_fast.m`：第二问MATLAB快速解法（遗传算法优化遮挡时间）

### 第三问（完整场景求解）
- `disanwen.m`：第三问完整MATLAB解法代码

## ⚙️ 运行环境
- Python 3.8+
- MATLAB R2022a+（需Global Optimization Toolbox支持遗传算法）

## 🏃 快速使用
1.  克隆仓库到本地：
    ```bash
    git clone https://github.com/spider307/legendary-disco.git
