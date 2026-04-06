%% 清屏、清空工作区、关闭图像窗口
clear;close all;
rng(14)  % 固定随机种子，保证遗传算法每次运行结果一致（可复现）

%% ===================== 2. 生成真目标区域采样点 =====================
% 功能：生成上下两个半圆的8个空间点，作为【真目标位置集合】
% 真目标参数
n = 4;                  % 每个底面采样点数：下底面4个 + 上底面4个 = 共8个点

% ---------------------- 下底面（z=0）：右半圆（角度 -90° ~ 90°） ----------------------
theta_down = pi * linspace(-0.5, 0.5, n);  % 生成下底面角度：-π/2 到 π/2（右半圆）
z_down = zeros(1, n);                      % 下底面高度 z=0
x_down = 7 * cos(theta_down);              % 下底面 x 坐标（半径7）
y_down = 200 + 7 * sin(theta_down);        % 下底面 y 坐标（圆心y=200）

% ---------------------- 上底面（z=10）：左半圆（角度 90° ~ 270°） ----------------------
theta_up = pi * linspace(0.5, 1.5, n);     % 上底面角度：π/2 到 3π/2（左半圆）
z_up = 10 * ones(1, n);                    % 上底面高度 z=10
x_up = 7 * cos(theta_up);                  % 上底面 x 坐标（半径7）
y_up = 200 + 7 * sin(theta_up);            % 上底面 y 坐标（圆心y=200）

% 合并上下底面坐标，得到 8×3 的真目标坐标矩阵
Ai_down = [x_down', y_down', z_down'];     % 下底面4个点 (4×3)
Ai_up = [x_up', y_up', z_up'];             % 上底面4个点 (4×3)
Ai = [Ai_down; Ai_up];                     % 合并为 8×3 矩阵（8个真目标点）

%% ===================== 遗传算法优化（寻找最优遮挡参数） =====================
% 优化变量：[theta, v, t_drop, dt_det]
% theta：飞行角度  v：飞行速度  t_drop：投放时间  dt_det：检测延迟时间
lb=[0,70,0,0,0,0.01,0.01,0.01];          % 优化变量下限
ub=[pi/10,90,5,5,5,5,5,5];     % 优化变量上限
nvars=8;                % 优化变量个数 = 8

A=[0,0,0,1,-1,0,0,0;
   0,0,1,-1,0,0,0,0];
B=[-1;
   -1];            % 无线性不等式约束
Aeq=[]; Beq=[];         % 无线性等式约束

% 定义优化目标函数：适应度函数（输入参数x，输出适应度值）
objFun = @(x) shi_ying_du(x, Ai); 

% 遗传算法参数设置
options = optimoptions('ga', ...
    'PopulationSize', 100, ...     % 种群规模
    'MaxGenerations', 300, ...     % 最大迭代进化
    'EliteCount', 6, ...         % 精英个体保留
    'PlotFcn', @gaplotbestf);     % 画出最优适应度曲线

% 运行遗传算法
tic;
[x_1, fval, exitflag, output, population] = ga(objFun, nvars, A, B, Aeq, Beq, lb, ub, [], options);
toc;
fprintf('第二问的有效遮挡时间为 %.3f秒',-fval)  % 显示最优适应度值
fprintf('\n决策变量:\n无人机极角 %.3f°\n无人机飞行速率 %.3fm\\s',x_1(1)*180/pi,x_1(2))
fprintf('\n投放时间 %.3fs %.3fs %.3fs ',x_1(3),x_1(4),x_1(5))
fprintf('\n延迟时长 %.3fs %.3fs %.3fs ',x_1(6),x_1(7),x_1(8))
t=excel(x_1,Ai);
fprintf('\n在 %.3fs 进入有效遮蔽，在 %.3fs 结束有效遮蔽,有效遮挡时间为 %.3f秒\n',t(3),t(2),t(1))
%% ===================== 核心适应度函数：计算云团遮挡时间 =====================
% 输入：x = [theta, v, t_drop, dt_det]  飞行参数
% 输入：A = 8×3 真目标点矩阵
% 输出：TIME = -最小遮挡时间（遗传算法求最小值 = 最大化遮挡时间）
function TIME=shi_ying_du(x,A)
     % ------------ 1. 解析输入的优化参数 ------------
    theta   = x(1);        % 飞行角度
    v       = x(2);            % 飞行速度
    t_drop1 = x(3);       % 投放时刻1
    t_drop2 = x(4);       % 投放时刻2
    t_drop3 = x(5);       % 投放时刻3
    dt_det1 = x(6);       % 检测延迟时间1
    dt_det2 = x(7);       % 检测延迟时间2
    dt_det3 = x(8);       % 检测延迟时间3

    % ------------ 2. 固定物理参数 ------------
    miss_spd = 300;                  % 导弹速度
    FY1_vel=v*[cos(theta),sin(theta),0];% 飞机速度向量（水平飞行）
    s3     = 3;                         % 云团下沉速度
    g      = 9.8;                       % 重力加速度
    t_in1  = t_drop1+dt_det1;           % 云团1开始运动时间
    t_out1 = t_in1+20;                  % 云团1结束运动时间（总时长20s）
    t_in2  = t_drop2+dt_det2;           % 云团2开始运动时间
    t_out2 = t_in2+20;                  % 云团2结束运动时间（总时长20s)
    t_in3  = t_drop3+dt_det3;           % 云团3开始运动时间
    t_out3 = t_in3+20;                  % 云团3结束运动时间（总时长20s)

    % ------------ 3. 初始位置坐标 ------------
    miss_int_pos     = [20000,0,2000];    % 导弹初始位置
    norm_miss_origin = norm(miss_int_pos);% 导弹初始位置模长
    FY1_init_pos     = [17800,0,1800];    % 飞机初始位置

    % 计算云团初始位置（考虑飞机运动 + 下落）
    cloud_init_pos1 = FY1_init_pos+t_in1*FY1_vel-[0,0,0.5*g*dt_det1^2];
    cloud_init_pos2 = FY1_init_pos+t_in2*FY1_vel-[0,0,0.5*g*dt_det2^2];
    cloud_init_pos3 = FY1_init_pos+t_in3*FY1_vel-[0,0,0.5*g*dt_det3^2];

    % ------------ 4. 生成时间轴与轨迹 ------------
    dt_points = 400;                     % 时间采样200个点
    t_span = linspace(t_in1, t_out3, dt_points)';  % 时间向量 t_in1 ~ t_out3
    
    % 云团运动轨迹（下沉）
    cloud_traj1 = cloud_init_pos1 - s3 * [zeros(dt_points,2), (t_span-t_in1)];
    cloud_traj2 = cloud_init_pos2 - s3 * [zeros(dt_points,2), (t_span-t_in2)];
    cloud_traj3 = cloud_init_pos3 - s3 * [zeros(dt_points,2), (t_span-t_in3)];
    % 导弹运动轨迹（匀速飞向原点）
    miss_traj1 = miss_int_pos - miss_spd * t_span * miss_int_pos / norm_miss_origin;
    miss_traj2 = miss_int_pos - miss_spd * t_span * miss_int_pos / norm_miss_origin;
    miss_traj3 = miss_int_pos - miss_spd * t_span * miss_int_pos / norm_miss_origin;

    T = zeros(size(A,1),1);  % 存储每个真目标的【遮挡时间】
    % ------------ 5. 遍历每个真目标点，计算遮挡时间 ------------
    for i = 1:size(A,1)
        A_i = A(i,:);  % 取出第i个真目标点
        A_mat = repmat(A_i,dt_points,1);  % 扩展成和轨迹等长的矩阵（向量化计算）
        
        % 向量化计算：云团到【线段：真目标→导弹】的距离 - 10
        % 距离 < 10 表示被遮挡
        d1 = dist_point_segment_vec(cloud_traj1, A_mat, miss_traj1) - 10;
        d2 = dist_point_segment_vec(cloud_traj2, A_mat, miss_traj2) - 10;
        d3 = dist_point_segment_vec(cloud_traj3, A_mat, miss_traj3) - 10;

        d = min(d1,min(d2,d3));
        size(d);
        diff_d2 = diff(sign(d));  % 计算符号变化（判断是否穿越距离=10）
        sign_change_idx = find(diff_d2 ~= 0);  % 找到符号变化点（遮挡开始/结束）
        if length(sign_change_idx) == 5
            disp("5")
        end
        if length(sign_change_idx) == 6
            disp("6")
        end
        if isempty(sign_change_idx)
            % 情况1：无符号变化 → 全程不遮挡 → 时间=0
            T(i) = 0;

        elseif isscalar(sign_change_idx)
            % 情况2：只有1次穿越 → 只进入遮挡 或 只离开遮挡
            idx = sign_change_idx(1);
            x1 = t_span(idx); y1 = d(idx);
            x2 = t_span(idx+1); y2 = d(idx+1);
            root = x1 - y1*(x2-x1)/(y2-y1);  % 线性插值求根（遮挡时刻）

            if diff_d2(idx) == 2
                % 符号：- → +  离开遮挡
                T(i) = root - t_in1;
            else
                % 符号：+ → -  进入遮挡
                T(i) = t_out3 - root;
            end

        else
            % 情况3：2次穿越 → 完整遮挡一段区间（进入+离开）
            idx1 = sign_change_idx(1);
            idx2 = sign_change_idx(2);

            % 求进入遮挡时刻
            x1 = t_span(idx1); y1 = d(idx1);
            x2 = t_span(idx1+1); y2 = d(idx1+1);
            root_left = x1 - y1*(x2-x1)/(y2-y1);

            % 求离开遮挡时刻
            x1 = t_span(idx2); y1 = d(idx2);
            x2 = t_span(idx2+1); y2 = d(idx2+1);
            root_right = x1 - y1*(x2-x1)/(y2-y1);

            % 遮挡总时长 = 离开 - 进入
            T(i) = root_right - root_left;
        end
    end
    
    % ------------ 6. 输出适应度值 ------------
    TIME=min(T(:,1));  % 取8个点中【最小遮挡时间】（保证所有点都能被遮挡）
    if ~isfinite(TIME), TIME = 0; end  % 异常值保护
    TIME=-TIME;   % 取负 → 遗传算法求最小值 = 最大化遮挡时间
end

%% ===================== 向量化函数：点到线段的距离 =====================
% 功能：批量计算 点P 到 线段AB 的距离（支持矩阵输入，速度极快）
% P：云团坐标矩阵 N×3
% A：线段起点（真目标）N×3
% B：线段终点（导弹）N×3
function y = dist_point_segment_vec(P,A,B)
    AB = B - A;              % 线段AB向量
    AP = P - A;              % 点A到点P向量
    AB_norm_2 = sum(AB.^2,2); % AB长度平方
    t = sum(AP .* AB,2) ./ max(AB_norm_2, 1e-8);  % 投影系数

    y = zeros(size(t));
    idx0 = t < 0;            % 投影点在线段外（靠近A）
    idx1 = t > 1;            % 投影点在线段外（靠近B）
    idx_mid = ~(idx0 | idx1);% 投影点在线段上

    % 分情况计算距离
    y(idx0) = vecnorm(AP(idx0,:),2,2);
    y(idx1) = vecnorm(P(idx1,:) - B(idx1,:),2,2);
    y(idx_mid) = vecnorm(A(idx_mid,:) + AB(idx_mid,:) .* t(idx_mid) - P(idx_mid,:),2,2);
end
%%
function y=excel(x,A)
      % ------------ 1. 解析输入的优化参数 ------------
    theta   = x(1);        % 飞行角度
    v       = x(2);            % 飞行速度
    t_drop1 = x(3);       % 投放时刻1
    t_drop2 = x(4);       % 投放时刻2
    t_drop3 = x(5);       % 投放时刻3
    dt_det1 = x(6);       % 检测延迟时间1
    dt_det2 = x(7);       % 检测延迟时间2
    dt_det3 = x(8);       % 检测延迟时间3

    % ------------ 2. 固定物理参数 ------------
    miss_spd = 300;                  % 导弹速度
    FY1_vel=v*[cos(theta),sin(theta),0];% 飞机速度向量（水平飞行）
    s3     = 3;                         % 云团下沉速度
    g      = 9.8;                       % 重力加速度
    t_in1  = t_drop1+dt_det1;           % 云团1开始运动时间
    t_out1 = t_in1+20;                  % 云团1结束运动时间（总时长20s）
    t_in2  = t_drop2+dt_det2;           % 云团2开始运动时间
    t_out2 = t_in2+20;                  % 云团2结束运动时间（总时长20s)
    t_in3  = t_drop3+dt_det3;           % 云团3开始运动时间
    t_out3 = t_in3+20;                  % 云团3结束运动时间（总时长20s)

    % ------------ 3. 初始位置坐标 ------------
    miss_int_pos     = [20000,0,2000];    % 导弹初始位置
    norm_miss_origin = norm(miss_int_pos);% 导弹初始位置模长
    FY1_init_pos     = [17800,0,1800];    % 飞机初始位置

    % 计算云团初始位置（考虑飞机运动 + 下落）
    cloud_init_pos1 = FY1_init_pos+t_in1*FY1_vel-[0,0,0.5*g*dt_det1^2];
    cloud_init_pos2 = FY1_init_pos+t_in2*FY1_vel-[0,0,0.5*g*dt_det2^2];
    cloud_init_pos3 = FY1_init_pos+t_in3*FY1_vel-[0,0,0.5*g*dt_det3^2];

    % ------------ 4. 生成时间轴与轨迹 ------------
    dt_points = 400;                     % 时间采样200个点
    t_span = linspace(t_in1, t_out3, dt_points)';  % 时间向量 t_in1 ~ t_out3
    
    % 云团运动轨迹（下沉）
    cloud_traj1 = cloud_init_pos1 - s3 * [zeros(dt_points,2), (t_span-t_in1)];
    cloud_traj2 = cloud_init_pos2 - s3 * [zeros(dt_points,2), (t_span-t_in2)];
    cloud_traj3 = cloud_init_pos3 - s3 * [zeros(dt_points,2), (t_span-t_in3)];
    % 导弹运动轨迹（匀速飞向原点）
    miss_traj1 = miss_int_pos - miss_spd * t_span * miss_int_pos / norm_miss_origin;
    miss_traj2 = miss_int_pos - miss_spd * t_span * miss_int_pos / norm_miss_origin;
    miss_traj3 = miss_int_pos - miss_spd * t_span * miss_int_pos / norm_miss_origin;

    T = zeros(size(A,1),3);  % 存储每个真目标的【遮挡时间】
    % ------------ 5. 遍历每个真目标点，计算遮挡时间 ------------
    for i = 1:size(A,1)
        A_i = A(i,:);  % 取出第i个真目标点
        A_mat = repmat(A_i,dt_points,1);  % 扩展成和轨迹等长的矩阵（向量化计算）
        
        % 向量化计算：云团到【线段：真目标→导弹】的距离 - 10
        % 距离 < 10 表示被遮挡
        d1 = dist_point_segment_vec(cloud_traj1, A_mat, miss_traj1) - 10;
        d2 = dist_point_segment_vec(cloud_traj2, A_mat, miss_traj2) - 10;
        d3 = dist_point_segment_vec(cloud_traj3, A_mat, miss_traj3) - 10;

        d = min(d1,min(d2,d3));
        size(d);
        diff_d2 = diff(sign(d));  % 计算符号变化（判断是否穿越距离=10）
        sign_change_idx = find(diff_d2 ~= 0);  % 找到符号变化点（遮挡开始/结束）
        if length(sign_change_idx) >= 3
            disp("!")
        end
        if isempty(sign_change_idx)
            % 情况1：无符号变化 → 全程不遮挡 → 时间=0
            T(i) = 0;

        elseif isscalar(sign_change_idx)
            % 情况2：只有1次穿越 → 只进入遮挡 或 只离开遮挡
            idx = sign_change_idx(1);
            x1 = t_span(idx); y1 = d(idx);
            x2 = t_span(idx+1); y2 = d(idx+1);
            root = x1 - y1*(x2-x1)/(y2-y1);  % 线性插值求根（遮挡时刻）

            if diff_d2(idx) == 2
                % 符号：- → +  离开遮挡
                T(i,:) = [root - t_in1,root,t_in1];
            else
                % 符号：+ → -  进入遮挡
                T(i,:) = [t_out3 - root,t_out3 ,root];
            end

        else
            % 情况3：2次穿越 → 完整遮挡一段区间（进入+离开）
            idx1 = sign_change_idx(1);
            idx2 = sign_change_idx(2);

            % 求进入遮挡时刻
            x1 = t_span(idx1); y1 = d(idx1);
            x2 = t_span(idx1+1); y2 = d(idx1+1);
            root_left = x1 - y1*(x2-x1)/(y2-y1);

            % 求离开遮挡时刻
            x1 = t_span(idx2); y1 = d(idx2);
            x2 = t_span(idx2+1); y2 = d(idx2+1);
            root_right = x1 - y1*(x2-x1)/(y2-y1);

            % 遮挡总时长 = 离开 - 进入
            T(i,:) = [root_right - root_left,root_right , root_left];
        end
    end
    
    % ------------ 6. 输出适应度值 ------------
    [~,idx]=min(T(:,1));  % 取8个点中【最小遮挡时间】（保证所有点都能被遮挡）
    y=T(idx,:);
end