%--------------------------------------------------------------------------
% 函数名称: params_init
% 函数功能: 初始化雷达跟踪和跌倒检测的参数
% 输入参数: 无
% 输出参数:
%   - inst: 包含所有参数的结构体
% 函数原理:
%   初始化雷达信号处理、目标跟踪、卡尔曼滤波和跌倒检测所需的所有参数
%   参数值经过优化，适用于人体目标的跟踪和跌倒检测场景
%--------------------------------------------------------------------------
function inst = params_init
%--------------------------------------------------------------------------
% 点云数据信息
%--------------------------------------------------------------------------
inst.point_cloud = [];       % 点云数据结构体数组
inst.num_points = 0;         % 点云数量

%--------------------------------------------------------------------------
% 计时信息
%--------------------------------------------------------------------------
inst.heartBeat = 0;          % 心跳计数器，用于跟踪时间

%--------------------------------------------------------------------------
% 目标信息
%--------------------------------------------------------------------------
inst.target_num_active = 0;  % 活跃目标数量
inst.target_num_all = 0;     % 总目标数量
inst.target = [];            % 目标结构体数组

%--------------------------------------------------------------------------
% 门限参数（数据关联）
%--------------------------------------------------------------------------
inst.gatingParams.limits = [3 2 0];  % 门限限制 [长度, 宽度, 速度]
inst.gatingParams.volume = 8;        % 门限体积系数

%--------------------------------------------------------------------------
% 分配量测参数（新目标初始化）
%--------------------------------------------------------------------------
inst.allocationParams.maxVelThre = 0.5; % 速度阈值，适应人体移动
inst.allocationParams.maxDistanceThre = 0.5; % 距离阈值，避免目标合并
inst.allocationParams.pointsThre = 5; % 点云数量阈值，控制目标初始化难度
inst.allocationParams.velThre = 0.2; % 速度阈值，用于目标分配

%--------------------------------------------------------------------------
% 速度参数（数据关联和速度估计）
%--------------------------------------------------------------------------
% inst.velParams.vel_init = 0;        % 速度初始化状态
% inst.velParams.vel_rate_filter = 1;  % 速度率滤波状态
% inst.velParams.vel_tracking = 2;     % 速度跟踪状态
% inst.velParams.vel_locked = 3;       % 速度锁定状态
% 
% inst.velParams.maxradvel = 20;       % 最大径向速度

inst.velParams.alpha = 0.5;           % 速度平滑因子
inst.velParams.confidence = 0.1;      % 速度置信度阈值

%--------------------------------------------------------------------------
% 目标状态参数
%--------------------------------------------------------------------------
inst.stateParams.pointsThre = 2;      % 点云数量阈值，用于目标状态判断
inst.stateParams.detection = 1;       % 检测状态
inst.stateParams.active = 2;          % 活跃状态
inst.stateParams.free = 3;            % 空闲状态
inst.stateParams.det2actThre = 1;     % 检测到活跃的阈值
inst.stateParams.act2freeThre = 3;    % 活跃到空闲的阈值
inst.stateParams.det2freeThre = 2;    % 检测到空闲的阈值

%--------------------------------------------------------------------------
% EKF（扩展卡尔曼滤波）参数
%--------------------------------------------------------------------------
inst.dt = 0.05; % 采样周期（秒）
dt = inst.dt;
dt2 = dt^2;
dt3 = dt^3;
dt4 = dt^4;
inst.maxAcceleration = 2; % 最大加速度

% 状态转移矩阵 F
% 状态向量: [x, y, vx, vy, ax, ay]'
inst.F = [1 0 dt 0 dt2/2 0;
          0 1 0 dt 0 dt2/2;
          0 0 1 0 dt 0;
          0 0 0 1 0 dt;
          0 0 0 0 1 0;
          0 0 0 0 0 1];
      
% 过程噪声协方差矩阵 Q
inst.Q = [ dt4/4 0 dt3/2 0 dt2/2 0;
           0 dt4/4 0 dt3/2 0 dt2/2;
           dt3/2 0 dt2 0 dt 0;
           0 dt3/2 0 dt2 0  dt;
           dt2/2 0 dt 0 1 0;
           0 dt2/2 0 dt 0 1];
inst.processVariance = (0.5*inst.maxAcceleration)^2;

%--------------------------------------------------------------------------
% EKF 更新参数
%--------------------------------------------------------------------------
inst.variationParams.lengthStd = 0.289; % 长度测量标准差
inst.variationParams.velStd = 1;        % 速度测量标准差
inst.variationParams.widthStd = 0.289;  % 宽度测量标准差
inst.min_points_to_update_dispersion = 5; % 更新色散所需的最小点数
inst.est_points = 10;                   % 估计点数
inst.dispersion_alpha = 0.1;            % 色散平滑因子

%--------------------------------------------------------------------------
% 跌倒检测参数
%--------------------------------------------------------------------------
inst.fdtParams.zero_age = 0;            % 零年龄计数器
inst.fdtParams.age = 0;                 % 年龄计数器
inst.fdtParams.fall_detected_age = 0;   % 跌倒检测年龄计数器
inst.fdtParams.fall_detected = 0;       % 跌倒检测标志
inst.fdtParams.ave_height = 0;          % 平均高度
inst.fdtParams.ave_height_count = 0;    % 平均高度计数
inst.fdtParams.ave_height_count2 = 0;   % 平均高度计数2
inst.fdtParams.ave_height_delta = 0;    % 平均高度变化
inst.fdtParams.ave_target_range = 0;    % 平均目标距离
inst.fdtParams.ave_target_azimuth = 0;  % 平均目标方位角
inst.fdtParams.ave_target_vel = 0;      % 平均目标速度
inst.fdtParams.object_ID_before = 0;    % 上一帧目标ID
inst.fdtParams.frame_count = 0;         % 帧计数器
inst.fdtParams.data = [];               % 跌倒检测数据
inst.fdtParams.data_age = [];           % 年龄数据
inst.fdtParams.data_error = [];         % 误差数据
inst.fdtParams.data_age_normal = [];    % 归一化年龄数据
inst.fdtParams.data_error_normal = [];  % 归一化误差数据
inst.fdtParams.flag = 0;                % 标志位
inst.fdtParams.fall_down_flag = -1;     % 跌倒标志
inst.fdtParams.fall_down_flag_1 = -1;   % 跌倒标志1
inst.fdtParams.dataPoints = {};         % 数据点
inst.fdtParams.t1 = 0;                  % 时间点1
inst.fdtParams.t2 = 0;                  % 时间点2
inst.fdtParams.fall_down_flag_10=0;     % 跌倒标志10

%--------------------------------------------------------------------------
% 关联点云
%--------------------------------------------------------------------------
inst.associatedPoints = [];             % 已关联的点云

%--------------------------------------------------------------------------
% 跟踪ID管理
%--------------------------------------------------------------------------
inst.trackID_all = 1:20;                % 跟踪ID数组，模拟栈结构

%--------------------------------------------------------------------------
% 单人跟踪参数
%--------------------------------------------------------------------------
inst.last_plot_trackID = 0;             % 上一帧绘制的目标ID
% 单人跟踪中，多径效应产生的两目标相离较近，则plot上一帧中目标，
% 保持plot的跟踪连续性，避免跟踪目标在不同目标间跳跃

%--------------------------------------------------------------------------
% GUI使用的变量
%--------------------------------------------------------------------------
inst.gui.vel_plot = []; % 速度曲线数据

inst.gui.point_x = [];  % 点云x坐标
inst.gui.point_y = [];  % 点云y坐标
inst.gui.point_x_associate = []; % 已关联点云x坐标
inst.gui.point_y_associate = []; % 已关联点云y坐标
inst.gui.target_x_all = []; % 所有目标x坐标
inst.gui.target_y_all = []; % 所有目标y坐标
inst.gui.target_x = []; % 活跃目标x坐标
inst.gui.target_y = []; % 活跃目标y坐标
inst.gui.true_target_index = 0; % 真实目标索引
inst.gui.state = []; % 目标状态
inst.gui.ID_string = []; % ID字符串
inst.gui.age_string = []; % 年龄字符串
inst.gui.xy_string = []; % 坐标字符串
inst.gui.range_string = []; % 距离字符串
inst.gui.vel_string = []; % 速度字符串
inst.gui.power_string = []; % 功率字符串


end