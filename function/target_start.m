% =========================================================================
% 初始化新目标函数
% =========================================================================
% 功能说明：
%   该函数初始化一个新的跟踪目标，设置其初始状态和参数
%   包括：
%   1. 从极坐标系转换到直角坐标系
%   2. 初始化目标的状态向量和协方差矩阵
%   3. 设置目标的关联相关参数
%   4. 初始化目标的状态管理参数
%   5. 分配跟踪ID
%   6. 设置目标的功率信息
% 
% 输入参数：
%   inst - 跟踪器实例
%   un - 目标的极坐标系状态 [range, angle, vel]'
%   power_sum - 关联点云的功率总和
%   power_mean - 关联点云的平均功率
% 
% 输出参数：
%   inst - 更新后的跟踪器实例，包含新初始化的目标
% =========================================================================

function inst = target_start(inst, un, power_sum, power_mean)
    % =================================================================
    % 1. 设置目标的极坐标系状态
    % =================================================================
    inst.target(inst.target_num_all).Hs = un;  % 存储极坐标系状态 [range, angle, vel]'
    
    % =================================================================
    % 2. 从极坐标系转换到直角坐标系
    % =================================================================
    range = un(1);  % 距离
    angle = un(2);  % 角度
    vel = un(3);  % 速度
    
    % 计算直角坐标系中的位置
    posx = range * sin(angle);  % x 坐标
    posy = range * cos(angle);  % y 坐标
    
    % 计算直角坐标系中的速度
    velx = vel * sin(angle);  % x 方向速度
    vely = vel * cos(angle);  % y 方向速度
    
    % =================================================================
    % 3. 初始化目标的状态向量和协方差矩阵
    % =================================================================
    new_target_index = inst.target_num_all;  % 新目标的索引
    
    % 初始化预测状态向量 [x, y, vx, vy, ax, ay]'
    inst.target(new_target_index).S_apriori_hat = [posx posy velx vely 0 0]';
    % 当前状态向量初始化为预测状态向量
    inst.target(new_target_index).S_hat = inst.target(inst.target_num_all).S_apriori_hat;
    
    % 初始化预测协方差矩阵
    inst.target(new_target_index).P_apriori_hat = diag([0.1 0.1 0.1 0.1 0.1 0.1]);
    % 当前协方差矩阵初始化为预测协方差矩阵
    inst.target(new_target_index).P_hat = inst.target(inst.target_num_all).P_apriori_hat;
    
    % =================================================================
    % 4. 设置目标的关联相关参数
    % =================================================================
    inst.target(new_target_index).G = 1;  % 门限值
    inst.target(new_target_index).associatedPoints = 0;  % 关联点云计数
    inst.target(new_target_index).gD = zeros(1, 9);  % 点云分散度
    inst.target(new_target_index).misdetect = 0;  % 漏检计数
    
    % =================================================================
    % 5. 初始化目标的时间相关参数
    % =================================================================
    inst.target(new_target_index).heartBeatCount = inst.heartBeat;  % 心跳计数
    inst.target(new_target_index).age = 0;  % 目标年龄
    
    % 注释掉的速度处理相关参数
    % inst.target(new_target_index).vel_handing = inst.velParams.vel_init;
    % inst.target(new_target_index).rangeRate = un(3);
    % inst.target(new_target_index).allocationvel = un(3);
    % inst.target(new_target_index).allocationRange = un(1);
    % inst.target(new_target_index).allocationTime = inst.heartBeat;
    
    % =================================================================
    % 6. 初始化目标的状态管理参数
    % =================================================================
    inst.target(new_target_index).state = inst.stateParams.detection;  % 初始状态为检测状态
    inst.target(new_target_index).detect2activeCount = 0;  % 检测到激活的计数
    inst.target(new_target_index).active2freeCount = 0;  % 激活到自由的计数
    inst.target(new_target_index).detect2freeCount = 0;  % 检测到自由的计数
    
    % =================================================================
    % 7. 分配跟踪ID
    % =================================================================
    inst.target(new_target_index).trackID = inst.trackID_all(1);  % 分配第一个可用的trackID
    inst.target(new_target_index).point_num = 0;  % 点云数量
    % trackID 出栈（从可用ID列表中移除）
    inst.trackID_all = inst.trackID_all(2:end);
    
    % =================================================================
    % 8. 设置目标的功率信息
    % =================================================================
    inst.target(new_target_index).power_sum = power_sum;  % 功率总和
    inst.target(new_target_index).power_mean = power_mean;  % 平均功率
end