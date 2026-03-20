% =========================================================================
% 更新点云数据函数
% =========================================================================
% 功能说明：
%   该函数将新的检测结果转换为点云格式，为每个检测点创建一个点云元素
%   并初始化关联相关的字段
% 
% 输入参数：
%   inst - 跟踪器实例
%   angle - 角度数据（弧度）
%   range - 距离数据（米）
%   vel - 速度数据（米/秒）
%   snr - 信噪比数据
% 
% 输出参数：
%   inst - 更新后的跟踪器实例，包含新的点云数据
% 
% 点云元素结构：
%   - vel: 速度（米/秒）
%   - angle: 角度（弧度）
%   - range: 距离（米）
%   - power: 功率/信噪比
%   - best_score: 最佳关联分数（初始化为无穷大）
%   - best_ind: 最佳关联目标索引（初始化为0，表示未关联）
% =========================================================================

function inst = update_point_cloud(inst, angle, range, vel, snr)
    % 清空现有点云数据
    inst.point_cloud = [];
    
    % 为每个检测点创建点云元素
    for i = 1:length(vel)
        % 创建点云元素并设置基本属性
        inst.point_cloud(i).vel = vel(i);  % 速度（米/秒）
        inst.point_cloud(i).angle = angle(i);  % 角度（弧度）
        inst.point_cloud(i).range = range(i);  % 距离（米）
        inst.point_cloud(i).power = snr(i);  % 功率/信噪比
        
        % 初始化关联相关字段
        inst.point_cloud(i).best_score = inf;  % 最佳关联分数，初始化为无穷大
        inst.point_cloud(i).best_ind = 0;  % 最佳关联目标索引，初始化为0（未关联）
    end
    
    % 更新点云数量
    inst.num_points = length(vel);
    
    % 更新心跳计数
    inst.heartBeat = inst.heartBeat + 1;
end