% =========================================================================
% 生成 steering 向量函数
% =========================================================================
% 功能说明：
%   该函数生成用于波束形成的 steering 向量矩阵
%   steering 向量表示不同角度下天线阵列的相位响应
% 
% 输入参数：
%   angle_range - 角度范围（度），例如 90 表示 -90° 到 90°
%   angle_res - 角度分辨率（度）
%   num_ant - 天线数量
% 
% 输出参数：
%   num_vec - steering 向量数量
%   steering_vec - steering 向量矩阵，维度为 [num_vec, num_ant]
% 
% 实现原理：
%   1. 生成角度范围的角度序列
%   2. 对每个角度，计算对应天线阵列的 steering 向量
%   3. steering 向量的每个元素表示该角度下对应天线的相位延迟
% =========================================================================

% function [num_vec, steering_vec] = gen_steering_vec(angle_range, angle_res, num_ant)
%     % 生成角度范围的角度序列
%     angles = -angle_range:angle_res:angle_range;
%     num_vec = length(angles);  % steering 向量数量
% 
%     % 初始化 steering 向量矩阵
%     steering_vec = zeros(num_vec, num_ant, 'like', 1i);
% 
%     % 对每个角度，计算对应天线阵列的 steering 向量
%     for i = 1:num_vec
%         for j = 1:num_ant
%             % 计算 steering 向量的元素
%             % 假设天线间距为半波长，相位延迟与角度的正弦成正比
%             % AWR2944的虚拟天线排列可能需要调整这个公式
%             steering_vec(i, j) = exp(1i * pi * (j-1) * sind(angles(i)));
%         end
%     end
% 
%     % 对于AWR2944的1发4收，虚拟天线数量应该是4
%     if num_ant == 4
%         fprintf('生成了 %d 个steering向量，用于4虚拟天线系统\n', num_vec);
%     end
% end
function [num_vec, steering_vec] = gen_steering_vec(angle_range, angle_res, num_ant)
    % 生成角度范围的角度序列
    angles = -angle_range:angle_res:angle_range;
    num_vec = length(angles);  % steering 向量数量
    
    % 初始化 steering 向量矩阵
    steering_vec = zeros(num_vec, num_ant, 'like', 1i);
    
    % 根据天线数量计算天线间距
    % 假设均匀线阵，相邻天线间距为半波长
    d_lambda = 0.5;  % 天线间距与波长之比
    
    % 对每个角度，计算对应天线阵列的 steering 向量
    for i = 1:num_vec
        for j = 1:num_ant
            % 计算 steering 向量的元素
            % 相位 = 2π * (天线索引) * (间距/波长) * sin(角度)
            steering_vec(i, j) = exp(1i * 2 * pi * (j-1) * d_lambda * sind(angles(i)));
        end
    end
    
    fprintf('生成了 %d 个steering向量，用于 %d 虚拟天线系统\n', num_vec, num_ant);
end