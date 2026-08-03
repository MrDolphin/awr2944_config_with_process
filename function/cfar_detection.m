% =========================================================================
% CFAR 目标检测函数 - 修改为处理距离-多普勒谱
% =========================================================================
% 功能说明：
%   该函数使用恒虚警率（CFAR）算法检测距离-多普勒谱中的目标
%   采用二维 CFAR 检测，分别在距离和多普勒方向上执行 CFAR
%   提高目标检测的可靠性，减少虚警
% 
% 输入参数：
%   doppler_spectrum - 距离-多普勒谱，维度为 [num_chirps, BINS_PROCESSED]
%   config - 雷达配置参数
% 
% 输出参数：
%   peaks - 检测到的峰值位置，逻辑矩阵
%   dopplers - 检测到的目标的多普勒索引
%   ranges - 检测到的目标的距离索引
%   snrs - 检测到的目标的信噪比
% 
% 实现原理：
%   1. 对距离-多普勒谱取对数，提高动态范围
%   2. 在多普勒方向上执行 CFAR
%   3. 在距离方向上执行 CFAR
%   4. 合并两次 CFAR 的结果
%   5. 跳过边缘区域的检测
%   6. 提取目标位置和信噪比
% =========================================================================

function [peaks, dopplers, ranges, snrs] = cfar_detection(doppler_spectrum, config)
    % =================================================================
    % 1. 预处理：对距离-多普勒谱取对数
    % =================================================================
    heatmap_log = log2(abs(doppler_spectrum) + eps);  % 取对数，提高动态范围
    
    % 获取距离-多普勒谱的维度
    [num_dopplers, num_ranges] = size(heatmap_log);
    fprintf('  cfar_detection: 输入 heatmap_log [%d, %d]\n', num_dopplers, num_ranges);
    
    % =================================================================
    % 2. 多普勒方向 CFAR
    % =================================================================
    first_pass = zeros(size(heatmap_log));  % 存储多普勒方向 CFAR 结果
    fprintf('  cfar_detection: 开始多普勒方向 CFAR...\n');
    
    for doppler_idx = 1:num_dopplers
        % 对每个多普勒单元，在距离方向上执行 CFAR
        [threshold, noise_floor] = cfar_ca(heatmap_log(doppler_idx, :), ...
            config.GUARD_LEN, config.NOISE_LEN, config.L_BOUND_DOPPLER);
        % 标记超过阈值的位置
        first_pass(doppler_idx, :) = heatmap_log(doppler_idx, :) > threshold;
    end
    fprintf('  cfar_detection: 多普勒 CFAR 完成\n');
    
    % =================================================================
    % 3. 距离方向 CFAR
    % =================================================================
    second_pass = zeros(size(heatmap_log));  % 存储距离 CFAR 结果
    noise_floor_matrix = zeros(size(heatmap_log));  % 存储噪声基底
    fprintf('  cfar_detection: 开始距离方向 CFAR...\n');
    
    for range_idx = 1:num_ranges
        % 对每个距离单元，在多普勒方向上执行 CFAR
        [threshold, noise_floor] = cfar_ca(heatmap_log(:, range_idx), ...
            config.GUARD_LEN, config.NOISE_LEN, config.L_BOUND_RANGE);
        % 标记超过阈值的位置
        second_pass(:, range_idx) = heatmap_log(:, range_idx) > threshold;
        % 存储噪声基底
        noise_floor_matrix(:, range_idx) = noise_floor;
    end
    fprintf('  cfar_detection: 距离 CFAR 完成\n');
    
    % 打印中间结果尺寸
    fprintf('  cfar_detection: first_pass [%d, %d], second_pass [%d, %d]\n', ...
        size(first_pass, 1), size(first_pass, 2), size(second_pass, 1), size(second_pass, 2));
    
    % =================================================================
    % 4. 合并两次 CFAR 的结果
    % =================================================================
    peaks = first_pass & second_pass;  % 逻辑与，只有在两个方向上都超过阈值的位置才被认为是目标
    
    % =================================================================
    % 5. 跳过边缘区域
    % =================================================================
    % 边缘区域的信号可能不可靠，跳过检测
    peaks(1:config.SKIP_SIZE, :) = false;  % 跳过顶部边缘（多普勒）
    peaks(end-config.SKIP_SIZE+1:end, :) = false;  % 跳过底部边缘（多普勒）
    peaks(:, 1:config.SKIP_SIZE) = false;  % 跳过左侧边缘（距离）
    peaks(:, end-config.SKIP_SIZE+1:end) = false;  % 跳过右侧边缘（距离）
    
    % =================================================================
    % 6. 提取目标位置和信噪比
    % =================================================================
    [dopplers, ranges] = find(peaks);  % 提取目标的多普勒和距离索引
    fprintf('  cfar_detection: 检测到 %d 个峰值\n', length(dopplers));
    
    % 计算目标的信噪比
    if ~isempty(dopplers)
        % 信噪比 = 信号强度 - 噪声基底
        snrs = heatmap_log(sub2ind(size(heatmap_log), dopplers, ranges)) - ...
               noise_floor_matrix(sub2ind(size(heatmap_log), dopplers, ranges));
    else
        snrs = [];  % 没有检测到目标时，返回空数组
    end
end