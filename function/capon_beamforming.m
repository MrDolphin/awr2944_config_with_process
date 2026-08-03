% =========================================================================
% Capon 波束形成函数
% =========================================================================
% 功能说明：
%   该函数使用 Capon 波束形成算法计算距离-角度谱，提高角度分辨率
%   相比于传统的 FFT 波束形成，Capon 波束形成具有更高的角度分辨率
% 
% 输入参数：
%   radar_cube - 雷达立方体数据，维度为 [num_chirps, num_virt_ant, num_samples]
%   steering_vec -  steering 向量矩阵，维度为 [num_angles, num_virt_ant]
%   config - 雷达配置参数
% 
% 输出参数：
%   range_azimuth - 距离-角度谱，维度为 [ANGLE_BINS, BINS_PROCESSED]
%   beam_weights - 波束权重，维度为 [num_virt_ant, ANGLE_BINS, BINS_PROCESSED]
% 
% 实现原理：
%   1. 对每个距离单元，计算接收信号的协方差矩阵
%   2. 对每个角度，计算 Capon 波束权重
%   3. 计算每个角度的信号功率
%   4. 构建距离-角度谱
% =========================================================================

function [range_azimuth, beam_weights] = capon_beamforming(radar_cube, steering_vec, config)
    % 获取雷达立方体的维度
    [num_chirps, num_virt_ant, num_samples] = size(radar_cube);
    
    % 验证虚拟天线数量是否与配置匹配
    if num_virt_ant ~= config.VIRT_ANT
        warning('实际虚拟天线数(%d)与配置(%d)不匹配', num_virt_ant, config.VIRT_ANT);
    end
    
    % 确定要处理的距离单元数量
    bins_processed = min(config.BINS_PROCESSED, num_samples);
    
    % 初始化距离-角度谱和波束权重
    range_azimuth = zeros(config.ANGLE_BINS, bins_processed);
    beam_weights = zeros(num_virt_ant, config.ANGLE_BINS, bins_processed, 'like', 1i);
    
    % 遍历每个距离单元
    for i = 1:bins_processed
        % 提取当前距离单元的数据并转置
        x = squeeze(radar_cube(:, :, i))';  % 维度变为 [num_virt_ant, num_chirps]
        
        % =================================================================
        % 1. 计算协方差矩阵
        % =================================================================
        Rxx = x * x' / size(x, 2);  % 计算样本协方差矩阵
        
        % 添加正则化项，提高矩阵求逆的稳定性
        % 对于较小的天线数（如4），可能需要调整正则化参数
        reg_param = 0.001 * trace(Rxx) / size(Rxx, 1);
        Rxx_inv = inv(Rxx + reg_param * eye(size(Rxx)));
        
        % 初始化角度维度的变量
        den = zeros(config.ANGLE_BINS, 1);  % 存储每个角度的分母项
        weights = zeros(num_virt_ant, config.ANGLE_BINS, 'like', 1i);  % 存储每个角度的波束权重
        
        % =================================================================
        % 2. 对每个角度计算波束权重和功率
        % =================================================================
        for theta_idx = 1:config.ANGLE_BINS
            % 获取当前角度的 steering 向量
            a = steering_vec(theta_idx, :)';
            
            % 计算当前角度的波束权重
            numerator = Rxx_inv * a;
            denominator = real(a' * numerator);
            
            % 避免分母为零
            if denominator > 0
                den(theta_idx) = 1 / denominator;
                weights(:, theta_idx) = numerator * den(theta_idx);
            else
                den(theta_idx) = 0;
                weights(:, theta_idx) = zeros(num_virt_ant, 1);
            end
        end
        
        % =================================================================
        % 3. 更新距离-角度谱和波束权重
        % =================================================================
        range_azimuth(:, i) = abs(den);  % 存储当前距离单元的角度谱
        beam_weights(:, :, i) = weights;  % 存储当前距离单元的波束权重
    end
    
    fprintf('Capon波束形成完成: %d个角度 × %d个距离单元\n', config.ANGLE_BINS, bins_processed);
end