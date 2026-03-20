% =========================================================================
% 多普勒速度估计函数
% =========================================================================
% 功能说明：
%   该函数估计检测到的目标的多普勒速度
%   使用波束权重对信号进行聚焦，然后通过 FFT 估计多普勒频率
% 
% 输入参数：
%   radar_cube - 雷达立方体数据，维度为 [num_chirps, num_virt_ant, num_samples]
%   beam_weights - 波束权重，维度为 [num_virt_ant, ANGLE_BINS, BINS_PROCESSED]
%   range_indices - 检测到的目标的距离索引
%   azimuth_indices - 检测到的目标的角度索引
%   config - 雷达配置参数
% 
% 输出参数：
%   dopplers - 估计的目标多普勒速度，单位为 m/s
% 
% 实现原理：
%   1. 对每个检测到的目标，使用对应的波束权重对信号进行聚焦
%   2. 对聚焦后的信号执行多普勒 FFT
%   3. 找到 FFT 结果中的峰值，对应目标的多普勒频率
%   4. 将多普勒频率转换为速度
% =========================================================================

function dopplers = doppler_estimation(radar_cube, beam_weights, range_indices, azimuth_indices, config)
    % 检查是否有目标需要处理
    if isempty(range_indices)
        dopplers = [];  % 没有目标时返回空数组
        return;
    end
    
    % 获取雷达立方体的维度
    num_chirps = size(radar_cube, 1);  % chirp 数量
    num_virt_ant = size(radar_cube, 2);  % 虚拟天线数量
    num_ranges = length(range_indices);  % 目标数量
    
    % 验证虚拟天线数量
    if num_virt_ant ~= config.VIRT_ANT
        warning('doppler_estimation: 实际虚拟天线数(%d)与配置(%d)不匹配', num_virt_ant, config.VIRT_ANT);
    end
    
    % 初始化多普勒 FFT 输入
    doppler_fft_input = zeros(num_chirps, num_ranges, 'like', 1i);
    
    % =================================================================
    % 1. 对每个目标，使用波束权重聚焦信号
    % =================================================================
    for i = 1:num_ranges
        range_idx = range_indices(i);  % 当前目标的距离索引
        azimuth_idx = azimuth_indices(i);  % 当前目标的角度索引
        
        % 检查索引是否有效
        if range_idx > size(radar_cube, 3) || azimuth_idx > size(beam_weights, 2)
            warning('索引超出范围: 距离索引=%d, 角度索引=%d', range_idx, azimuth_idx);
            continue;
        end
        
        % 提取当前距离单元的数据
        x = squeeze(radar_cube(:, :, range_idx));  % 维度变为 [num_chirps, num_virt_ant]
        
        % 获取对应角度和距离的波束权重
        w = beam_weights(:, azimuth_idx, range_idx);  % 维度为 [num_virt_ant, 1]
        
        % 使用波束权重聚焦信号
        doppler_fft_input(:, i) = x * w;  % 维度变为 [num_chirps, 1]
    end
    
    % =================================================================
    % 2. 执行多普勒 FFT 并估计速度
    % =================================================================
    if size(doppler_fft_input, 2) > 0
        % 执行多普勒 FFT（沿 chirp 维度），应用加窗减少频谱泄漏
        window = hamming(num_chirps);
        doppler_fft_input_windowed = doppler_fft_input .* window;
        doppler_est = fft(doppler_fft_input_windowed, [], 1);
        
        % 找到每个目标的多普勒频率峰值
        [~, doppler_indices] = max(abs(doppler_est), [], 1);
        
        % 处理负频率（多普勒频率折叠）
        doppler_indices(doppler_indices > num_chirps/2) = doppler_indices(doppler_indices > num_chirps/2) - num_chirps;
        
        % 将多普勒索引转换为速度
        dopplers = doppler_indices * config.DOPPLER_RESOLUTION;
    else
        dopplers = [];  % 没有有效输入时返回空数组
    end
end