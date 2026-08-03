% =========================================================================
% 静态杂波滤除函数 - 室内环境优化版
% =========================================================================
% 功能说明：
%   该函数通过多种方法滤除静态杂波，减少室内环境中的墙体、玻璃等反射
%   采用优化算法，保持高效运行
% 
% 输入参数：
%   radar_cube - 雷达立方体数据，维度为 [num_chirps, num_rx, num_samples]
% 
% 输出参数：
%   radar_cube - 滤除杂波后的雷达立方体数据
% 
% 实现原理：
%   1. 减去均值（标准方法）
%   2. 高通滤波（一阶差分）
%   3. 自适应阈值抑制
%   4. 所有操作均使用向量化实现，保证效率
% =========================================================================

function radar_cube = clutter_removal(radar_cube)
    % 获取数据维度
    [num_chirps, num_rx, num_samples] = size(radar_cube);
    
    %% 方法1：标准均值滤除（最基础、最高效）
    % 计算所有chirp的均值（沿第一个维度）
    mean_val = mean(radar_cube, 1);
    % 减去均值，去除静态分量
    radar_cube = radar_cube - mean_val;
    
    % %% 方法2：高通滤波 - 使用向量化操作
    % % 对每个天线-距离单元执行一阶差分（高通滤波）
    % % 这可以进一步去除缓慢变化的静态分量
    % 
    % % 方法2a：简单差分（效率最高）
    % % 创建差分矩阵 [0; diff(data)]
    % radar_cube_diff = [zeros(1, num_rx, num_samples); 
    %                    diff(radar_cube, 1, 1)];
    % 
    % % 混合原始信号和差分信号，平衡静态抑制和信号保留
    % alpha = 0.7;  % 差分信号权重
    % radar_cube = (1 - alpha) * radar_cube + alpha * radar_cube_diff;
    % 
    %% 方法3：自适应幅度抑制（针对强静态反射）
    % 计算每个距离单元的平均功率
    power_profile = squeeze(mean(abs(radar_cube).^2, [1, 2]));

    % 找到功率特别强的距离单元（可能是墙体）
    power_threshold = median(power_profile) * 3;  % 3倍中值作为阈值
    strong_reflections = power_profile > power_threshold;

    % 对强反射单元进行额外抑制
    if any(strong_reflections)
        % 创建抑制掩码
        suppression_mask = ones(1, 1, num_samples);
        suppression_mask(1, 1, strong_reflections) = 0.5;  % 强反射单元衰减到30%

        % 应用抑制（向量化操作）
        radar_cube = radar_cube .* suppression_mask;
    end
    
    % %% 方法4：零多普勒抑制（针对完全静止的物体）
    % % 计算每个距离-天线单元的多普勒谱（快速FFT）
    % % 只对零频附近进行抑制
    % 
    % % 对每个天线-距离单元，沿chirp维度做FFT
    % doppler_spectrum = fft(radar_cube, [], 1);
    % 
    % % 找到零多普勒频点（第一个频点）
    % zero_doppler = 1;
    % 
    % % 计算零多普勒能量的比例
    % zero_doppler_energy = abs(doppler_spectrum(zero_doppler, :, :)).^2;
    % total_energy = sum(abs(doppler_spectrum).^2, 1);
    % 
    % % 避免除零
    % total_energy(total_energy == 0) = eps;
    % 
    % % 计算静态能量比例
    % static_ratio = zero_doppler_energy ./ total_energy;
    % 
    % % 对静态比例高的单元进行抑制
    % static_threshold = 0.6;  % 60%以上能量在零多普勒认为是静态
    % static_units = static_ratio > static_threshold;
    % 
    % % 对静态单元进行衰减（但不完全消除，保留移动目标）
    % if any(static_units(:))
    %     % 创建衰减因子
    %     attenuation = ones(size(radar_cube));
    % 
    %     % 将静态单元对应的所有chirp都衰减
    %     for chirp_idx = 1:num_chirps
    %         attenuation(chirp_idx, static_units) = 0.5;  % 衰减到50%
    %     end
    % 
    %     % 应用衰减
    %     radar_cube = radar_cube .* attenuation;
    % end
    % 
    % 
    % %% 性能统计（可选，调试用）
    % persistent frame_count
    % if isempty(frame_count)
    %     frame_count = 0;
    % end
    % frame_count = frame_count + 1;
    % 
    % if mod(frame_count, 10) == 0  % 每10帧输出一次
    %     % 计算抑制前后的能量对比
    %     energy_before = mean(abs(radar_cube(:)).^2);
    %     energy_after = mean(abs(radar_cube(:)).^2);
    % 
    %     fprintf('  clutter_removal: 能量抑制比 %.2f dB\n', ...
    %         10*log10(energy_after / (energy_before + eps)));
    % end
end