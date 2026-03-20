% =========================================================================
% 加载雷达数据函数
% =========================================================================
% 功能说明：
%   该函数从二进制文件中加载雷达数据，并组织成合适的格式
%   包括原始数据和结构化的复数数据
% 
% 输入参数：
%   config - 雷达配置参数
% 
% 输出参数：
%   adc_data - 原始ADC数据
%   all_data - 组织后的复数数据，维度为 [NUM_FRAMES, num_chirps_per_frame, NUM_RX, NUM_ADC_SAMPLES]
% 
% 实现原理：
%   1. 从文件中读取原始uint16数据
%   2. 计算每帧的数据大小和总帧数
%   3. 检查请求的帧数是否超过实际可用帧数
%   4. 对每帧数据进行组织和格式转换
%   5. 返回原始数据和组织后的复数数据
% =========================================================================
function [adc_data, all_data] = load_radar_data(config)
    fprintf('正在读取AWR2944二进制文件: %s\n', config.DATA_FILE);
    
    fid = fopen(config.DATA_FILE, 'rb');
    if fid == -1
        error('无法打开文件: %s', config.DATA_FILE);
    end
    
    % 读取原始数据（uint16格式，只有实部）
    raw_data = fread(fid, 'uint16');
    fclose(fid);
    
    fprintf('原始数据大小: %d 个 uint16 样本\n', length(raw_data));
    
    % =================================================================
    % 计算数据帧相关参数
    % =================================================================
    num_tx = config.NUM_TX;  % 使用配置中的TX数量（1）
    num_chirps_per_frame = config.NUM_CHIRPS * num_tx;  % 每帧的chirp数量
    
    % AWR2944只有实部，每个样本占1个uint16
    if isfield(config, 'DATA_FORMAT') && strcmp(config.DATA_FORMAT, 'real')
        samples_per_frame = num_chirps_per_frame * config.NUM_RX * config.NUM_ADC_SAMPLES;
        fprintf('数据格式: 实部（AWR2944）\n');
    else
        % 兼容旧版本（复数数据）
        samples_per_frame = num_chirps_per_frame * config.NUM_RX * config.NUM_ADC_SAMPLES * 2;
        fprintf('数据格式: 复数（AWR1642）\n');
    end
    
    num_frames = floor(length(raw_data) / samples_per_frame);
    fprintf('检测到 %d 帧数据\n', num_frames);
    
    if num_frames < config.NUM_FRAMES
        warning('请求数据帧数 %d 超过实际帧数 %d，将使用所有可用数据', config.NUM_FRAMES, num_frames);
        config.NUM_FRAMES = num_frames;
    end
    
    fprintf('正在组织AWR2944数据...\n');
    
    % 初始化输出数据
    all_data = zeros(config.NUM_FRAMES, num_chirps_per_frame, config.NUM_RX, config.NUM_ADC_SAMPLES, 'like', 1i);
    
    % 处理每一帧数据
    for frame_idx = 1:config.NUM_FRAMES
        start_idx = (frame_idx - 1) * samples_per_frame + 1;
        end_idx = frame_idx * samples_per_frame;
        raw_frame = raw_data(start_idx:end_idx);
        
        % 组织当前帧数据
        all_data(frame_idx, :, :, :) = organize_frame_awr2944(raw_frame, num_chirps_per_frame, config.NUM_RX, config.NUM_ADC_SAMPLES);
    end
    
    adc_data = raw_data(1:config.NUM_FRAMES * samples_per_frame);
    
    fprintf('数据组织完成!\n');
    fprintf('输出数据形状: [%d, %d, %d, %d]\n', size(all_data, 1), size(all_data, 2), size(all_data, 3), size(all_data, 4));
end

% =========================================================================
% AWR2944数据组织函数（只有实部）
% =========================================================================
function frame = organize_frame_awr2944(raw_frame, num_chirps, num_rx, num_samples)
    raw_len = length(raw_frame);

    % AWR2944只有实部数据，转换为复数（虚部为0）
    % 直接将实部数据转换为复数
    lvds = zeros(1, raw_len, 'like', 1i);

    for i = 1:raw_len
        lvds(1, i) = complex(double(raw_frame(i)), 0);
    end

    % 重塑数据形状
    lvds = reshape(lvds, num_rx * num_samples, num_chirps);
    lvds = lvds';  % 转置为 [num_chirps, num_rx * num_samples]

    % 进一步组织数据
    adc_data = zeros(num_rx, num_chirps * num_samples);

    for row = 1:num_rx
        for i = 1:num_chirps
            adc_data(row, (i-1)*num_samples+1:i*num_samples) = lvds(i, (row-1)*num_samples+1:row*num_samples);
        end
    end

    % 最终组织数据形状
    frame = reshape(adc_data', [num_samples, num_chirps, num_rx]);
    frame = permute(frame, [2, 3, 1]);  % [num_chirps, num_rx, num_samples]
end


% =========================================================================
% 保留原函数用于兼容
% =========================================================================
function frame = organize_frame(raw_frame, num_chirps, num_rx, num_samples)
    % 调用AWR2944版本
    frame = organize_frame_awr2944(raw_frame, num_chirps, num_rx, num_samples);
end