function config = load_config_from_gui(guiData)
    % 从GUI控件加载配置参数，生成完整的雷达配置结构体
    % 从GUI加载配置参数
    config = struct();
    % 获取RX和TX通道数
    rxVal = get(guiData.rxChannels, 'Value');
    txVal = get(guiData.txChannels, 'Value');
    
    % 获取其他参数 - 添加空值检查
    chirpsStr = get(guiData.chirpsPerFrame, 'String');
    if isempty(chirpsStr), chirpsStr = '128'; end
    config.NUM_CHIRPS = str2double(chirpsStr);
    
    frameStr = get(guiData.framePeriod, 'String');
    if isempty(frameStr), frameStr = '200'; end
    config.FRAME_PERIOD_MS = str2double(frameStr);
    
    startFreqStr = get(guiData.startFreq, 'String');
    if isempty(startFreqStr), startFreqStr = '77'; end
    config.START_FREQ_GHZ = str2double(startFreqStr);
    
    idleStr = get(guiData.idleTime, 'String');
    if isempty(idleStr), idleStr = '34'; end
    config.IDLE_TIME_US = str2double(idleStr);
    
    adcStartStr = get(guiData.adcStartTime, 'String');
    if isempty(adcStartStr), adcStartStr = '6'; end
    config.ADC_START_TIME_US = str2double(adcStartStr);
    
    rampStr = get(guiData.rampEndTime, 'String');
    if isempty(rampStr), rampStr = '66'; end
    config.RAMP_END_TIME_US = str2double(rampStr);
    
    slopeStr = get(guiData.freqSlope, 'String');
    if isempty(slopeStr), slopeStr = '60.012'; end
    config.FREQ_SLOPE_MHZ_PER_US = str2double(slopeStr);
    
    samplesStr = get(guiData.samplesPerChirp, 'String');
    if isempty(samplesStr), samplesStr = '256'; end
    config.NUM_ADC_SAMPLES = str2double(samplesStr);
    
    rateStr = get(guiData.sampleRate, 'String');
    if isempty(rateStr), rateStr = '5000'; end
    config.SAMPLE_RATE_KSPS = str2double(rateStr);
    
    % 验证基本参数有效性
    basic_params = [config.NUM_CHIRPS, config.FRAME_PERIOD_MS, config.START_FREQ_GHZ, ...
                    config.IDLE_TIME_US, config.ADC_START_TIME_US, config.RAMP_END_TIME_US, ...
                    config.FREQ_SLOPE_MHZ_PER_US, config.NUM_ADC_SAMPLES, config.SAMPLE_RATE_KSPS];
                
    if any(isnan(basic_params)) || any(basic_params <= 0)
        error('配置参数包含无效数值，请检查输入');
    end
    
    % =================================================================
    % 1. 雷达硬件参数 (与 load_config.m 对应)
    % =================================================================
    config.NUM_RX = rxVal;                          % RX天线数量
    config.NUM_TX = txVal;                           % TX天线数量
    config.VIRT_ANT = config.NUM_RX * config.NUM_TX; % 虚拟天线数量
    config.NUM_FRAMES = 100;                          % 处理的帧数
    config.DATA_FORMAT = 'real';                      % 数据格式：实部（AWR2944只有实部）
    
    % =================================================================
    % 2. 计算关键性能参数 (使用计算值)
    % =================================================================
    c = 3e8;  % 光速
    
    % 带宽计算: B = freqSlope * (rampEndTime - adcStartTime)
    % freqSlope 单位是 MHz/us，需要转换为 Hz/s: * 1e12
    % rampEndTime - adcStartTime 单位是 us，需要转换为 s: * 1e-6
    bandwidth = config.FREQ_SLOPE_MHZ_PER_US * 1e12 * (config.RAMP_END_TIME_US - config.ADC_START_TIME_US) * 1e-6;
    if bandwidth <= 0
        error('带宽计算无效，请检查rampEndTime和adcStartTime');
    end
    
    % 距离分辨率: ΔR = c / (2 * B) - 使用计算值
    config.RANGE_RESOLUTION = c / (2 * bandwidth);
    
    % 最大距离: Rmax = (Fs * c) / (2 * freqSlope)
    config.MAX_RANGE = (config.SAMPLE_RATE_KSPS * 1e3 * c) / (2 * config.FREQ_SLOPE_MHZ_PER_US * 1e12);
    
    % 波长
    lambda = c / (config.START_FREQ_GHZ * 1e9);
    
    % Chirp持续时间
    chirp_duration = (config.IDLE_TIME_US + config.RAMP_END_TIME_US) * 1e-6;
    
    % 帧持续时间
    frame_duration = config.NUM_CHIRPS * chirp_duration;
    
    % 多普勒分辨率: Δv = λ / (2 * Tf) - 使用计算值
    config.DOPPLER_RESOLUTION = lambda / (2 * frame_duration);
    
    % 最大速度: vmax = λ / (4 * Tc)
    config.MAX_VELOCITY = lambda / (4 * chirp_duration);
    
    % =================================================================
    % 3. 角度处理参数 (与 load_config.m 对应)
    % =================================================================
    config.SKIP_SIZE = 4;                            % 边缘跳过的单元数
    config.ANGLE_RES = 1;                              % 角度分辨率（度）
    config.ANGLE_RANGE = 90;                            % 角度范围（度）- 与load_config一致
    config.ANGLE_BINS = (config.ANGLE_RANGE * 2) / config.ANGLE_RES + 1;  % 角度bin数量
    config.BINS_PROCESSED = 100;                        % 处理的距离单元数量（与load_config一致）
    
    % =================================================================
    % 4. CFAR检测参数 (与 load_config.m 对应)
    % =================================================================
    config.GUARD_LEN = 4;                              % 保护单元长度
    config.NOISE_LEN = 20;                              % 噪声参考单元长度（与load_config一致）
    config.L_BOUND_AZIMUTH = 1;                         % 方位角CFAR左边界
    config.L_BOUND_RANGE = 3;                           % 距离方向CFAR阈值（与load_config一致）
    config.L_BOUND_DOPPLER = 3.5;                        % 多普勒方向CFAR阈值（与load_config一致）
    
    % =================================================================
    % 5. 数据文件路径 (与 load_config.m 对应)
    % =================================================================
    config.DATA_FILE = '.\adc_data_Raw_0.bin';          % 数据文件路径（与load_config一致）
    
    % =================================================================
    % 6. 验证必要字段是否存在
    % =================================================================
    required_fields = {'NUM_CHIRPS', 'NUM_ADC_SAMPLES', 'NUM_RX', 'NUM_TX', ...
                       'RANGE_RESOLUTION', 'DOPPLER_RESOLUTION', 'ANGLE_RANGE', ...
                       'ANGLE_RES', 'ANGLE_BINS', 'BINS_PROCESSED', 'GUARD_LEN', ...
                       'NOISE_LEN', 'L_BOUND_RANGE', 'L_BOUND_DOPPLER'};
    
    missing_fields = {};
    for i = 1:length(required_fields)
        if ~isfield(config, required_fields{i})
            missing_fields{end+1} = required_fields{i};
        end
    end
    
    if ~isempty(missing_fields)
        warning('配置缺少以下字段，将使用默认值: %s', strjoin(missing_fields, ', '));
    end
    
    % =================================================================
    % 7. 输出调试信息
    % =================================================================
    fprintf('\n=== load_config_from_gui 配置加载完成 ===\n');
    fprintf('【雷达硬件参数】\n');
    fprintf('  NUM_RX: %d, NUM_TX: %d, VIRT_ANT: %d\n', ...
        config.NUM_RX, config.NUM_TX, config.VIRT_ANT);
    fprintf('  NUM_CHIRPS: %d, NUM_ADC_SAMPLES: %d\n', ...
        config.NUM_CHIRPS, config.NUM_ADC_SAMPLES);
    fprintf('  NUM_FRAMES: %d\n', config.NUM_FRAMES);
    fprintf('  DATA_FORMAT: %s\n', config.DATA_FORMAT);
    
    fprintf('\n【性能参数 - 计算值】\n');
    fprintf('  RANGE_RESOLUTION: %.4f m\n', config.RANGE_RESOLUTION);
    fprintf('  MAX_RANGE: %.2f m\n', config.MAX_RANGE);
    fprintf('  DOPPLER_RESOLUTION: %.4f m/s\n', config.DOPPLER_RESOLUTION);
    fprintf('  MAX_VELOCITY: %.2f m/s\n', config.MAX_VELOCITY);
    
    fprintf('\n【角度处理参数】\n');
    fprintf('  SKIP_SIZE: %d\n', config.SKIP_SIZE);
    fprintf('  ANGLE_RES: %d°, ANGLE_RANGE: %d°, ANGLE_BINS: %d\n', ...
        config.ANGLE_RES, config.ANGLE_RANGE, config.ANGLE_BINS);
    fprintf('  BINS_PROCESSED: %d\n', config.BINS_PROCESSED);
    
    fprintf('\n【CFAR检测参数】\n');
    fprintf('  GUARD_LEN: %d, NOISE_LEN: %d\n', ...
        config.GUARD_LEN, config.NOISE_LEN);
    fprintf('  L_BOUND_RANGE: %.1f, L_BOUND_DOPPLER: %.1f\n', ...
        config.L_BOUND_RANGE, config.L_BOUND_DOPPLER);
    
    fprintf('\n【数据文件】\n');
    fprintf('  DATA_FILE: %s\n', config.DATA_FILE);
    fprintf('=====================================\n\n');
    
end