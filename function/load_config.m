function config = load_config()


    % =================================================================
    % 1. 雷达硬件参数
    % =================================================================
    config.NUM_RX = 4;  % RX天线数量
    config.NUM_TX = 1;  % TX天线数量（改为1）
    config.VIRT_ANT = 4;  % 虚拟天线数量（改为1*4=4）
    config.NUM_CHIRPS = 128;  % 每个帧的chirp数量
    config.NUM_ADC_SAMPLES = 256;  % 每个chirp的ADC采样数
    config.NUM_FRAMES = 100;  % 处理的帧数
    config.RANGE_RESOLUTION = 0.0416;  % 距离分辨率（米）
    config.DOPPLER_RESOLUTION = 0.03;  % 多普勒分辨率（米/秒）
    config.DATA_FORMAT = 'real';  % 数据格式：实部（AWR2944只有实部）
    

    % =================================================================
    % 2. 角度处理参数
    % =================================================================
    config.SKIP_SIZE = 4;  % 边缘跳过的单元数
    config.ANGLE_RES = 1;  % 角度分辨率（度）
    config.ANGLE_RANGE = 90;  % 角度范围（度）
    config.ANGLE_BINS = (config.ANGLE_RANGE * 2) / config.ANGLE_RES + 1;
    config.BINS_PROCESSED = 100;  % 处理的距离单元数量
    
    % =================================================================
    % 3. CFAR检测参数
    % =================================================================
    config.GUARD_LEN = 4; % 保护单元长度
    config.NOISE_LEN = 20; % 噪声参考单元长度
    config.L_BOUND_AZIMUTH = 1;
    config.L_BOUND_RANGE = 3; % 距离方向CFAR阈值
    config.L_BOUND_DOPPLER = 3.5;  % 多普勒方向CFAR阈值（新增）
    

    % =================================================================
    % 4. 数据文件路径
    % =================================================================
    config.DATA_FILE = '.\adc_data_Raw_0.bin';  % 改为您的AWR2944数据文件
    
    fprintf('AWR2944配置加载完成 - 1发4收，实部数据\n');
end