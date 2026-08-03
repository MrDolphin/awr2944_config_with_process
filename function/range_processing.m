% =========================================================================
% 距离处理函数
% =========================================================================
% 功能说明：
%   该函数对雷达ADC数据执行距离FFT处理，生成雷达立方体
%   可选地应用窗函数以减少频谱泄漏
% 
% 输入参数：
%   adc_data - ADC数据，维度为 [num_chirps, num_rx, num_samples]
%   window_type_1d - 可选，窗函数类型，如 'hanning', 'hamming', 'blackman' 等
% 
% 输出参数：
%   radar_cube - 距离FFT结果，即雷达立方体，维度与输入相同
% =========================================================================

function radar_cube = range_processing(adc_data, window_type_1d)
    % 检查输入参数数量
    if nargin < 2
        window_type_1d = [];  % 如果未提供窗函数类型，设为空
    end
    
    % 获取ADC数据的维度
    [num_chirps, num_rx, num_samples] = size(adc_data);
    
    % 应用窗函数（如果指定）
    if ~isempty(window_type_1d)
        fft1d_in = apply_windowing(adc_data, window_type_1d, 3);
    else
        fft1d_in = adc_data;  % 不应用窗函数
    end
    
    % 执行距离FFT（沿第三个维度，即样本维度）
    radar_cube = fft(fft1d_in, [], 3);
    

    
end

% =========================================================================
% 应用窗函数函数
% =========================================================================
% 功能说明：
%   该函数对数据应用指定类型的窗函数，以减少频谱泄漏
% 
% 输入参数：
%   data - 输入数据，维度为 [num_chirps, num_rx, num_samples]
%   window_type - 窗函数类型
%   axis - 应用窗函数的轴
% 
% 输出参数：
%   windowed_data - 应用窗函数后的数据
% =========================================================================

function windowed_data = apply_windowing(data, window_type, axis)
    % 获取数据维度
    [num_chirps, num_rx, num_samples] = size(data);
    
    % 根据指定的窗函数类型生成窗函数
    switch lower(window_type)
        case 'bartlett'
            window = bartlett(num_samples);  % Bartlett窗
        case 'blackman'
            window = blackman(num_samples);  % Blackman窗
        case 'hanning'
            window = hanning(num_samples);  % Hanning窗
        case 'hamming'
            window = hamming(num_samples);  % Hamming窗
        otherwise
            window = ones(num_samples, 1);  % 矩形窗（无窗）
    end
    
    % 将窗函数应用到数据上
    windowed_data = data .* reshape(window, [1, 1, num_samples]);
end
