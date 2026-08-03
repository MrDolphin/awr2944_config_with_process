%--------------------------------------------------------------------------
% 函数名称: convert_to_units
% 函数功能: 将雷达原始测量值转换为物理单位
% 输入参数:
%   - ranges: 距离索引或原始距离值
%   - azimuths: 方位角索引
%   - dopplers: 多普勒速度值
%   - config: 雷达配置结构体，包含分辨率和角度范围等参数
% 输出参数:
%   - ranges_m: 转换为米的距离值
%   - azimuths_rad: 转换为弧度的方位角值
%   - dopplers_ms: 转换为米/秒的多普勒速度值
% 函数原理:
%   1. 距离转换：乘以距离分辨率得到米单位
%   2. 方位角转换：将角度索引转换为实际角度（弧度）
%   3. 多普勒速度：保持原始单位（假设已经是米/秒）
% 方位角转换说明:
%   - 角度范围从 -ANGLE_RANGE/2 到 +ANGLE_RANGE/2
%   - 转换公式：(索引 - 中心点) * (角度分辨率) * (弧度转换因子)
% 应用场景:
%   - 雷达信号处理后的数据单位转换
%   - 目标跟踪和可视化前的数据预处理
%--------------------------------------------------------------------------
function [ranges_m, azimuths_rad, dopplers_ms] = convert_to_units(ranges, azimuths, dopplers, config)
    % 距离转换：索引乘以距离分辨率得到米单位
    ranges_m = ranges * config.RANGE_RESOLUTION;
    
    % 修复方位角转换：将索引转换为角度（弧度）
    % 角度范围从 -ANGLE_RANGE/2 到 +ANGLE_RANGE/2
    % 计算步骤：
    % 1. 计算角度索引的中心点偏移
    % 2. 乘以角度分辨率 (ANGLE_RANGE / ANGLE_BINS)
    % 3. 转换为弧度单位 (pi / 180)
    azimuths_rad = (azimuths - (config.ANGLE_BINS + 1) / 2) * (config.ANGLE_RANGE / config.ANGLE_BINS) * (pi / 180);
    
    % 多普勒速度处理：保持原始单位（假设已经是米/秒）
    if ~isempty(dopplers)
        dopplers_ms = dopplers;
    else
        dopplers_ms = [];
    end
end
