%--------------------------------------------------------------------------
% 函数名称: cfar_ca
% 函数功能: 实现单元平均恒虚警率(CFAR)检测算法
% 输入参数:
%   - x: 输入信号向量，通常是距离或多普勒谱
%   - guard_len: 保护单元长度，用于避免目标信号影响噪声估计
%   - noise_len: 噪声参考单元长度，用于估计背景噪声
%   - l_bound: 检测阈值偏移量
% 输出参数:
%   - threshold: 计算得到的CFAR阈值
%   - noise_floor: 估计的噪声基底
% 函数原理:
%   1. 将输入向量统一转换为行向量处理
%   2. 创建CFAR处理的卷积核，保护区域设置为0权重
%   3. 对输入信号进行循环填充，处理边界效应
%   4. 使用卷积计算噪声估计
%   5. 计算CFAR阈值
%   6. 恢复输出向量的原始形状
% 实现特点:
%   - 严格按照Python版本的CFAR实现
%   - 支持行向量和列向量输入
%   - 处理边界效应的循环填充
%   - 使用卷积方法提高计算效率
% 应用场景:
%   - 雷达目标检测中的阈值计算
%   - 信号处理中背景噪声估计
%--------------------------------------------------------------------------
function [threshold, noise_floor] = cfar_ca(x, guard_len, noise_len, l_bound)
    % 1. 统一处理向量：将所有输入转换为行向量
    was_column = iscolumn(x);  % 检查输入是否为列向量
    x_row = x(:)';             % 转换为行向量
    
    % 2. 创建与Python完全相同的卷积核
    kernel_len = 1 + (2 * guard_len) + (2 * noise_len);  % 卷积核总长度
    kernel = ones(1, kernel_len) / (2 * noise_len);      % 初始化权重
    kernel(noise_len+1:noise_len+(2*guard_len)+1) = 0;    % 设置保护区域权重为0
    
    % 3. 正确的循环填充：处理边界效应
    x_wrapped = [x_row(end-noise_len+1:end), x_row, x_row(1:noise_len)];
    
    % 4. 使用 'same' 模式卷积计算噪声估计
    noise_floor_full = conv(x_wrapped, kernel, 'same');
    
    % 5. 提取有效部分：去除填充区域
    noise_floor = noise_floor_full(noise_len+1:end-noise_len);
    
    % 6. 计算阈值：噪声基底加上偏移量
    threshold = noise_floor + l_bound;
    
    % 7. 恢复原始形状：保持输入输出形状一致
    if was_column
        threshold = threshold';
        noise_floor = noise_floor';
    end
    
    % 调试信息：显示CFAR处理结果
    %fprintf('    CFAR处理: 输入长度=%d, 输出长度=%d, 门限=%.2f\n', ...
    %    length(x), length(threshold), l_bound);
end