%--------------------------------------------------------------------------
% 函数名称: interleave_tx
% 函数功能: 对雷达数据立方体进行发射天线数据交织处理
% 输入参数:
%   - radar_cube: 原始雷达数据立方体，维度为 [chirps x rx x samples]
%   - num_tx: 发射天线数量
% 输出参数:
%   - radar_cube_interleaved: 交织后的雷达数据立方体
% 函数原理:
%   1. 从原始雷达数据中分离不同发射天线的数据
%   2. 将不同发射天线的数据在接收天线维度上拼接
%   3. 实现发射天线数据的交织处理
% 处理说明:
%   - 假设发射天线交替工作（TX1, TX2, TX1, TX2...）
%   - 提取奇数chirp作为TX1数据
%   - 提取偶数chirp作为TX2数据
%   - 在接收天线维度上拼接TX1和TX2数据
% 输入输出维度:
%   - 输入: [num_chirps x num_rx x num_samples]
%   - 输出: [num_chirps/2 x (num_rx*2) x num_samples]
% 应用场景:
%   - 多发射天线雷达的数据处理
%   - 发射天线数据的分离和重组
%--------------------------------------------------------------------------
function radar_cube_interleaved = interleave_tx(radar_cube, num_tx)
    % 获取雷达数据立方体的维度
    [num_chirps, num_rx, num_samples] = size(radar_cube);
    
    % 根据发射天线数量进行不同处理
    if num_tx == 1
        % AWR2944: 1发4收，无需交织处理
        radar_cube_interleaved = radar_cube;
        fprintf('单发天线模式: 无需交织处理，虚拟天线数: %d\n', num_rx);
        
    elseif num_tx == 2
        % 2发4收，需要交织处理
        % 分离不同发射天线的数据
        chirp_tx1 = radar_cube(1:2:end, :, :);  % 提取奇数chirp作为TX1数据
        chirp_tx2 = radar_cube(2:2:end, :, :);  % 提取偶数chirp作为TX2数据
        
        % 在接收天线维度上拼接TX1和TX2数据
        radar_cube_interleaved = cat(2, chirp_tx1, chirp_tx2);
        fprintf('双发天线模式: 完成数据交织，虚拟天线数: %d\n', size(radar_cube_interleaved, 2));
        
    elseif num_tx == 3
        % 3发4收模式
        % 假设发射天线顺序为 TX1, TX2, TX3, TX1, TX2, TX3, ...
        % 确保chirp数量是3的倍数
        if mod(num_chirps, 3) ~= 0
            warning('3发模式要求chirp数是3的倍数，当前chirp数: %d', num_chirps);
            % 截取到3的倍数
            num_chirps = num_chirps - mod(num_chirps, 3);
            radar_cube = radar_cube(1:num_chirps, :, :);
        end
        
        % 分离三个发射天线的数据
        chirp_tx1 = radar_cube(1:3:end, :, :);  % TX1数据
        chirp_tx2 = radar_cube(2:3:end, :, :);  % TX2数据
        chirp_tx3 = radar_cube(3:3:end, :, :);  % TX3数据
        
        % 在接收天线维度上拼接所有TX数据
        % 最终维度：[num_chirps/3 x (num_rx*3) x num_samples]
        radar_cube_interleaved = cat(2, chirp_tx1, chirp_tx2, chirp_tx3);
        fprintf('三发天线模式: 完成数据交织，虚拟天线数: %d\n', size(radar_cube_interleaved, 2));
        
    elseif num_tx == 4
        % 4发4收模式
        % 假设发射天线顺序为 TX1, TX2, TX3, TX4, TX1, TX2, TX3, TX4, ...
        % 确保chirp数量是4的倍数
        if mod(num_chirps, 4) ~= 0
            warning('4发模式要求chirp数是4的倍数，当前chirp数: %d', num_chirps);
            % 截取到4的倍数
            num_chirps = num_chirps - mod(num_chirps, 4);
            radar_cube = radar_cube(1:num_chirps, :, :);
        end
        
        % 分离四个发射天线的数据
        chirp_tx1 = radar_cube(1:4:end, :, :);  % TX1数据
        chirp_tx2 = radar_cube(2:4:end, :, :);  % TX2数据
        chirp_tx3 = radar_cube(3:4:end, :, :);  % TX3数据
        chirp_tx4 = radar_cube(4:4:end, :, :);  % TX4数据
        
        % 在接收天线维度上拼接所有TX数据
        radar_cube_interleaved = cat(2, chirp_tx1, chirp_tx2, chirp_tx3, chirp_tx4);
        fprintf('四发天线模式: 完成数据交织，虚拟天线数: %d\n', size(radar_cube_interleaved, 2));
        
    else
        error('不支持的发射天线数量: %d (支持: 1, 2, 3, 4)', num_tx);
    end
end