%--------------------------------------------------------------------------
% 函数名称: find_cluster
% 函数功能: 从点云中查找未关联点的聚类
% 输入参数:
%   - points: 点云数据结构体数组，包含range、angle、doppler、snr等字段
%   - start_idx: 聚类起始点的索引
%   - num_points: 点云总数
%   - bestIndex: 关联索引数组，255表示未关联点
%   - tracker: 跟踪器结构体，包含分配参数
% 输出参数:
%   - alloc_num: 聚类中的点数量
%   - alloc_snr: 聚类的总SNR值
%   - un: 聚类的平均测量值 [距离, 角度, 多普勒速度]
% 函数原理:
%   1. 以起始点为聚类中心初始化
%   2. 遍历后续的未关联点
%   3. 检查速度一致性：速度差异小于阈值
%   4. 检查距离一致性：两点之间距离小于阈值
%   5. 将符合条件的点加入聚类并更新平均值
% 聚类条件:
%   - 速度差异 < maxVelThre
%   - 距离差异 < maxDistanceThre
% 应用场景:
%   - 新目标分配前的点云聚类
%   - 提高目标检测的稳定性和准确性
%--------------------------------------------------------------------------
function [alloc_num, alloc_snr, un] = find_cluster(points, start_idx, num_points, bestIndex, tracker)
    % 初始化聚类参数
    alloc_num = 1;  % 聚类点数初始化为1（起始点）
    alloc_snr = points(start_idx).snr;  % 初始化总SNR
    un = [points(start_idx).range, points(start_idx).angle, points(start_idx).doppler];  % 初始聚类中心
    un_sum = un;  % 聚类总和，用于计算平均值
    
    % 遍历后续点云，查找可加入聚类的点
    for k = start_idx+1:num_points
        if bestIndex(k) == 255 % 检查是否为未关联点
            uk = [points(k).range, points(k).angle, points(k).doppler];  % 当前点的测量值
            
            % 速度一致性检查：速度差异小于阈值
            vel_diff = abs(uk(3) - un(3));  % 计算速度差异
            if vel_diff < tracker.allocationParams.maxVelThre
                % 距离一致性检查：两点之间距离小于阈值
                % 使用余弦定理计算两点之间的距离
                dist = un(1)^2 + uk(1)^2 - 2*un(1)*uk(1)*cos(un(2)-uk(2));
                if dist < tracker.allocationParams.maxDistanceThre
                    % 将点加入聚类
                    alloc_num = alloc_num + 1;  % 更新聚类点数
                    alloc_snr = alloc_snr + points(k).snr;  % 累加SNR
                    un_sum = un_sum + uk;  % 累加测量值
                    un = un_sum / alloc_num;  % 更新聚类平均值
                end
            end
        end
    end
end