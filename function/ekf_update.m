% =========================================================================
% EKF 状态更新函数
% =========================================================================
% 功能说明：
%   该函数使用扩展卡尔曼滤波器（EKF）更新目标状态，并管理目标的生命周期
%   包括：
%   1. 收集每个目标的关联点云
%   2. 计算关联点云的统计特性
%   3. 使用 EKF 更新目标状态
%   4. 管理目标的状态转换（检测→激活→自由）
%   5. 清理消亡的目标
% 
% 输入参数：
%   inst - 跟踪器实例，包含目标状态和点云数据
% 
% 输出参数：
%   inst - 更新后的跟踪器实例，包含更新后的目标状态
% =========================================================================

function inst = ekf_update(inst)
    % 遍历每个跟踪目标
    for it = 1:inst.target_num_all
        % =================================================================
        % 1. 初始化变量
        % =================================================================
        u_mean.range = 0;  % 关联点云的平均距离
        u_mean.angle = 0;  % 关联点云的平均角度
        u_mean.vel = 0;  % 关联点云的平均速度
        power_sum = 0;  % 关联点云的功率总和
        myPointNum = 0;  % 关联点云的数量
        Rm = zeros(9, 1);  % 测量噪声协方差矩阵
        Rc = zeros(9, 1);  % 组合协方差矩阵
        U = zeros(3, 1);  % 测量向量
        D = zeros(9, 1);  % 点云分散度
        
        % =================================================================
        % 2. 收集关联的点云
        % =================================================================
        for ip = 1:inst.num_points
            % 检查点云是否关联到当前目标
            if(inst.point_cloud(ip).best_ind == it)
                myPointNum = myPointNum + 1;  % 增加关联点云计数
                u_mean.range = u_mean.range + inst.point_cloud(ip).range;  % 累加距离
                u_mean.angle = u_mean.angle + inst.point_cloud(ip).angle;  % 累加角度
                u_mean.vel = u_mean.vel + inst.point_cloud(ip).vel;  % 累加速度
                power_sum = power_sum + inst.point_cloud(ip).power;  % 累加功率
                
                % 注释掉的速度处理代码
                % if myPointNum == 1
                %     rvPilot = inst.point_cloud(ip).vel;
                %     u_mean.vel = rvPilot;
                % else
                %     rvCurrent = unroll_radvel(inst.velParams.maxradvel, rvPilot, inst.point_cloud(ip).vel);
                %     inst.point_cloud(ip).vel = rvCurrent;
                %     u_mean.vel = u_mean.vel + rvCurrent;
                % end
            end
        end
        
        % =================================================================
        % 3. 处理无关联点云的情况
        % =================================================================
        if(myPointNum == 0)
            % 如果没有关联的点云，使用预测状态作为当前状态
            inst.target(it).S_hat = inst.target(it).S_apriori_hat;
            inst.target(it).P_hat = inst.target(it).P_apriori_hat;
            inst.target(it).power_sum = 0;
            inst.target(it).power_mean = 0;
            
            % 判断目标的状态
            inst = event(inst, it, myPointNum);
            continue;  % 跳过后续处理，处理下一个目标
        end
        
        % =================================================================
        % 4. 计算关联点云的统计特性
        % =================================================================
        % 计算平均距离、角度和速度
        u_mean.range = u_mean.range / myPointNum;
        u_mean.angle = u_mean.angle / myPointNum;
        u_mean.vel = u_mean.vel / myPointNum;
        
        % 更新目标的功率信息
        inst.target(it).power_sum = power_sum;
        inst.target(it).power_mean = power_sum / myPointNum;
        
        % 更新目标的关联点云计数
        inst.target(it).associatedPoints = inst.target(it).associatedPoints + myPointNum;
        
        % =================================================================
        % 5. 计算测量噪声协方差
        % =================================================================
        dRangeVar = inst.variationParams.lengthStd^2;  % 距离方差
        dVelVar = inst.variationParams.velStd^2;  % 速度方差
        
        Rm(1) = dRangeVar;  % 距离噪声方差
        % 计算角度噪声方差（与距离相关）
        angleStd = 2 * atan(0.5 * inst.variationParams.widthStd / inst.target(it).Hs(1));
        Rm(5) = angleStd^2;  % 角度噪声方差
        Rm(9) = dVelVar;  % 速度噪声方差
        
        % 构建测量向量
        U(1) = u_mean.range;  % 平均距离
        U(2) = u_mean.angle;  % 平均角度
        U(3) = u_mean.vel;  % 平均速度
        
        % 注释掉的速度状态处理
        % [inst, U] = vel_state_handing(inst, it, U);
        
        % =================================================================
        % 6. 更新点云分散度
        % =================================================================
        if (myPointNum > inst.min_points_to_update_dispersion)
            % 计算点云的分散度
            for ip = 1:inst.num_points
                if(inst.point_cloud(ip).best_ind == it)
                    % 计算距离分散度
                    D(1) = D(1) + (inst.point_cloud(ip).range - u_mean.range)^2;
                    % 计算角度分散度
                    D(5) = D(5) + (inst.point_cloud(ip).angle - u_mean.angle)^2;
                    % 计算速度分散度
                    D(9) = D(9) + (inst.point_cloud(ip).vel - u_mean.vel)^2;
                    % 计算交叉项
                    D(2) = D(2) + (inst.point_cloud(ip).range - u_mean.range) * (inst.point_cloud(ip).angle - u_mean.angle);
                    D(3) = D(3) + (inst.point_cloud(ip).range - u_mean.range) * (inst.point_cloud(ip).vel - u_mean.vel);
                    D(6) = D(6) + (inst.point_cloud(ip).angle - u_mean.angle) * (inst.point_cloud(ip).vel - u_mean.vel);
                end
            end
            
            % 计算平均分散度
            D(1) = D(1) / myPointNum;
            D(5) = D(5) / myPointNum;
            D(9) = D(9) / myPointNum;
            D(2) = D(2) / myPointNum;
            D(3) = D(3) / myPointNum;
            D(6) = D(6) / myPointNum;
            
            % 计算更新权重
            alpha = myPointNum / inst.target(it).associatedPoints;
            if alpha < inst.dispersion_alpha
                alpha = inst.dispersion_alpha;  % 限制最小权重
            end
            
            % 更新目标的分散度（使用指数移动平均）
            inst.target(it).gD(1) = (1-alpha) * inst.target(it).gD(1) + alpha * D(1);
            inst.target(it).gD(2) = (1-alpha) * inst.target(it).gD(2) + alpha * D(2);
            inst.target(it).gD(3) = (1-alpha) * inst.target(it).gD(3) + alpha * D(3);
            inst.target(it).gD(4) = inst.target(it).gD(2);  % 对称项
            inst.target(it).gD(5) = (1-alpha) * inst.target(it).gD(5) + alpha * D(5);
            inst.target(it).gD(6) = (1-alpha) * inst.target(it).gD(6) + alpha * D(6);
            inst.target(it).gD(7) = inst.target(it).gD(3);  % 对称项
            inst.target(it).gD(8) = inst.target(it).gD(6);  % 对称项
            inst.target(it).gD(9) = (1-alpha) * inst.target(it).gD(9) + alpha * D(9);
        end
        
        % =================================================================
        % 7. 计算组合协方差
        % =================================================================
        if myPointNum > inst.est_points
            alpha = 0;  % 点云数量足够，不需要添加分散度
        else
            alpha = (inst.est_points - myPointNum) / ((inst.est_points - 1) * myPointNum);  % 计算分散度权重
        end
        % 计算组合协方差
        Rc(1) = Rm(1) / myPointNum + alpha * inst.target(it).gD(1);
        Rc(5) = Rm(5) / myPointNum + alpha * inst.target(it).gD(5);
        Rc(9) = Rm(9) / myPointNum + alpha * inst.target(it).gD(9);
        
        % =================================================================
        % 8. EKF 状态更新
        % =================================================================
        % 计算雅可比矩阵
        J = computeJacobian(inst.target(it).S_apriori_hat);
        P = inst.target(it).P_apriori_hat;  % 预测的协方差矩阵
        S = inst.target(it).S_apriori_hat;  % 预测的状态向量
        u_tilda = U - inst.target(it).Hs;  % 测量残差
        
        % 重塑协方差矩阵
        Rc = [Rc(1:3)'; Rc(4:6)'; Rc(7:9)'];
        % 计算卡尔曼增益
        cC_inv = inv(J * P * J' + Rc);
        K = P * J' * cC_inv;
        % 更新状态向量
        S_hat = S + K * u_tilda;
        % 更新协方差矩阵
        P_hat = P - K * (P * J')';
        
        % 计算目标的全局协方差矩阵
        gD = inst.target(it).gD;
        gD = [gD(1:3); gD(4:6); gD(7:9)];
        Rm = [Rm(1:3)'; Rm(4:6)'; Rm(7:9)'];
        gC_temp = gD + J * P * J' + Rm;
        gC_temp_inv = inv(gC_temp);
        
        % 更新目标状态
        inst.target(it).S_hat = S_hat;  % 更新状态向量
        inst.target(it).P_hat = P_hat;  % 更新协方差矩阵
        inst.target(it).gC = [gC_temp(1,:) gC_temp(2,:) gC_temp(3,:)];  % 更新全局协方差
        inst.target(it).gC_inv = [gC_temp_inv(1,:) gC_temp_inv(2,:) gC_temp_inv(3,:)];  % 更新全局协方差的逆
        
        % =================================================================
        % 9. 判断目标的状态
        % =================================================================
        inst = event(inst, it, myPointNum);
    end
    
    % =================================================================
    % 10. 清理消亡的目标
    % =================================================================
    active_num = 0;  % 活跃目标数量
    free_index = [];  % 自由状态目标的索引
    
    for it = 1:inst.target_num_all
        % 检查目标是否为自由状态
        if inst.target(it).state == inst.stateParams.free
            free_index = [free_index it];  % 记录自由状态目标的索引
            % trackID 入栈（回收）
            inst.trackID_all = [inst.target(it).trackID inst.trackID_all];
        end
        % 检查目标是否为活跃状态
        if inst.target(it).state == inst.stateParams.active
            active_num = active_num + 1;  % 增加活跃目标计数
        end
    end
    
    % 删除自由状态的目标
    inst.target(free_index) = [];
    % 更新目标数量
    inst.target_num_active = active_num;
    inst.target_num_all = length(inst.target);
end

% =========================================================================
% 计算雅可比矩阵函数
% =========================================================================
% 功能说明：
%   该函数计算从直角坐标系到极坐标系的雅可比矩阵
%   用于 EKF 中的线性化处理
% 
% 输入参数：
%   cart - 直角坐标系状态向量 [x, y, vx, vy, ax, ay]'
% 
% 输出参数：
%   J - 雅可比矩阵
% =========================================================================

function J = computeJacobian(cart)
    J_temp = zeros(1, 18);  % 初始化雅可比矩阵
    
    % 提取状态变量
    posx = cart(1);  % x 坐标
    posy = cart(2);  % y 坐标
    velx = cart(3);  % x 方向速度
    vely = cart(4);  % y 方向速度
    
    % 计算距离相关变量
    range2 = posx^2 + posy^2;  % 距离的平方
    range = sqrt(range2);  % 距离
    range3 = range * range2;  % 距离的立方
    
    % 计算雅可比矩阵元素
    % 距离对状态的偏导数
    J_temp(1) = posx / range;  % 距离对 x 的偏导数
    J_temp(2) = posy / range;  % 距离对 y 的偏导数
    % 角度对状态的偏导数
    J_temp(7) = posy / range2;  % 角度对 x 的偏导数
    J_temp(8) = -posx / range2;  % 角度对 y 的偏导数
    % 距离变化率对状态的偏导数
    J_temp(13) = posy * (velx * posy - vely * posx) / range3;  % 距离变化率对 x 的偏导数
    J_temp(14) = posx * (vely * posx - velx * posy) / range3;  % 距离变化率对 y 的偏导数
    J_temp(15) = posx / range;  % 距离变化率对 vx 的偏导数
    J_temp(16) = posy / range;  % 距离变化率对 vy 的偏导数
    
    % 重塑雅可比矩阵
    J = [J_temp(1:6); J_temp(7:12); J_temp(13:18)];
end

% =========================================================================
% 速度状态处理函数
% =========================================================================
% 功能说明：
%   该函数处理目标的速度状态，包括初始化、滤波和锁定
% 
% 输入参数：
%   inst - 跟踪器实例
%   it - 目标索引
%   um - 测量向量
% 
% 输出参数：
%   inst - 更新后的跟踪器实例
%   um - 更新后的测量向量
% =========================================================================

function [inst, um] = vel_state_handing(inst, it, um)
    rvIn = um(3);  % 输入速度
    
    if inst.target(it).vel_handing == inst.velParams.vel_init
        % 速度初始状态
        um(3) = inst.target(it).rangeRate;
        inst.target(it).vel_handing = inst.velParams.vel_rate_filter;
    elseif inst.target(it).vel_handing == inst.velParams.vel_rate_filter
        % 速度滤波状态
        instanteneousRangeRate = (um(1) - inst.target(it).allocationRange) / ...
            (inst.target(it).heartBeatCount - inst.target(it).allocationTime) * inst.dt;
        % 低通滤波更新速度
        inst.target(it).rangeRate = inst.velParams.alpha * inst.target(it).rangeRate + ...
            (1 - inst.velParams.alpha) * instanteneousRangeRate;
        % um(3) = unroll_radvel(inst.velParams.maxradvel, inst.target(it).rangeRate, rvIn);
        um(3) = rvIn;
        % 计算速度误差
        rrError = (instanteneousRangeRate - inst.target(it).rangeRate) / inst.target(it).rangeRate;
        % 如果误差小于阈值，进入跟踪状态
        if abs(rrError) < inst.velParams.confidence
            inst.target(it).vel_handing = inst.velParams.vel_tracking;
        end
    elseif inst.target(it).vel_handing == inst.velParams.vel_tracking
        % 速度跟踪状态
        instanteneousRangeRate = (um(1) - inst.target(it).allocationRange) / ...
            (inst.target(it).heartBeatCount - inst.target(it).allocationTime) * inst.dt;
        % 低通滤波更新速度
        inst.target(it).rangeRate = inst.velParams.alpha * inst.target(it).rangeRate + ...
            (1 - inst.velParams.alpha) * instanteneousRangeRate;
        % um(3) = unroll_radvel(inst.velParams.maxradvel, inst.target(it).rangeRate, rvIn);
        um(3) = rvIn;
        % 计算速度误差
        rvError = (inst.target(it).Hs(3) - um(3)) / um(3);
        % 如果误差小于阈值，进入锁定状态
        if abs(rvError) < 0.1
            inst.target(it).vel_handing = inst.velParams.vel_locked;
        end
    elseif inst.target(it).vel_handing == inst.velParams.vel_locked
        % 速度锁定状态
        % um(3) = unroll_radvel(inst.velParams.maxradvel, inst.target(it).Hs(3), um(3));
        um(3) = rvIn;
    end
end

% =========================================================================
% 目标状态管理函数
% =========================================================================
% 功能说明：
%   该函数管理目标的状态转换，包括：
%   1. 检测状态 → 激活状态：当目标持续被检测到时
%   2. 检测状态 → 自由状态：当目标长时间未被检测到时
%   3. 激活状态 → 自由状态：当目标长时间未被检测到时
% 
% 输入参数：
%   inst - 跟踪器实例
%   it - 目标索引
%   pointNum - 关联的点云数量
% 
% 输出参数：
%   inst - 更新后的跟踪器实例
% =========================================================================

function inst = event(inst, it, pointNum)
    % 更新目标的点云数量
    inst.target(it).point_num = pointNum;
    
    % 处理检测状态的目标
    if inst.target(it).state == inst.stateParams.detection
        if pointNum > inst.stateParams.pointsThre
            % 点云数量超过阈值，增加激活计数
            inst.target(it).detect2freeCount = 0;
            inst.target(it).detect2activeCount = inst.target(it).detect2activeCount + 1;
            % 如果激活计数超过阈值，转换为激活状态
            if inst.target(it).detect2activeCount > inst.stateParams.det2actThre
                inst.target(it).state = inst.stateParams.active;
            end
        else
            if pointNum == 0
                % 无点云关联，增加自由计数，减少激活计数
                inst.target(it).detect2freeCount = inst.target(it).detect2freeCount + 1;
                if inst.target(it).detect2activeCount > 0
                    inst.target(it).detect2activeCount = inst.target(it).detect2activeCount - 1;
                end
                % 如果自由计数超过阈值，转换为自由状态
                if inst.target(it).detect2freeCount > inst.stateParams.det2freeThre
                    inst.target(it).state = inst.stateParams.free;
                end
            end
        end
    % 处理激活状态的目标
    elseif inst.target(it).state == inst.stateParams.active
        inst.target(it).age = inst.target(it).age + 1;  % 增加目标年龄
        if pointNum ~= 0
            % 有点云关联，重置自由计数
            inst.target(it).active2freeCount = 0;
        else
            % 无点云关联，增加自由计数
            inst.target(it).active2freeCount = inst.target(it).active2freeCount + 1;
            % 计算自由阈值（不超过心跳计数）
            thre = inst.stateParams.act2freeThre;
            if thre > inst.target(it).heartBeatCount
                thre = inst.target(it).heartBeatCount;
            end
            % 如果自由计数超过阈值，转换为自由状态
            if inst.target(it).active2freeCount > thre
                inst.target(it).state = inst.stateParams.free;
            end
        end
    end
end