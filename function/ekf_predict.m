% =========================================================================
% EKF 状态预测函数
% =========================================================================
% 功能说明：
%   该函数使用扩展卡尔曼滤波器（EKF）预测每个目标的下一状态
%   包括：
%   1. 状态预测：使用状态转移矩阵预测目标的下一状态
%   2. 协方差预测：预测下一状态的协方差矩阵
%   3. 协方差对称化：确保协方差矩阵的对称性，提高算法稳定性
%   4. 坐标系转换：将预测状态从直角坐标系转换为极坐标系
% 
% 输入参数：
%   inst - 跟踪器实例，包含当前目标状态
% 
% 输出参数：
%   inst - 更新后的跟踪器实例，包含预测的目标状态
% 
% 状态向量说明：
%   - S_hat: 当前状态向量 [x, y, vx, vy, ax, ay]'
%   - S_apriori_hat: 预测的下一状态向量
%   - P_hat: 当前状态协方差矩阵
%   - P_apriori_hat: 预测的下一状态协方差矩阵
%   - Hs: 极坐标系状态 [range, angle, vel]'
% =========================================================================

function inst = ekf_predict(inst)
    % 检查是否有目标需要预测
    if(~isempty(inst.target))
        % 遍历每个目标
        for i = 1:length(inst.target)
            % 注释掉的心跳计数更新
            % inst.target(i).heartBeatCount = inst.target(i).heartBeatCount + 1;
            
            % =================================================================
            % 1. 状态预测
            % =================================================================
            % 使用状态转移矩阵 F 预测目标的下一状态
            inst.target(i).S_apriori_hat = inst.F * inst.target(i).S_hat;
            
            % =================================================================
            % 2. 协方差预测
            % =================================================================
            % 预测下一状态的协方差矩阵
            inst.target(i).P_apriori_hat = inst.F * inst.target(i).P_hat * inst.F' + inst.Q * inst.processVariance;
            
            % =================================================================
            % 3. 协方差对称化
            % =================================================================
            % 将协方差矩阵对称化，提升算法稳定性
            inst.target(i).P_apriori_hat = 0.5 * (inst.target(i).P_apriori_hat + inst.target(i).P_apriori_hat');
            
            % =================================================================
            % 4. 坐标系转换（直角坐标系转极坐标系）
            % =================================================================
            % 极坐标系中 Hs = [range angle vel]
            posx = inst.target(i).S_apriori_hat(1);  % 预测的X坐标
            posy = inst.target(i).S_apriori_hat(2);  % 预测的Y坐标
            velx = inst.target(i).S_apriori_hat(3);  % 预测的X方向速度
            vely = inst.target(i).S_apriori_hat(4);  % 预测的Y方向速度
            
            % 计算距离（极径）
            inst.target(i).Hs(1) = sqrt(posx^2 + posy^2);
            
            % 计算角度（极角），处理特殊情况
            if posy == 0
                inst.target(i).Hs(2) = pi/2;  % 当y=0时，角度为90度
            elseif posy > 0
                inst.target(i).Hs(2) = atan(posx/posy);  % 当y>0时，直接计算反正切
            else
                inst.target(i).Hs(2) = atan(posx/posy) + pi/2;  % 当y<0时，调整角度
            end
            
            % 计算径向速度
            % 径向速度 = (x*vx + y*vy) / 距离
            inst.target(i).Hs(3) = (posx*velx + posy*vely) / inst.target(i).Hs(1);
        end
    end
end