%--------------------------------------------------------------------------
% 函数名称: visualize_results_simple
% 函数功能: 雷达数据和目标跟踪结果的简化版可视化
% 输入参数:
%   - inst: 包含跟踪结果的结构体
%   - frame_idx: 当前帧索引
%   - range_azimuth: 距离-角度谱数据
%   - peaks: CFAR检测峰值
%   - t_num: 目标数量（未使用，保留接口一致性）
%   - config: 雷达配置参数（未使用，保留接口一致性）
%   - ranges_m: 检测点的距离（米）
%   - azimuths_rad: 检测点的方位角（弧度）
%   - radar_cube: 雷达原始数据立方体
%   - doppler_spectrum: 多普勒谱数据 [num_chirps, num_samples]（新增）
%   - save_video: 是否保存视频（可选，默认为false）
%   - video_writer: 视频写入对象（可选）
% 输出参数:
%   - video_writer: 更新后的视频写入对象
% 函数原理:
%   创建一个2x2的子图布局，分别显示：
%   1. 距离FFT结果（使用radar_cube）
%   2. 距离-角度谱和检测峰值
%   3. RX0通道的速度谱（距离-多普勒）- 直接使用已计算的多普勒谱
%   4. 目标跟踪结果（含朝向箭头和历史轨迹）
%   5. 可选：将每一帧保存到视频文件
%--------------------------------------------------------------------------
function video_writer = visualize_results_simple(inst, frame_idx, range_azimuth, peaks, t_num, config, ranges_m, azimuths_rad, radar_cube, doppler_spectrum, save_video, video_writer)
    % 处理可选参数
    if nargin < 11 || isempty(save_video)
        save_video = false;
    end
    if nargin < 12
        video_writer = [];
    end
    
    % 确保图形窗口存在
    if ~ishandle(1)
        figure(1);
    end
    fig = gcf;
    set(fig, 'Name', 'Radar Results');
    
    % 清除所有子图内容
    clf;
    
    %----------------------------------------------------------------------
    % 第一子图：Range FFT（距离FFT）- 保持原始处理方式
    %----------------------------------------------------------------------
    subplot(2, 2, 1);
    if exist('radar_cube', 'var') && ~isempty(radar_cube) && size(radar_cube, 1) >= 1 && size(radar_cube, 3) >= 1
        try
            % 计算距离FFT（取第一个chirp和第一个天线的数据）
            range_fft = squeeze(radar_cube(1, 1, :));
            
            % 创建距离轴（米）
            range_axis_m = (1:length(range_fft)) * config.RANGE_RESOLUTION;
            
            % 绘制距离FFT（转换为dB单位）
            plot(range_axis_m, 10*log10(abs(range_fft) + eps), 'LineWidth', 2);
            title(sprintf('Range FFT (Frame %d)', frame_idx));
            xlabel('Range (m)');
            ylabel('Power (dB)');
            ylim([10,50]);
            xlim([0, config.RANGE_RESOLUTION * 130]);
            grid on;
        catch ME
            text(0.5, 0.5, 'Error plotting Range FFT', 'Units', 'normalized', ...
                'HorizontalAlignment', 'center', 'FontSize', 10);
            title('Range FFT Error');
        end
    else
        % 无数据时显示提示信息
        text(0.5, 0.5, 'No Range FFT data', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'FontSize', 10);
        title('Range FFT');
    end
    
    %----------------------------------------------------------------------
    % 第二子图：角度-距离谱（修改：Range Bin -> Range (m)）
    %----------------------------------------------------------------------
    subplot(2, 2, 2);
    if exist('range_azimuth', 'var') && ~isempty(range_azimuth)
        try
            % 创建距离轴（米）
            [num_angles, num_range_bins] = size(range_azimuth);
            range_axis_m = (1:num_range_bins) * config.RANGE_RESOLUTION;
            
            % 创建角度轴（度）
            angle_axis_deg = linspace(-config.ANGLE_RANGE/2, config.ANGLE_RANGE/2, num_angles);
            
            % 绘制距离-角度谱 - 横坐标为距离，纵坐标为角度
            imagesc(range_axis_m, angle_axis_deg, log2(range_azimuth + eps));
            colormap('jet');
            colorbar;
            clim([20,35]);
            title(sprintf('Range-Azimuth Spectrum (Frame %d)', frame_idx));
            xlabel('Range (m)');
            ylabel('Azimuth (deg)');
            axis xy;
            
            % 标记检测峰值
            if exist('peaks', 'var') && ~isempty(peaks)
                hold on;
                [azimuth_peaks, range_peaks] = find(peaks);
                if ~isempty(azimuth_peaks)
                    % 将峰值索引转换为物理单位
                    range_peaks_m = range_peaks * config.RANGE_RESOLUTION;
                    angle_peaks_deg = (azimuth_peaks - (num_angles + 1)/2) * config.ANGLE_RES;
                    plot(range_peaks_m, angle_peaks_deg, 'r+', 'MarkerSize', 8, 'LineWidth', 2);
                end
                hold off;
            end
        catch ME
            text(0.5, 0.5, 'Error plotting Azimuth-Range', 'Units', 'normalized', ...
                'HorizontalAlignment', 'center', 'FontSize', 10);
            title('Range-Azimuth Error');
        end
    % else
    %     text(0.5, 0.5, 'No Range-Azimuth data', 'Units', 'normalized', ...
    %         'HorizontalAlignment', 'center', 'FontSize', 10);
    %     title('Range-Azimuth Spectrum');
    % end
    
    %----------------------------------------------------------------------
    % 第三子图：速度谱（距离-多普勒）- 修改：Range Bin -> Range (m)
    %----------------------------------------------------------------------
    subplot(2, 2, 3);
    if exist('doppler_spectrum', 'var') && ~isempty(doppler_spectrum) && ndims(doppler_spectrum) >= 2
        try
            % 获取维度
            [num_chirps, num_samples] = size(doppler_spectrum);
            
            if num_chirps > 1
                % 创建距离轴（米）
                range_axis_m = (1:num_samples) * config.RANGE_RESOLUTION;
                
                % 创建速度轴（米/秒）
                vel_axis = linspace(-num_chirps/2 * config.DOPPLER_RESOLUTION, ...
                                    num_chirps/2 * config.DOPPLER_RESOLUTION, ...
                                    num_chirps);
                
                % 计算功率谱（dB单位）
                doppler_spectrum_db = 20*log10(abs(doppler_spectrum) + eps);
                
                % 绘制速度谱
                imagesc(range_axis_m, vel_axis, doppler_spectrum_db);
                colormap(gca, 'jet');
                colorbar;
                clim([90,120]);
                
                % 设置坐标轴标签
                xlabel('Range (m)');
                ylabel('Velocity (m/s)');
                
                title(sprintf('Doppler-Range Spectrum (Frame %d)', frame_idx));
                axis xy;
                
                % 标记检测到的目标在速度谱上的位置
                if exist('ranges_m', 'var') && ~isempty(ranges_m) && exist('azimuths_rad', 'var')
                    hold on;
                    
                    % 从inst中提取已检测目标的速度
                    if isfield(inst, 'point_cloud') && ~isempty(inst.point_cloud)
                        for i = 1:min(length(inst.point_cloud), 10)  % 最多显示10个点避免混乱
                            % if isfield(inst.point_cloud(i), 'range') && isfield(inst.point_cloud(i), 'vel')
                            %     % 在检测点位置绘制白色圆圈
                            %     plot(inst.point_cloud(i).range, inst.point_cloud(i).vel, ...
                            %         'wo', 'MarkerSize', 8, 'LineWidth', 2);
                            % end
                        end
                    end
                    hold off;
                end
            else
                text(0.5, 0.5, sprintf('Insufficient chirps: %d', num_chirps), ...
                    'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 10);
                title('Doppler-Range Spectrum');
            end
            
        catch ME
            text(0.5, 0.5, sprintf('Error: %s', ME.message), ...
                'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 8);
            title('Doppler Error');
        end
    else
        % 无数据时显示提示信息
        text(0.5, 0.5, 'No Doppler data available', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'FontSize', 10);
        title('Doppler-Range Spectrum');
    end
    
    %----------------------------------------------------------------------
    % 第四子图：跟踪效果（包含历史轨迹）
    %----------------------------------------------------------------------
    subplot(2, 2, 4);

    % 使用持久变量保存历史轨迹和上一帧号
    persistent history_trajectories max_history_frames last_frame

    % 初始化持久变量
    if isempty(max_history_frames)
        max_history_frames = 100;  % 保留100帧历史轨迹
    end

    % 检查是否为新的一次运行（帧号重置检测）
    current_frame = frame_idx;
    if isempty(last_frame) || current_frame < last_frame
        % 帧号变小，说明是新的运行开始，清空历史轨迹
        history_trajectories = [];
        last_frame = current_frame;
        fprintf('检测到新的一次运行（帧号从%d变为%d），历史轨迹已清空\n', last_frame, current_frame);
    else
        % 正常帧号递增，更新上一帧号
        last_frame = current_frame;
    end

    % 初始化历史轨迹（如果为空）
    if isempty(history_trajectories)
        history_trajectories = struct('trackID', {}, 'positions', {}, 'frames', {});
    end

    % 清理超过最大历史帧数的轨迹点
    for i = length(history_trajectories):-1:1
        if ~isempty(history_trajectories(i).frames)
            % 保留最近max_history_frames帧内的点
            valid_idx = history_trajectories(i).frames >= (current_frame - max_history_frames);
            history_trajectories(i).positions = history_trajectories(i).positions(valid_idx, :);
            history_trajectories(i).frames = history_trajectories(i).frames(valid_idx);

            % 如果轨迹为空，删除该轨迹记录
            if isempty(history_trajectories(i).positions)
                history_trajectories(i) = [];
            end
        end
    end

    % 更新当前帧的目标位置到历史轨迹
    if isfield(inst, 'target') && ~isempty(inst.target)
        for i = 1:length(inst.target)
            target = inst.target(i);
            if isfield(target, 'S_hat') && ~isempty(target.S_hat) && length(target.S_hat) >= 2
                track_id = target.trackID;
                current_pos = [target.S_hat(1), target.S_hat(2)];

                % 查找是否已存在该目标的轨迹
                found = false;
                for j = 1:length(history_trajectories)
                    if history_trajectories(j).trackID == track_id
                        % 添加当前位置到历史轨迹
                        history_trajectories(j).positions = [history_trajectories(j).positions; current_pos];
                        history_trajectories(j).frames = [history_trajectories(j).frames; current_frame];
                        found = true;
                        break;
                    end
                end

                % 如果是新目标，创建新的轨迹记录
                if ~found
                    new_traj.trackID = track_id;
                    new_traj.positions = current_pos;
                    new_traj.frames = current_frame;
                    if isempty(history_trajectories)
                        history_trajectories = new_traj;
                    else
                        history_trajectories(end+1) = new_traj;
                    end
                end
            end
        end
    end

    % 绘制跟踪结果和历史轨迹
    if isfield(inst, 'target') && ~isempty(inst.target)
        try
            % 从 inst 中提取当前帧目标信息
            x_pos = [];
            y_pos = [];
            ids = [];
            velocities = [];

            for i = 1:length(inst.target)
                target = inst.target(i);
                if isfield(target, 'S_hat') && ~isempty(target.S_hat) && length(target.S_hat) >= 2
                    x_pos = [x_pos; target.S_hat(1)];  % x位置
                    y_pos = [y_pos; target.S_hat(2)];  % y位置
                    ids = [ids; target.trackID];  % 使用trackID而不是索引

                    % 提取速度信息
                    if length(target.S_hat) >= 4
                        vx = target.S_hat(3);
                        vy = target.S_hat(4);
                        velocities = [velocities; vx vy];
                    else
                        velocities = [velocities; 0 0];
                    end
                end
            end

            if ~isempty(x_pos)
                hold on;

                % 首先绘制历史轨迹（使用渐变色显示时间远近）
                for i = 1:length(history_trajectories)
                    traj = history_trajectories(i);
                    if size(traj.positions, 1) >= 2  % 至少有两个点才绘制轨迹线
                        % 根据帧号计算透明度（越新的点越亮）
                        frame_ages = current_frame - traj.frames;
                        alpha_values = 1 - frame_ages / max_history_frames * 0.7;  % 透明度范围0.3-1.0

                        % 绘制轨迹线
                        for j = 1:size(traj.positions, 1)-1
                            % 线段颜色随新旧程度变化
                            alpha = alpha_values(j);
                            line_color = [0.5 0.5 0.5] * alpha + [0.5 0.5 0.5] * (1-alpha);
                            plot(traj.positions(j:j+1, 1), traj.positions(j:j+1, 2), ...
                                'Color', line_color, 'LineWidth', 1.5, 'LineStyle', '-');
                        end

                        % 绘制历史位置点（用淡灰色圆点表示）
                        scatter(traj.positions(:, 1), traj.positions(:, 2), 20, ...
                            [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.3);
                    end
                end

                % 绘制当前帧的目标点（红色填充圆）
                scatter(x_pos, y_pos, 100, 'r', 'filled');

                % 添加ID标签和朝向箭头
                for i = 1:length(x_pos)
                    % 添加ID标签
                    text(x_pos(i), y_pos(i) + 0.15, sprintf('ID:%d', ids(i)), ...
                        'FontSize', 9, 'FontWeight', 'bold', ...
                        'HorizontalAlignment', 'center', ...
                        'BackgroundColor', 'w', 'EdgeColor', 'k');

                    % 计算并绘制朝向箭头
                    if i <= size(velocities, 1)
                        vx = velocities(i, 1);
                        vy = velocities(i, 2);
                        speed = sqrt(vx^2 + vy^2);

                        % 如果速度大于阈值，绘制箭头
                        if speed > 0.05
                            % 归一化速度向量并缩放
                            arrow_length = 0.4; % 箭头长度
                            dx = (vx / speed) * arrow_length;
                            dy = (vy / speed) * arrow_length;

                            % 绘制箭头（绿色，加粗，大箭头头部）
                            quiver(x_pos(i), y_pos(i), dx, dy, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.0);
                        end
                    end
                end
                hold off;

                title(sprintf('Tracked Targets: %d (History: %d frames)', ...
                    length(x_pos), max_history_frames));
            else
                text(0.5, 0.5, 'No tracked targets', 'Units', 'normalized', ...
                    'HorizontalAlignment', 'center', 'FontSize', 11);
                title('Tracked Targets: 0');
            end
        catch ME
            text(0.5, 0.5, 'Error plotting targets', 'Units', 'normalized', ...
                'HorizontalAlignment', 'center', 'FontSize', 10);
            title('Tracking Error');
        end
    else
        %无目标数据时显示提示信息
        text(0.5, 0.5, 'No tracked targets', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center', 'FontSize', 11);
        title('Tracked Targets: 0');
    end

    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;
    xlim([-3, 3]);  % x轴范围
    ylim([0, 5]);   % y轴范围

    % 刷新显示
    drawnow;  
 

    % 保存视频帧
    if save_video
        try
            if isempty(video_writer) || ~isvalid(video_writer)
                % 创建视频写入对象
                video_filename = sprintf('radar_tracking_%s.avi', datestr(now, 'yyyyMMdd_HH-mm-ss'));
                video_writer = VideoWriter(video_filename, 'Motion JPEG AVI');
                video_writer.FrameRate = 5;  % 设置帧率
                open(video_writer);
                fprintf('开始保存视频到 %s\n', video_filename);
            end
            
            % 获取当前帧
            frame = getframe(fig);
            writeVideo(video_writer, frame);
        catch ME
            fprintf('视频保存出错: %s\n', ME.message);
        end
    end
    
end