% =========================================================================
% 离线二进制 UART 雷达点云解析器 (专为 AWR2944 及同类 TI 雷达设计)
% =========================================================================
function data = parse_uart_bin(fileName)
    fprintf('正在读取二进制数据文件: %s\n', fileName);
    
    fid = fopen(fileName, 'rb');
    if fid == -1
        error(['无法打开文件: ', fileName]);
    end
    % 以无符号8位整型读取全量二进制数据
    rawBytes = fread(fid, inf, 'uint8');
    fclose(fid);
    
    % TI 官方规定的 Magic Word 同步字序列
    MAGIC_WORD = [2, 1, 4, 3, 6, 5, 8, 7]'; 
    
    % 在全量内存中极速寻找所有魔法字的索引起点
    fprintf('正在扫描 Magic Word 同步头...\n');
    idxList = strfind(rawBytes', MAGIC_WORD')';
    
    % 初始化预分配结构体数组
    data = struct('frame_num', {}, 'num_obj', {}, 'x', {}, 'y', {}, 'z', {}, 'v', {});
    frameCount = 0;
    
    for i = 1:length(idxList)
        idx = idxList(i);
        if idx + 40 - 1 > length(rawBytes)
            continue; % 数据截断，跳过
        end
        
        % 解析 40 字节的核心头部 (Header)
        headerBytes = rawBytes(idx : idx+39);
        total_packet_len = typecast(uint8(headerBytes(13:16)), 'uint32');
        frame_num = typecast(uint8(headerBytes(21:24)), 'uint32');
        num_obj = typecast(uint8(headerBytes(29:32)), 'uint32');
        num_tlvs = typecast(uint8(headerBytes(33:36)), 'uint32');
        
        if idx + total_packet_len - 1 > length(rawBytes)
            continue; % 本帧尚未落盘完毕，舍弃
        end
        
        offset = idx + 40; % 指向第一个 TLV
        
        curr_x = [];
        curr_y = [];
        curr_z = [];
        curr_v = [];
        
        % 开始循环解析该帧内所有的 TLV (点云、热力图、噪声图等)
        for t = 1:num_tlvs
            if offset + 8 - 1 > length(rawBytes)
                break;
            end
            tlv_type = typecast(uint8(rawBytes(offset : offset+3)), 'uint32');
            tlv_len = typecast(uint8(rawBytes(offset+4 : offset+7)), 'uint32');
            offset = offset + 8;
            
            % TLV Type 1 就是散点云 (Point Cloud)
            if tlv_type == 1 
                payload = rawBytes(offset : offset + tlv_len - 1);
                pointCount = floor(double(tlv_len) / 16); % AWR2944 中每点占 16 字节
                for p = 1:min(double(num_obj), pointCount)
                    p_off = (p-1)*16 + 1;
                    % ffff -> 用 IEEE 754 float 解析
                    curr_x(end+1) = typecast(uint8(payload(p_off : p_off+3)), 'single');
                    curr_y(end+1) = typecast(uint8(payload(p_off+4 : p_off+7)), 'single');
                    curr_z(end+1) = typecast(uint8(payload(p_off+8 : p_off+11)), 'single');
                    curr_v(end+1) = typecast(uint8(payload(p_off+12 : p_off+15)), 'single');
                end
            % 在这里您可以自己依样画葫芦扩写：
            % if tlv_type == 5  (Range-Doppler Heatmap) 即可解包热力图！
            end
            
            % 跳过当前 TLV payload 前往下一个 TLV
            offset = offset + double(tlv_len);
        end
        
        frameCount = frameCount + 1;
        data(frameCount).frame_num = frame_num;
        data(frameCount).num_obj = length(curr_x);
        data(frameCount).x = curr_x;
        data(frameCount).y = curr_y;
        data(frameCount).z = curr_z;
        data(frameCount).v = curr_v;
    end
    
    fprintf('✅ 解析大功告成！共计获得完整数据帧: %d 帧\n', frameCount);
    
    % ==== 自动可视化演示 ====
    % 为了方便你们直观评测，此处加入了 2D 点云逐帧动画播放器
    figure('Name', '雷达检测离线轨迹实录', 'Color', 'w');
    for i = 1:length(data)
        if data(i).num_obj > 0
            % X: 左右, Y: 前后(深度)
            scatter(data(i).x, data(i).y, 30, data(i).v, 'filled'); % 颜色映射表示多普勒速度
            colormap parula; cbar = colorbar; ylabel(cbar, '多普勒速度 (m/s)');
            caxis([-1.5 1.5]); % 速度色阶卡范围
            axis([-5 5 0 10]); grid on;
            xlabel('X 轴横向宽度 (m)'); ylabel('Y 轴正前方距离 (m)');
            title(sprintf('帧号: %d | 存活反射散点数量: %d', data(i).frame_num, data(i).num_obj));
            drawnow;
        end
        pause(0.005);
    end
end
