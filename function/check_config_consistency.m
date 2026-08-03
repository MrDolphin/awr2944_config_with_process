% =========================================================================
% 检查配置一致性函数
% =========================================================================
function check_config_consistency(config)
    fprintf('\n=== 配置一致性检查 ===\n');

    % 检查虚拟天线数量是否正确
    expected_virt_ant = config.NUM_TX * config.NUM_RX;
    if config.VIRT_ANT ~= expected_virt_ant
        warning('虚拟天线数配置不一致: 配置=%d, 预期=%d (TX×RX=%d×%d)', ...
            config.VIRT_ANT, expected_virt_ant, config.NUM_TX, config.NUM_RX);
        config.VIRT_ANT = expected_virt_ant;
        fprintf('已自动修正虚拟天线数为: %d\n', config.VIRT_ANT);
    else
        fprintf('虚拟天线数: %d (正确)\n', config.VIRT_ANT);
    end

    % 检查数据格式
    if isfield(config, 'DATA_FORMAT')
        fprintf('数据格式: %s\n', config.DATA_FORMAT);
    else
        fprintf('数据格式: 未指定 (默认假设为复数数据)\n');
    end

    fprintf('配置检查完成\n');
end
