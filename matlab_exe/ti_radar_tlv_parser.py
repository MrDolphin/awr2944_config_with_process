import serial
import time
import struct
import argparse
import sys

# TI 毫米波雷达的“魔法词”（数据包头的起始标志）
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

def send_config_to_radar(cfg_port_name, config_file_path):
    """
    通过 CFG 串口向雷达下发配置命令（启动雷达）
    """
    print(f"正在打开配置串口 {cfg_port_name} (波特率: 115200)...")
    try:
        # CFG 端口波特率固定为 115200
        # 将 timeout 降为 0.1 秒，避免 readlines() 对每一条指令死等
        cfg_port = serial.Serial(cfg_port_name, 115200, timeout=0.1)
    except Exception as e:
        print(f"❌ 无法打开配置串口 {cfg_port_name}。权限不够(需 sudo)或被占用？\n错误: {e}")
        return False

    try:
        with open(config_file_path, 'r') as f:
            lines = f.readlines()
            
        print(f"成功读取配置文件 {config_file_path}，开始下发指令...")
        for line in lines:
            line = line.strip()
            # 忽略空行和以 % 开头的注释
            if line == "" or line.startswith("%"):
                continue
                
            print(f"发送指令: {line}")
            # 必须以 \r 结尾
            cfg_port.write((line + '\r').encode('utf-8'))
            time.sleep(0.05) # 短暂延时等待芯片处理
            
            # 读取雷达的回显和 Ack
            response = cfg_port.readlines()
            for r in response:
                try:
                    print(f" => 雷达回复: {r.decode('utf-8').strip()}")
                except UnicodeDecodeError:
                    pass
                
        print("\n✅ 雷达配置下发完毕! 传感器现在应该已经开始发射雷达波了。\n")
        time.sleep(1) 
        cfg_port.close()
        return True
    except FileNotFoundError:
        print(f"❌ 找不到配置文件: {config_file_path}")
        cfg_port.close()
        return False
    except Exception as e:
        print(f"❌ 发送配置时发生错误: {e}")
        cfg_port.close()
        return False

def parse_radar_data(data_port_name, baud_rate=921600):
    """
    连接雷达的数据串口并解析 TLV 点云数据
    """
    print(f"试图连接数据串口 {data_port_name} (波特率: {baud_rate})...")
    
    try:
        data_port = serial.Serial(data_port_name, baud_rate, timeout=1)
        print("✅ 成功打开数据串口！正在监听雷达数据流...\n")
    except Exception as e:
        print(f"❌ 无法打开数据串口 {data_port_name}。")
        print(f"错误信息: {e}")
        return

    buffer = bytearray()
    
    try:
        while True:
            # 持续读取串口数据到缓冲区
            buffer.extend(data_port.read(data_port.in_waiting or 1))
            
            # 寻找魔法词同步数据帧
            magic_idx = buffer.find(MAGIC_WORD)
            
            if magic_idx == -1:
                if len(buffer) > 2048:
                    buffer = buffer[-2048:]
                continue
                
            if magic_idx > 0:
                buffer = buffer[magic_idx:]
            
            # 检查是否有足够的长度包含 Header (通常是 40 字节)
            if len(buffer) < 40:
                continue
                
            try:
                # 解析 Frame Header (假设 40 bytes)
                header_data = struct.unpack('<8sIIIIIIII', buffer[:40])
                magic, version, total_packet_len, platform, frame_num, time_cpu, num_obj, num_tlvs, sub_frame_num = header_data
            except struct.error:
                continue
                
            if len(buffer) < total_packet_len:
                continue
                
            print(f"==== 接收到第 {frame_num} 帧数据包 ====")
            print(f"数据包总长度: {total_packet_len} bytes, 含 TLV: {num_tlvs} 个, 杂点云/目标总计: {num_obj} 个")
            
            frame_data = buffer[40:total_packet_len]
            # 从缓冲区移除已处理的帧
            buffer = buffer[total_packet_len:]
            
            # --- 解析 TLV 数据块 ---
            offset = 0
            for i in range(num_tlvs):
                if offset + 8 > len(frame_data):
                    break
                    
                tlv_type, tlv_length = struct.unpack('<II', frame_data[offset:offset+8])
                offset += 8
                tlv_payload = frame_data[offset:offset+tlv_length]
                offset += tlv_length
                
                # Type 1: Point Cloud
                if tlv_type == 1:
                    print(f"  -> 📦 [类型 1] 点云数据块 (大小: {tlv_length} bytes)")
                    # 简单演示拆解坐标（假设最基本的 point struct 共16字节，如果乱码请依据 AWR2944 手册调整 bytes）
                    if num_obj > 0 and tlv_length >= num_obj * 16:
                        for obj_i in range(min(num_obj, 3)): # 最多只打印前 3 个点防止刷屏
                            x, y, z, v = struct.unpack('<ffff', tlv_payload[obj_i*16:(obj_i+1)*16])
                            print(f"     点 {obj_i+1}: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m, 速度={v:.2f}m/s")
                # Type 7 or 10: Target List
                elif tlv_type == 7 or tlv_type == 10:
                    print(f"  -> 🎯 [类型 {tlv_type}] 目标跟踪轨迹块 (大小: {tlv_length} bytes)")
                else:
                    print(f"  -> 📄 [类型 {tlv_type}] 其他衍生数据块 (大小: {tlv_length} bytes)")
            
            print("==================================\n")
            
    except KeyboardInterrupt:
        print("您已通过键盘中断了程序。")
    finally:
        data_port.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="树莓派雷达控制器与 TLV 读取脚本")
    # 修改默认值为树莓派常见的串口名称
    parser.add_argument('--cfg_port', type=str, default='/dev/ttyACM0', help='CFG 配置串口 (Linux通常为 ttyACM0)')
    parser.add_argument('--data_port', type=str, default='/dev/ttyACM1', help='Data 数据串口 (Linux通常为 ttyACM1)')
    parser.add_argument('--baud', type=int, default=921600, help='波特率 (默认: 921600)')
    # 新增对配置文件的支持
    parser.add_argument('--cfg_file', type=str, default='', help='(可选) profile.cfg 的路径。如果提供，脚本会自动唤醒并启动雷达。')
    args = parser.parse_args()
    
    # 步骤 1：下发配置文件唤醒雷达
    if args.cfg_file:
        success = send_config_to_radar(args.cfg_port, args.cfg_file)
        if not success:
            print("配置下发失败，退出程序。")
            sys.exit(1)
            
    # 步骤 2：监听数据
    parse_radar_data(args.data_port, args.baud)
